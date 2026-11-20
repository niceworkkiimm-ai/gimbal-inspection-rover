#include "ReturnPath.h"
#include "WheelSpeedConfig.h"
#include "CarConfig.h"
#include <string.h>

static const int8_t signs[4]=WHEEL_ENCODER_SIGNS;
static int32_t Delta(uint32_t a,uint32_t b) {
    uint32_t d=a-b;
    return d<=0x7fffffffU?(int32_t)d:(int32_t)(-((int64_t)(~d)+1));
}
static int64_t Abs(int64_t x){return x<0?-x:x;}
static void Zero(ReturnPath *s){memset(s->demand,0,sizeof(s->demand));}
void ReturnPath_Init(ReturnPath *s){memset(s,0,sizeof(*s));}
void ReturnPath_Abort(ReturnPath *s,uint8_t reason){
    if(s->state==RETURN_RECORDING||s->state==RETURN_RUNNING){
        s->state=RETURN_ABORTED;s->reason=reason;s->ready=0;Zero(s);
    }
}
void ReturnPath_CountFault(ReturnPath *s, uint8_t wheels)
{
    if (wheels && (s->state == RETURN_RECORDING || s->state == RETURN_RUNNING)) {
        s->count_fault_wheels = wheels;
        ReturnPath_Abort(s, RETURN_COUNTS);
    }
}
static uint8_t Save(ReturnPath *s,const uint32_t counts[4],uint32_t now){
    if(s->count>=RETURN_CAPACITY){ReturnPath_Abort(s,RETURN_FULL);return 0;}
    memcpy(s->point[s->count++],counts,sizeof(s->previous));s->saved_ms=now;return 1;
}
void ReturnPath_Command(ReturnPath *s,uint32_t boot,uint32_t request,
                        uint8_t action,uint32_t task,uint32_t now){
    uint32_t delta=request-s->ack;
    /* A stop from this boot always wins, including a stale tab's request ID.
       It must never be rejected merely because another tab advanced the ACK. */
    if(s->boot&&boot==s->boot&&request&&action==RETURN_CANCEL){
        ReturnPath_Abort(s,RETURN_STOP);s->stop_requested=1;
    }
    /* Wrong boot and old/duplicate requests cannot mutate state or renew a lease.
       Keep the last ACK intact so a lost HTTP response can be retried safely. */
    if(!s->boot||boot!=s->boot||!request||!delta||delta>=0x80000000U)return;
    s->ack=request;s->command_boot=boot;s->command_task=task;s->command_action=action;
    s->result=RETURN_ACCEPTED;
    if(action==RETURN_CANCEL)return;
    if(s->state==RETURN_RUNNING){s->result=RETURN_BUSY;return;}
    if(!s->ready||(uint32_t)(now-s->last_ms)>120U){s->result=RETURN_NOT_READY;return;}
    if(action==RETURN_SET){
        s->count=0;s->index=0;s->task=request;s->state=RETURN_RECORDING;
        memset(s->travel_sign,0,sizeof(s->travel_sign));
        s->count_fault_wheels=0;s->reason=RETURN_OK;s->pulse=0;Save(s,s->previous,now);return;
    }
    if(action!=RETURN_START||task!=s->task){s->result=RETURN_BAD_TASK;return;}
    if(s->state!=RETURN_RECORDING){s->result=RETURN_NO_PATH;return;}
    /* Include the exact parked endpoint, even below the sampling threshold. */
    if(memcmp(s->previous,s->point[s->count-1],sizeof(s->previous))&&
       !Save(s,s->previous,now)){s->result=RETURN_NO_PATH;return;}
    if(s->count<2){s->result=RETURN_NO_PATH;return;}
    s->planned_index=RETURN_CAPACITY;
    s->speed_limit=RETURN_APPROACH_RPM;
    s->index=(uint16_t)(s->count-2);s->state=RETURN_RUNNING;
    s->segment_ms=s->run_ms=s->lease_ms=now;s->pulse=0;s->settled=0;
}
void ReturnPath_Heartbeat(ReturnPath *s,uint32_t boot,uint32_t task,
                          uint32_t pulse,uint32_t now){
    uint32_t d=pulse-s->pulse;
    if(s->state==RETURN_RUNNING&&boot==s->boot&&task==s->task&&
       pulse&&d&&d<0x80000000U&&(uint32_t)(now-s->lease_ms)<=RETURN_BROWSER_TIMEOUT_MS){
        s->pulse=pulse;s->lease_ms=now;
    }
}
/* Logical wheel deltas: (left front, right front, left back, right back).
 * A repeatable wheel vector can represent an arc or a spin as well as a line.
 * For turning cruise, each side must move coherently, including a parked side. */
static uint8_t TravelCap(const int32_t v[4],int32_t peak)
{
    unsigned i;
    uint8_t straight=1;
    for(i=0;i<4;i++){
        if((v[i]>0)!=(v[0]>0) || Abs(v[i])*5<peak*4)straight=0;
    }
    if(straight)return RETURN_MAX_RPM;
    if(Abs(v[0]-v[2])*4>peak || Abs(v[1]-v[3])*4>peak)return 0;
    return RETURN_TURN_RPM;
}

/* Compare both adjacent segments in wheel-count space. Normalizing each by
 * its largest wheel displacement makes the check independent of point spacing.
 * A significant wheel start/stop, reversal or ratio change > 1/4 defines
 * a hard corner. Ambiguous motion within tick tolerance has no preview. */
static uint8_t PlanSegment(const ReturnPath *s, uint16_t index, int32_t advance[4])
{
    int32_t incoming[4], outgoing[4], in_peak=0, out_peak=0, horizon;
    unsigned i;
    uint8_t in_cap,out_cap;
    if (!index) return 0;
    for (i=0; i<4; ++i) {
        int32_t a=Delta(s->point[index][i],s->point[index+1][i]);
        int32_t b=Delta(s->point[index-1][i],s->point[index][i]);
        if (Abs(a)>2U*WHEEL_SPEED_CPR || Abs(b)>2U*WHEEL_SPEED_CPR) return 0;
        /* Near a pivot a stationary inner wheel may record 0, 1 or 2 ticks.
         * Keep that wheel's exact checkpoint target when its sign is ambiguous.
         * A significant start, stop or reversal still requires a hard corner. */
        if ((a>0)!=(b>0) || (a<0)!=(b<0)){
            if(Abs(a)>RETURN_TOLERANCE_TICKS || Abs(b)>RETURN_TOLERANCE_TICKS)return 0;
            a=b=0;
        }
        incoming[i]=a*signs[i];
        outgoing[i]=b*signs[i];
        if (Abs(a)>in_peak) in_peak=(int32_t)Abs(a);
        if (Abs(b)>out_peak) out_peak=(int32_t)Abs(b);
    }
    if (in_peak<=RETURN_TOLERANCE_TICKS || out_peak<=RETURN_TOLERANCE_TICKS) return 0;
    for (i=0; i<4; ++i) {
        int64_t drift=(int64_t)incoming[i]*out_peak-(int64_t)outgoing[i]*in_peak;
        if (Abs(drift)*4>(int64_t)in_peak*out_peak) return 0;
    }
    horizon=out_peak<RETURN_LOOKAHEAD_TICKS?out_peak:RETURN_LOOKAHEAD_TICKS;
    if (advance) {
        for (i=0; i<4; ++i)
            advance[i]=(int32_t)((int64_t)outgoing[i]*horizon/out_peak);
    }
    if(in_peak<RETURN_CRUISE_MIN_TICKS || out_peak<RETURN_CRUISE_MIN_TICKS)return 0;
    in_cap=TravelCap(incoming,in_peak);
    out_cap=TravelCap(outgoing,out_peak);
    return in_cap<out_cap?in_cap:out_cap;
}

void ReturnPath_Step(ReturnPath *s,uint32_t now,const uint32_t counts[4],
                     uint8_t healthy,uint8_t centered_ready){
    uint32_t dt=now-s->last_ms;
    uint8_t i,moved=0,save=0,reversal=0,at_target=1,bad_wheels=0;
    int32_t error[4],preview_peak=0;int64_t peak=0,d,limit;float rpm;
    Zero(s);
    if(!s->initialized){
        s->initialized=1;s->last_ms=s->changed_ms=now;
        memcpy(s->previous,counts,sizeof(s->previous));return;
    }
    if(!healthy)ReturnPath_Abort(s,RETURN_FEEDBACK);
    if(s->state==RETURN_RUNNING){
        if(!centered_ready)ReturnPath_Abort(s,RETURN_MANUAL);
        else if((uint32_t)(now-s->lease_ms)>RETURN_BROWSER_TIMEOUT_MS)ReturnPath_Abort(s,RETURN_BROWSER);
    }
    /* Every service observes counts, independently of the display journal.
       Never continue across a foreground stall or an implausible count jump. */
    limit=(int64_t)WHEEL_SPEED_CPR*180U*dt/60000U+4;
    for(i=0;i<4;i++){
        d=Delta(counts[i],s->previous[i]);if(d)moved=1;
        if(Abs(d)>limit)bad_wheels|=(uint8_t)(1U<<i);
        if(s->state==RETURN_RECORDING&&d){
            int8_t sign=d>0?1:-1;
            if(s->travel_sign[i]&&sign!=s->travel_sign[i])reversal=1;
            s->travel_sign[i]=sign;
        }
    }
    ReturnPath_CountFault(s,bad_wheels);
    if(dt>120U)ReturnPath_Abort(s,RETURN_GAP);
    if(s->state==RETURN_RECORDING&&reversal&&
       memcmp(s->previous,s->point[s->count-1],sizeof(s->previous)))Save(s,s->previous,now);
    if(moved||!centered_ready||!healthy)s->changed_ms=now;
    s->ready=(uint8_t)(healthy&&centered_ready&&s->boot&&
                       (uint32_t)(now-s->changed_ms)>=500U);
    s->last_ms=now;memcpy(s->previous,counts,sizeof(s->previous));
    if(s->state==RETURN_RECORDING){
        for(i=0;i<4;i++){
            d=Delta(counts[i],s->point[s->count-1][i]);
            if(Abs(d)>=16)save=1;
        }
        if((uint32_t)(now-s->saved_ms)>=200U&&
           memcmp(counts,s->point[s->count-1],sizeof(s->previous)))save=1;
        if(save)Save(s,counts,now);
        return;
    }
    if(s->state!=RETURN_RUNNING)return;
    if((uint32_t)(now-s->segment_ms)>8000U||(uint32_t)(now-s->run_ms)>600000U){
        ReturnPath_Abort(s,RETURN_TIMEOUT);return;
    }
    /* Points are immutable during return. Plan once per index, and require
     * a compatible joint ahead to leave a low-speed segment before a hard corner. */
    if(s->planned_index!=s->index){
        memset(s->advance,0,sizeof(s->advance));
        s->cruise_rpm=PlanSegment(s,s->index,s->advance);
        if(s->cruise_rpm){
            /* A nonzero cap implies index >= 1, since home has no outgoing leg. */
            uint8_t ahead=PlanSegment(s,(uint16_t)(s->index-1),NULL);
            if(ahead<s->cruise_rpm)s->cruise_rpm=ahead;
        }
        s->planned_index=s->index;
    }
    for(i=0;i<4;i++){
        d=(int64_t)Delta(s->point[s->index][i],counts[i])*signs[i];
        if(Abs(d)>WHEEL_SPEED_CPR*2U){bad_wheels|=(uint8_t)(1U<<i);continue;}
        /* Each wheel must reach this checkpoint in sequence. A wheel that
         * has crossed it may lead only as far as the bounded preview. */
        if (Abs(d)>RETURN_TOLERANCE_TICKS &&
            (d*s->advance[i]>=0 || Abs(d)>Abs(s->advance[i])+RETURN_TOLERANCE_TICKS))
            at_target=0;
        if(Abs(s->advance[i])>preview_peak)preview_peak=(int32_t)Abs(s->advance[i]);
        error[i]=(int32_t)d+s->advance[i];
        if(Abs(error[i])>peak)peak=Abs(error[i]);
    }
    if(bad_wheels){ReturnPath_CountFault(s,bad_wheels);return;}
    if(at_target){
        if(s->index){s->index--;s->segment_ms=now;s->settled=0;}
        else {
            if(!s->settled || moved){s->settled=1;s->settled_ms=now;}
            if((uint32_t)(now-s->settled_ms)>=400U){s->state=RETURN_DONE;s->reason=RETURN_OK;}
        }
        /* Keep driving toward the preview on this very call. At a hard
         * corner or home the preview is zero and all four outputs stay zero. */
        if(peak<=RETURN_TOLERANCE_TICKS || s->state!=RETURN_RUNNING){
            s->speed_limit=RETURN_APPROACH_RPM;
            return;
        }
    }
    s->settled=0;
    /* Common scaling preserves the four-wheel proportions through a pass point.
       The inner WheelSpeed PI handles speed feedback, slew and DIR dwell. */
    {
        uint8_t cruise=(uint8_t)(s->cruise_rpm && peak>=RETURN_CRUISE_MIN_TICKS);
        /* Compare each wheel against its expected share of the segment.
         * An inner wheel at one third of outer speed is normal on an arc. */
        for(i=0;i<4 && cruise;i++){
            int64_t expected=Abs(s->advance[i]);
            if(expected<=RETURN_TOLERANCE_TICKS && Abs(error[i])<=RETURN_TOLERANCE_TICKS)continue;
            if(!expected || (int64_t)error[i]*s->advance[i]<=0 ||
               Abs(error[i])*2*preview_peak<peak*expected ||
               Abs(error[i])*preview_peak>2*peak*expected)cruise=0;
        }
        if(cruise){
            s->speed_limit+=RETURN_CRUISE_RAMP_RPM_S*(float)dt/1000.0f;
            if(s->speed_limit>s->cruise_rpm)s->speed_limit=(float)s->cruise_rpm;
            rpm=s->speed_limit;
        }else{
            s->speed_limit=RETURN_APPROACH_RPM;
            rpm=(float)peak*0.55f;
            if(rpm>RETURN_APPROACH_RPM)rpm=RETURN_APPROACH_RPM;
            if(rpm<3.0f)rpm=3.0f;
        }
    }
    for(i=0;i<4;i++){
        float target_rpm;
        if(Abs(error[i])<=RETURN_TOLERANCE_TICKS)continue;
        target_rpm=rpm*(float)error[i]/(float)peak;
        /* Position error and target RPM use the encoder frame. Demand uses logical PWM. */
        s->demand[i]=(int16_t)(target_rpm*WHEEL_PWM_TO_RPM_SIGN*
                               CAR_MAX_PWM_PERMILLE/WHEEL_SPEED_MAX_RPM);
    }
}
