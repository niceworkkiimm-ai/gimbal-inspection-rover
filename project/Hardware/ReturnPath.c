#include "ReturnPath.h"
#include <string.h>
#include <math.h>

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
static uint8_t PlanSegment(const ReturnPath *s,uint16_t index,int32_t advance[4]);
static uint8_t Save(ReturnPath *s,const uint32_t counts[4],uint32_t now){
    uint32_t elapsed=now-s->saved_ms;
    if(s->count>=RETURN_CAPACITY){ReturnPath_Abort(s,RETURN_FULL);return 0;}
    s->leg_ms[s->count]=!s->count?0:(uint16_t)(elapsed>65535U?65535U:elapsed);
    memcpy(s->point[s->count],counts,sizeof(s->previous));
    s->entry_rpm[s->count++]=0;
    if(s->count>=3){
        uint16_t joint=(uint16_t)(s->count-2);
        s->entry_rpm[joint]=PlanSegment(s,joint,NULL);
    }
    s->saved_ms=now;
    return 1;
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
        s->last_motion_ms=now;s->count_fault_wheels=0;s->reason=RETURN_OK;s->pulse=0;Save(s,s->previous,now);return;
    }
    if(action!=RETURN_START||task!=s->task){s->result=RETURN_BAD_TASK;return;}
    if(s->state!=RETURN_RECORDING){s->result=RETURN_NO_PATH;return;}
    /* Include the exact parked endpoint, even below the sampling threshold. */
    if(memcmp(s->previous,s->point[s->count-1],sizeof(s->previous))&&
       !Save(s,s->previous,s->last_motion_ms)){s->result=RETURN_NO_PATH;return;}
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

static uint8_t RecordedPace(int32_t peak,uint16_t elapsed)
{
    uint32_t divisor,rpm;
    if(!elapsed)return 0;
    divisor=(uint32_t)elapsed*WHEEL_SPEED_CPR;
    rpm=((uint32_t)peak*60000U+divisor/2U)/divisor;
    return (uint8_t)(rpm>RETURN_MAX_RPM?RETURN_MAX_RPM:rpm);
}

/* Compare both adjacent segments in wheel-count space. Normalizing each by
 * its largest wheel displacement makes the check independent of point spacing.
 * A significant wheel start/stop, reversal or ratio change > 1/4 defines
 * a hard corner. Ambiguous motion within tick tolerance has no preview. */
static uint8_t PlanSegment(const ReturnPath *s, uint16_t index, int32_t advance[4])
{
    int32_t incoming[4], outgoing[4], in_peak=0, out_peak=0, horizon;
    unsigned i;
    uint8_t cap,limit;
    uint8_t exit_rpm;
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
        return 0; /* preview only; the pace was cached when the point was saved */
    }
    if(in_peak<RETURN_CRUISE_MIN_TICKS || out_peak<RETURN_CRUISE_MIN_TICKS)return 0;
    /* SAVE processes joints in outbound order, so the next reverse joint
     * already has a braking envelope. No full-path pass at start or in an ISR. */
    cap=TravelCap(incoming,in_peak);
    limit=TravelCap(outgoing,out_peak);
    if(limit<cap)cap=limit;
    limit=RecordedPace(in_peak,s->leg_ms[index+1]);
    if(limit<cap)cap=limit;
    limit=RecordedPace(out_peak,s->leg_ms[index]);
    if(limit<cap)cap=limit;
    exit_rpm=s->entry_rpm[index-1];
    if(!exit_rpm)return cap>RETURN_APPROACH_RPM?(uint8_t)RETURN_APPROACH_RPM:cap;
    /* v^2 = v_exit^2 + 2*a*distance in wheel ticks and RPM. */
    {
        float braking_squared=(float)exit_rpm*exit_rpm+
                              (120.0f*RETURN_BRAKE_RPM_S/WHEEL_SPEED_CPR)*(float)out_peak;
        return (float)cap*cap<=braking_squared?cap:(uint8_t)sqrtf(braking_squared);
    }
}

/* Compute the current wheel errors again immediately after crossing a smooth
 * checkpoint. The speed loop must not brake towards the previous preview. */
static uint8_t MeasureTarget(ReturnPath *s,const uint32_t counts[4],int32_t error[4],
                             int64_t *peak,int32_t *preview_peak)
{
    uint8_t i,at_target=1,bad_wheels=0;
    int64_t d;
    *peak=0;*preview_peak=0;
    /* Points are immutable during return. Plan once per index, and require
     * a compatible joint ahead to leave a low-speed segment before a hard corner. */
    if(s->planned_index!=s->index){
        int32_t leg_peak=0;
        for(i=0;i<4;i++){
            int64_t travel=Abs(Delta(s->point[s->index+1][i],s->point[s->index][i]));
            if(travel>2U*WHEEL_SPEED_CPR)bad_wheels|=(uint8_t)(1U<<i);
            else if(travel>leg_peak)leg_peak=(int32_t)travel;
        }
        if(bad_wheels){ReturnPath_CountFault(s,bad_wheels);return 0;}
        s->approach_rpm=RecordedPace(leg_peak,s->leg_ms[s->index+1]);
        if(s->approach_rpm>RETURN_APPROACH_RPM)s->approach_rpm=(uint8_t)RETURN_APPROACH_RPM;
        if(s->approach_rpm<3)s->approach_rpm=3;
        memset(s->advance,0,sizeof(s->advance));
        PlanSegment(s,s->index,s->advance);
        s->cruise_rpm=s->entry_rpm[s->index];
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
        if(Abs(s->advance[i])>*preview_peak)*preview_peak=(int32_t)Abs(s->advance[i]);
        error[i]=(int32_t)d+s->advance[i];
        if(Abs(error[i])>*peak)*peak=Abs(error[i]);
    }
    if(bad_wheels)ReturnPath_CountFault(s,bad_wheels);
    return at_target;
}

void ReturnPath_Step(ReturnPath *s,uint32_t now,const uint32_t counts[4],
                     uint8_t healthy,uint8_t centered_ready){
    uint32_t dt=now-s->last_ms;
    uint8_t i,moved=0,save=0,reversal=0,at_target,bad_wheels=0,passed=0;
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
    if(s->state==RETURN_RECORDING && moved){
        uint8_t resumed=(uint8_t)((uint32_t)(now-s->last_motion_ms)>RETURN_PARK_GAP_MS);
        if((reversal||resumed) && memcmp(s->previous,s->point[s->count-1],sizeof(s->previous)))
            Save(s,s->previous,s->last_motion_ms);
        if(resumed)s->saved_ms=s->last_ms;
        s->last_motion_ms=now;
    }
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
        if(save)Save(s,counts,s->last_motion_ms);
        return;
    }
    if(s->state!=RETURN_RUNNING)return;
    if((uint32_t)(now-s->segment_ms)>8000U||(uint32_t)(now-s->run_ms)>600000U){
        ReturnPath_Abort(s,RETURN_TIMEOUT);return;
    }
    at_target=MeasureTarget(s,counts,error,&peak,&preview_peak);
    if(s->state!=RETURN_RUNNING)return;
    while(at_target && s->index){
        /* At speed a sample can cross more than one dense joint. Validate
         * each one in sequence, never drive through an unverified turn. */
        if(passed++>=4)return;
        s->index--;s->segment_ms=now;s->settled=0;
        if(!preview_peak){s->speed_limit=RETURN_APPROACH_RPM;return;}
        at_target=MeasureTarget(s,counts,error,&peak,&preview_peak);
        if(s->state!=RETURN_RUNNING)return;
    }
    if(at_target){
        if(!s->settled || moved){s->settled=1;s->settled_ms=now;}
        if((uint32_t)(now-s->settled_ms)>=400U){s->state=RETURN_DONE;s->reason=RETURN_OK;}
        s->speed_limit=RETURN_APPROACH_RPM;
        return;
    }
    s->settled=0;
    /* Common scaling preserves the four-wheel proportions through a pass point.
       The inner WheelSpeed PI handles speed feedback, slew and DIR dwell. */
    {
        uint8_t cruise=peak>=RETURN_CRUISE_MIN_TICKS?s->cruise_rpm:0;
        /* Compare each wheel against its expected share of the segment.
         * An inner wheel at one third of outer speed is normal on an arc. */
        for(i=0;i<4 && cruise;i++){
            int64_t expected=Abs(s->advance[i]);
            if(Abs(error[i])<=RETURN_TOLERANCE_TICKS){
                /* This wheel can wait at its preview while the others catch up. */
                if(expected>RETURN_TOLERANCE_TICKS && cruise>RETURN_SYNC_RPM)cruise=RETURN_SYNC_RPM;
            }else if(!expected || (int64_t)error[i]*s->advance[i]<=0){
                cruise=0; /* parked-wheel drift or an actual direction correction */
            }else if(Abs(error[i])*2*preview_peak<peak*expected ||
                     Abs(error[i])*preview_peak>2*peak*expected){
                if(cruise>RETURN_SYNC_RPM)cruise=RETURN_SYNC_RPM;
            }
        }
        if(cruise){
            s->speed_limit+=RETURN_CRUISE_RAMP_RPM_S*(float)dt/1000.0f;
            if(s->speed_limit>cruise)s->speed_limit=(float)cruise;
            rpm=s->speed_limit;
        }else{
            s->speed_limit=RETURN_APPROACH_RPM;
            rpm=(float)peak*0.55f;
            if(rpm>s->approach_rpm)rpm=(float)s->approach_rpm;
            if(rpm<3.0f)rpm=3.0f;
        }
    }
    for(i=0;i<4;i++){
        float target_rpm;
        if(Abs(error[i])<=RETURN_TOLERANCE_TICKS)continue;
        target_rpm=rpm*(float)error[i]/(float)peak;
        s->demand[i]=(int16_t)(target_rpm*WHEEL_PWM_TO_RPM_SIGN*10.0f);
    }
}
