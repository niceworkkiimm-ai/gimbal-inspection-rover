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
        s->reason=RETURN_OK;s->pulse=0;Save(s,s->previous,now);return;
    }
    if(action!=RETURN_START||task!=s->task){s->result=RETURN_BAD_TASK;return;}
    if(s->state!=RETURN_RECORDING){s->result=RETURN_NO_PATH;return;}
    /* Include the exact parked endpoint, even below the sampling threshold. */
    if(memcmp(s->previous,s->point[s->count-1],sizeof(s->previous))&&
       !Save(s,s->previous,now)){s->result=RETURN_NO_PATH;return;}
    if(s->count<2){s->result=RETURN_NO_PATH;return;}
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
void ReturnPath_Step(ReturnPath *s,uint32_t now,const uint32_t counts[4],
                     uint8_t healthy,uint8_t centered_ready){
    uint32_t dt=now-s->last_ms;
    uint8_t i,moved=0,save=0,reversal=0,at_target=1;
    int32_t error[4];int64_t peak=0,d,limit;float rpm;
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
        if(Abs(d)>limit)ReturnPath_Abort(s,RETURN_COUNTS);
        if(s->state==RETURN_RECORDING&&d){
            int8_t sign=d>0?1:-1;
            if(s->travel_sign[i]&&sign!=s->travel_sign[i])reversal=1;
            s->travel_sign[i]=sign;
        }
    }
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
    for(i=0;i<4;i++){
        d=(int64_t)Delta(s->point[s->index][i],counts[i])*signs[i];
        if(Abs(d)>WHEEL_SPEED_CPR*2U){ReturnPath_Abort(s,RETURN_COUNTS);return;}
        error[i]=(int32_t)d;if(Abs(d)>peak)peak=Abs(d);
        if(Abs(d)>RETURN_TOLERANCE_TICKS)at_target=0;
    }
    if(at_target){
        if(s->index){s->index--;s->segment_ms=now;s->settled=0;}
        else {
            if(!s->settled){s->settled=1;s->settled_ms=now;}
            if((uint32_t)(now-s->settled_ms)>=400U){s->state=RETURN_DONE;s->reason=RETURN_OK;}
        }
        return;
    }
    s->settled=0;
    /* Common scaling makes the four wheels reach each checkpoint together.
       The inner WheelSpeed PI handles speed feedback, slew and DIR dwell. */
    rpm=(float)peak*0.55f;if(rpm>RETURN_MAX_RPM)rpm=RETURN_MAX_RPM;
    if(rpm<3.0f)rpm=3.0f;
    for(i=0;i<4;i++){
        float target_rpm;
        if(Abs(error[i])<=RETURN_TOLERANCE_TICKS)continue;
        target_rpm=rpm*(float)error[i]/(float)peak;
        /* Position error and target RPM use the encoder frame. Demand uses logical PWM. */
        s->demand[i]=(int16_t)(target_rpm*WHEEL_PWM_TO_RPM_SIGN*
                               CAR_MAX_PWM_PERMILLE/WHEEL_SPEED_MAX_RPM);
    }
}
