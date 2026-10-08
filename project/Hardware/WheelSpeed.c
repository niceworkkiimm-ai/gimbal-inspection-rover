/* V8.9 / 2026-09-15: route-only sign correction/startup assist removed.
 * Manual monitor/optional PI behavior and fault checks are unchanged. */
#include "WheelSpeed.h"
#include "CarConfig.h"
#include <string.h>

static const int8_t encoder_sign[4]=WHEEL_ENCODER_SIGNS;
static float Abs(float v){return v<0?-v:v;}
static float Clamp(float v,float low,float high){return v<low?low:v>high?high:v;}
static int8_t Sign(int32_t v){return v>0?1:v<0?-1:0;}
static int32_t CountDelta(uint32_t a,uint32_t b)
{
    uint32_t d=a-b;
    return d<=0x7fffffffU?(int32_t)d:(int32_t)(-((int64_t)(~d)+1));
}
void WheelSpeed_Init(WheelSpeed *s){memset(s,0,sizeof(*s));}
static void Zero(WheelSpeed *s,uint32_t now)
{
    uint8_t i;
    for(i=0;i<4;i++){
        if(s->output[i])s->zero_ms[i]=now;
        s->output[i]=0;s->target[i]=s->integral[i]=s->effort[i]=0;
        s->started[i]=s->wrong[i]=0;
    }
    s->limited=0;
}
void WheelSpeed_ClearFault(WheelSpeed *s){s->fault=0;s->fault_wheels=0;}
void WheelSpeed_Step(WheelSpeed *s,uint32_t now,const uint32_t counts[4],
                     const int16_t demand[4],uint8_t enabled,uint8_t allowed)
{
    uint32_t dt,total_ms=0;
    int32_t ticks[4],target_sign;
    int64_t magnitude,limit,total;
    uint8_t i,j,new_sample=0;
    float seconds,desired,measured,error,next_i,unsaturated,goal,step;
    s->enabled=enabled?1:0;
    if(!s->initialized){
        s->initialized=1;s->last_ms=now;
        for(i=0;i<4;i++){s->count[i]=counts[i];s->moved_ms[i]=now;}
        Zero(s,now);return;
    }
    dt=now-s->last_ms;
    if(dt>=WHEEL_SPEED_PERIOD_MS){
        new_sample=1;s->sample_dt=dt;s->last_ms=now;
        if(dt>WHEEL_SPEED_MAX_GAP_MS){
            s->valid=0;s->size=s->head=0;
            if(enabled&&allowed){s->fault|=WHEEL_FAULT_TIMING;s->fault_wheels=15;}
            for(i=0;i<4;i++)s->count[i]=counts[i];
        }else{
            s->valid=1;
            limit=(int64_t)WHEEL_SPEED_CPR*WHEEL_SPEED_SANITY_RPM*dt/60000+4;
            for(i=0;i<4;i++){
                ticks[i]=CountDelta(counts[i],s->count[i]);s->count[i]=counts[i];
                magnitude=ticks[i]<0?-(int64_t)ticks[i]:ticks[i];
                if(magnitude>limit){
                    s->valid=0;
                    if(enabled&&allowed){s->fault|=WHEEL_FAULT_COUNTS;s->fault_wheels|=(uint8_t)(1U<<i);}
                }
                if(ticks[i])s->moved_ms[i]=now;
            }
            if(!s->valid){s->size=s->head=0;}
            else{
                s->window_ms[s->head]=dt;
                for(i=0;i<4;i++)s->window_ticks[s->head][i]=ticks[i]*encoder_sign[i];
                s->head=(uint8_t)((s->head+1)%WHEEL_SPEED_WINDOW);
                if(s->size<WHEEL_SPEED_WINDOW)s->size++;
                for(j=0;j<s->size;j++)total_ms+=s->window_ms[j];
                for(i=0;i<4;i++){
                    total=0;for(j=0;j<s->size;j++)total+=s->window_ticks[j][i];
                    s->rpm[i]=(float)total*60000.0f/((float)WHEEL_SPEED_CPR*(float)total_ms);
                }
            }
        }
    }
    if(!enabled){Zero(s,now);return;} /* Monitor never changes existing manual PWM. */
    if(!allowed||!s->valid||s->fault){Zero(s,now);return;}
    /* Handle neutral/reversal immediately, even between feedback samples. */
    for(i=0;i<4;i++){
        target_sign=Sign(demand[i]);
        if(!target_sign||(s->previous_sign[i]&&target_sign!=s->previous_sign[i])){
            if(s->output[i])s->zero_ms[i]=now;
            s->output[i]=0;s->effort[i]=s->integral[i]=s->target[i]=0;s->started[i]=s->wrong[i]=0;
        }
    }
    if(!new_sample)return;
    seconds=(float)dt/1000.0f;s->limited=0;
    for(i=0;i<4;i++){
        target_sign=Sign(demand[i]);
        if(!target_sign)continue;
        if(s->previous_sign[i]&&target_sign!=s->previous_sign[i]&&
           (uint32_t)(now-s->zero_ms[i])<CAR_REVERSE_PAUSE_MS)continue;
        if(!s->started[i]){s->started[i]=1;s->started_ms[i]=s->moved_ms[i]=now;s->wrong[i]=0;}
        s->previous_sign[i]=(int8_t)target_sign;
        desired=Clamp(Abs((float)demand[i])*WHEEL_SPEED_MAX_RPM/CAR_MAX_PWM_PERMILLE,0,WHEEL_SPEED_MAX_RPM);
        s->target[i]=desired*(float)target_sign;
        measured=s->rpm[i]*(float)target_sign;
        if((uint32_t)(now-s->started_ms[i])>=WHEEL_SPEED_START_GRACE_MS){
            if(s->output[i]&&Abs((float)s->output[i])>=80.0f&&
               (uint32_t)(now-s->moved_ms[i])>=WHEEL_SPEED_STALL_MS){
                s->fault|=WHEEL_FAULT_NO_FEEDBACK;s->fault_wheels|=(uint8_t)(1U<<i);
            }
            if(measured < -6.0f&&(uint32_t)(now-s->started_ms[i])>=WHEEL_SPEED_REVERSE_GRACE_MS){
                if(!s->wrong[i]){s->wrong[i]=1;s->wrong_ms[i]=now;}
                else if((uint32_t)(now-s->wrong_ms[i])>=WHEEL_SPEED_WRONG_DIRECTION_MS){
                    s->fault|=WHEEL_FAULT_DIRECTION;s->fault_wheels|=(uint8_t)(1U<<i);
                }
            }else s->wrong[i]=0;
        }
        error=desired-measured;
        next_i=Clamp(s->integral[i]+WHEEL_SPEED_KI*error*seconds,
                     -WHEEL_SPEED_INTEGRAL_LIMIT,WHEEL_SPEED_INTEGRAL_LIMIT);
        unsaturated=WHEEL_SPEED_FF*desired+WHEEL_SPEED_KP*error+next_i;
        if(!((unsaturated>CAR_MAX_PWM_PERMILLE&&error>0)||(unsaturated<0&&error<0)))s->integral[i]=next_i;
        goal=Clamp(WHEEL_SPEED_FF*desired+WHEEL_SPEED_KP*error+s->integral[i],0,CAR_MAX_PWM_PERMILLE);
        if(unsaturated>=CAR_MAX_PWM_PERMILLE||goal>=CAR_MAX_PWM_PERMILLE)
            s->limited|=(uint8_t)(1U<<i);
        step=WHEEL_SPEED_OUTPUT_SLEW*seconds;
        s->effort[i]+=Clamp(goal-s->effort[i],-step,step);
        s->output[i]=(int16_t)((int32_t)(s->effort[i]+0.5f)*target_sign);
    }
    if(s->fault)Zero(s,now); /* Any feedback fault stops all four wheels; no open-loop fallback. */
}
