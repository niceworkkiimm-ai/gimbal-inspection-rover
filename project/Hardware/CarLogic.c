#include "CarLogic.h"
#include "CarConfig.h"
#include <string.h>

static int32_t Magnitude(int32_t x) { return x < 0 ? -x : x; }
static int8_t Sign(int32_t x) { return x < 0 ? -1 : x > 0 ? 1 : 0; }
void CarLogic_Init(CarLogic *s, uint32_t now)
{
    memset(s,0,sizeof(*s));s->last_step=now;s->state=CAR_DISABLED;
}
static void ZeroPwm(CarLogic *s, uint32_t now)
{
    uint8_t i;
    for(i=0;i<4;i++) {
        if(s->pwm[i])s->zero_since[i]=now;
        s->pwm[i]=0;
    }
}
void CarLogic_Stop(CarLogic *s, uint32_t now, uint8_t state)
{
    ZeroPwm(s,now);
    s->armed=0;s->neutral_count=0;s->state=state;s->last_step=now;
    s->paused=0;s->recovery_count=0;
    s->hold_reason=state==CAR_NEUTRAL?CAR_HOLD_CENTER:CAR_HOLD_NONE;
}
static void PauseInput(CarLogic *s, uint32_t now, uint8_t reason)
{
    if(s->armed) {
        if(!s->paused){s->paused=1;s->pause_since=now;}
        ZeroPwm(s,now);s->neutral_count=0;s->recovery_count=0;
        s->state=CAR_NEUTRAL;s->last_step=now;
        /* Repeated glitches cannot extend the first pause indefinitely. */
        if((uint32_t)(now-s->pause_since)>=CAR_TRANSIENT_WINDOW_MS)
            CarLogic_Stop(s,now,CAR_NEUTRAL);
    }
    else CarLogic_Stop(s,now,CAR_NEUTRAL);
    s->hold_reason=reason;
}
void CarLogic_Mix(int16_t forward,int16_t left,int16_t out[4])
{
    int32_t l=(int32_t)forward-left,r=(int32_t)forward+left;
    int32_t peak=Magnitude(l)>Magnitude(r)?Magnitude(l):Magnitude(r);
    if(peak>CAR_MAX_PWM_PERMILLE) {
        l=l*CAR_MAX_PWM_PERMILLE/peak;r=r*CAR_MAX_PWM_PERMILLE/peak;
    }
    out[0]=out[2]=(int16_t)l;out[1]=out[3]=(int16_t)r;
}
static int16_t Tilt(int32_t angle)
{
    int32_t n=Magnitude(angle);
    if(n<=CAR_DEADZONE_TENTHS)return 0;
    if(n>CAR_FULL_TILT_TENTHS)n=CAR_FULL_TILT_TENTHS;
    return (int16_t)(Sign(angle)*(n-CAR_DEADZONE_TENTHS)*CAR_MAX_PWM_PERMILLE/
                    (CAR_FULL_TILT_TENTHS-CAR_DEADZONE_TENTHS));
}
void CarLogic_Step(CarLogic *s,uint32_t now,uint8_t active,uint8_t enabled,
                   uint8_t safe,uint8_t fresh,uint8_t ready,uint32_t packet,
                   int16_t roll,int16_t pitch)
{
    int32_t r=(int32_t)roll-CAR_ROLL_ZERO_TENTHS;
    int32_t p=(int32_t)pitch-CAR_PITCH_ZERO_TENTHS;
    uint32_t dt=now-s->last_step, step;
    uint8_t i,new_packet=(uint8_t)(!s->have_packet||packet!=s->last_packet);
    int16_t target[4],goal;
    int32_t delta;
    if(!enabled){CarLogic_Stop(s,now,CAR_DISABLED);return;}
    if(!active){CarLogic_Stop(s,now,CAR_INACTIVE);s->have_packet=0;return;}
    if(!safe){CarLogic_Stop(s,now,CAR_FAULT);return;}
    if(!fresh){CarLogic_Stop(s,now,CAR_LINK_LOST);return;}
    if(new_packet){s->have_packet=1;s->last_packet=packet;}
    if(!ready){
        PauseInput(s,now,CAR_HOLD_INPUT);return;
    }
    if(Magnitude(r)>CAR_UNSAFE_TILT_TENTHS||Magnitude(p)>CAR_UNSAFE_TILT_TENTHS){
        PauseInput(s,now,CAR_HOLD_TILT);return;
    }
    if(s->paused) {
        ZeroPwm(s,now);
        s->state=CAR_NEUTRAL;s->hold_reason=CAR_HOLD_RECOVER;s->last_step=now;
        if((uint32_t)(now-s->pause_since)>=CAR_TRANSIENT_WINDOW_MS) {
            CarLogic_Stop(s,now,CAR_NEUTRAL);
        }
        else {
            if(Magnitude(r)>CAR_RECOVERY_TILT_TENTHS||
               Magnitude(p)>CAR_RECOVERY_TILT_TENTHS) {
                s->recovery_count=0;s->hold_reason=CAR_HOLD_TILT;return;
            }
            if(new_packet) {
                if(!s->recovery_count)s->recovery_since=now;
                if(s->recovery_count<255)s->recovery_count++;
                if(s->recovery_count>=CAR_RECOVERY_PACKETS&&
                   (uint32_t)(now-s->recovery_since)>=CAR_RECOVERY_HOLD_MS) {
                    s->paused=0;s->recovery_count=0;
                    s->state=CAR_READY;
                }
            }
            if(s->paused)return;
            /* Restart the normal slew at zero, not at the pre-pause duty. */
            dt=0;
        }
    }
    if(!s->armed) {
        s->state=CAR_NEUTRAL;s->hold_reason=CAR_HOLD_CENTER;s->last_step=now;
        if(Magnitude(r)>CAR_NEUTRAL_TENTHS||Magnitude(p)>CAR_NEUTRAL_TENTHS) {
            s->neutral_count=0;return;
        }
        if(new_packet) {
            if(!s->neutral_count)s->neutral_since=now;
            if(s->neutral_count<255)s->neutral_count++;
            if(s->neutral_count>=CAR_NEUTRAL_PACKETS&&
               (uint32_t)(now-s->neutral_since)>=CAR_NEUTRAL_HOLD_MS)
                s->armed=1;
        }
        if(!s->armed)return;
    }
    s->hold_reason=CAR_HOLD_NONE;
    /* Same axes as the gimbal: pitch fore/aft, roll left/right. */
    CarLogic_Mix(Tilt(p*CAR_FORWARD_SIGN),Tilt(r*CAR_TURN_LEFT_SIGN),target);
    /* Neutral means stop now; no slow drift while the wrist is centered. */
    if(!target[0]&&!target[1]) {
        for(i=0;i<4;i++){if(s->pwm[i])s->zero_since[i]=now;s->pwm[i]=0;}
        s->state=CAR_READY;s->last_step=now;return;
    }
    if(dt<CAR_CONTROL_PERIOD_MS)return;
    if(dt>CAR_MAIN_DEADLINE_MS){CarLogic_Stop(s,now,CAR_FAULT);return;}
    s->last_step=now;step=CAR_SLEW_PER_SECOND*dt/1000U;if(!step)step=1;
    s->state=CAR_READY;
    for(i=0;i<4;i++) {
        goal=target[i];
        if(s->pwm[i]&&Sign(s->pwm[i])!=Sign(goal))goal=0;
        if(!s->pwm[i]&&goal&&s->previous_sign[i]&&
           Sign(goal)!=s->previous_sign[i]&&
           (uint32_t)(now-s->zero_since[i])<CAR_REVERSE_PAUSE_MS)goal=0;
        delta=(int32_t)goal-s->pwm[i];
        if(delta>(int32_t)step)delta=(int32_t)step;
        if(delta<-(int32_t)step)delta=-(int32_t)step;
        if(s->pwm[i]&&s->pwm[i]+delta==0)s->zero_since[i]=now;
        s->pwm[i]=(int16_t)(s->pwm[i]+delta);
        if(s->pwm[i]){s->previous_sign[i]=Sign(s->pwm[i]);s->state=CAR_MOVING;}
    }
}
