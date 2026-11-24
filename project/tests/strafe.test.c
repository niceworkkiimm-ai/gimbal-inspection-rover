#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "CarLogic.h"
#include "CarConfig.h"
#include "WheelSpeedConfig.h"
#include "HandheldProtocol.h"

static unsigned now;
static void drive(CarLogic *s, int frames, int mask, int pitch, int fresh)
{
    int i;
    for(i=0;i<frames;i++){
        now+=20;
        CarLogic_StepWithButtons(s,now,1,1,1,0,(uint8_t)fresh,1,now,0,(int16_t)pitch,(uint8_t)mask);
    }
}
static void stopped(const CarLogic *s)
{
    int i;for(i=0;i<4;i++)assert(s->pwm[i]==0);
}
int main(void)
{
    HandheldFrame frame={0},parsed={0};
    char buf[64];int n,mask;
    CarLogic s;
    frame.sequence=9;frame.mode=1;frame.epoch=7;frame.ready=1;frame.pitch=-20;
    for(mask=0;mask<=3;mask++){
        frame.buttons=(uint8_t)mask;n=HH_Format(buf,sizeof(buf),&frame);
        assert(n>0&&buf[1]==(mask?'2':'1'));
        assert(HH_Parse(buf,(size_t)n-1,&parsed));
        assert(parsed.buttons==mask&&parsed.sequence==9&&parsed.pitch==-20);
        buf[n-2]^=1;assert(!HH_Parse(buf,(size_t)n-1,&parsed));
    }
    frame.mode=0;assert(!HH_Format(buf,sizeof(buf),&frame));
    CarLogic_Init(&s,0);
    drive(&s,30,1,-20,1);stopped(&s);assert(!s.armed);
    drive(&s,30,0,-20,1);assert(s.armed);
    drive(&s,25,1,140,1);
    assert(s.pwm[0]*WHEEL_PWM_TO_RPM_SIGN==-CAR_STRAFE_PWM_PERMILLE&&s.pwm[1]==-s.pwm[0]&&s.pwm[2]==s.pwm[1]&&s.pwm[3]==s.pwm[0]);
    drive(&s,1,0,-20,1);stopped(&s);
    drive(&s,25,2,-20,1);
    assert(s.pwm[0]*WHEEL_PWM_TO_RPM_SIGN==CAR_STRAFE_PWM_PERMILLE&&s.pwm[1]==-s.pwm[0]&&s.pwm[2]==s.pwm[1]&&s.pwm[3]==s.pwm[0]);
    drive(&s,1,3,-20,1);stopped(&s);
    drive(&s,30,1,-20,1);stopped(&s);
    drive(&s,30,0,-20,1);drive(&s,25,2,-20,1);
    drive(&s,1,2,-20,0);stopped(&s);
    drive(&s,30,2,-20,1);stopped(&s);assert(!s.armed);
    drive(&s,30,0,-20,1);drive(&s,25,1,-20,1);
    drive(&s,1,0,190,1);stopped(&s);assert(!s.armed);
    puts("PASS H1/H2, left/right wheel signs, hold-release, chord, rearm and stale input");
    return 0;
}

