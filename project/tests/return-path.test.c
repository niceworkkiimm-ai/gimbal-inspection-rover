#include "ReturnPath.h"
#include "ReturnProtocol.h"
#include "WheelSpeed.h"
#include "WheelPulse.h"
#include "CarLogic.h"
#include "CarConfig.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
static ReturnPath s;
static uint32_t now,counts[4];
static const int signs[4]=WHEEL_ENCODER_SIGNS;
static void tick(uint32_t dt,int a,int b,int c,int d,int healthy,int ready){
    int v[4]={a,b,c,d};unsigned i;now+=dt;
    for(i=0;i<4;i++)counts[i]+=(uint32_t)(v[i]*signs[i]);
    ReturnPath_Step(&s,now,counts,(uint8_t)healthy,(uint8_t)ready);
}
static void init(void){
    unsigned i;ReturnPath_Init(&s);s.boot=913;now=0;
    for(i=0;i<4;i++)counts[i]=0;
    tick(0,0,0,0,0,1,1);
    for(i=0;i<13;i++)tick(40,0,0,0,0,1,1);
    assert(s.ready);ReturnPath_Command(&s,913,1,RETURN_SET,0,now);
    assert(s.state==RETURN_RECORDING&&s.count==1);
}
static void park(void){unsigned i;for(i=0;i<13;i++)tick(40,0,0,0,0,1,1);assert(s.ready);}
static void start(void){park();ReturnPath_Command(&s,913,2,RETURN_START,1,now);assert(s.state==RETURN_RUNNING);}
static void zero(void){unsigned i;for(i=0;i<4;i++)assert(s.demand[i]==0);}
static void checks_logged_drive_polarity(void)
{
    WheelSpeed wheel;
    unsigned i;
    init();
    /* The field log shows all four signed RPMs positive under -298 PWM.
       These are the approximate signed ticks in one 40 ms sample. */
    tick(40,27,26,26,26,1,0);
    start();
    tick(40,0,0,0,0,1,1);
    WheelSpeed_Init(&wheel);
    WheelSpeed_Step(&wheel,now,counts,s.demand,1,1);
    WheelSpeed_Step(&wheel,now+40,counts,s.demand,1,1);
    for(i=0;i<4;i++){
        assert(s.demand[i]>0);      /* Undo the negative manual PWM. */
        assert(wheel.target[i]<0); /* Reduce the recorded positive wheel displacement. */
        assert(wheel.output[i]>0);
    }
    puts("PASS measured chassis polarity: positive outbound RPM unwinds with positive logical PWM");
}
static void ideal_return(void){
    unsigned step,i;uint32_t pulse=0;double pos[4],fraction[4]={0};
    for(i=0;i<4;i++)pos[i]=counts[i];
    (void)pos;
    for(step=0;step<10000&&s.state==RETURN_RUNNING;step++){
        uint16_t target=s.index;
        now+=40;ReturnPath_Heartbeat(&s,s.boot,s.task,++pulse,now);
        for(i=0;i<4;i++){
            int delta;fraction[i]-=s.demand[i]/10.0*WHEEL_SPEED_CPR*40.0/60000.0;
            delta=(int)fraction[i];fraction[i]-=delta;counts[i]+=(uint32_t)(delta*signs[i]);
        }
        ReturnPath_Step(&s,now,counts,1,1);
        if(s.index!=target||s.state==RETURN_DONE)
            for(i=0;i<4;i++)assert(abs((int32_t)(counts[i]-s.point[target][i]))<=RETURN_TOLERANCE_TICKS);
    }
    assert(s.state==RETURN_DONE);zero();
    for(i=0;i<4;i++)assert(abs((int32_t)(counts[i]-s.point[0][i]))<=RETURN_TOLERANCE_TICKS);
}
static void checks_protocol(void){
    char line[160];uint32_t v[4];int n;
    n=ReturnFinish(line,sizeof(line),snprintf(line,sizeof(line),"Q1,4294967295,12,2,1"));
    assert(ReturnFields(line,n-1,'1',v,4)&&v[0]==UINT32_MAX&&v[2]==2);
    line[5]^=1;assert(!ReturnFields(line,n-1,'1',v,4));
    n=ReturnFinish(line,sizeof(line),snprintf(line,sizeof(line),"Q1,4294967296,12,2,1"));
    assert(!ReturnFields(line,n-1,'1',v,4));
    n=ReturnFinish(line,sizeof(line),snprintf(line,sizeof(line),"Q1,1,12,2,1,7"));
    assert(!ReturnFields(line,n-1,'1',v,4));
    assert(!ReturnFields("Q1,",3,'1',v,4));
    puts("PASS CRC, integer overflow, trailing fields and truncated frames");
}
static void checks_commands(void){
    uint16_t n;init();
    tick(40,16,16,16,16,1,0);n=s.count;
    ReturnPath_Command(&s,913,1,RETURN_SET,0,now);assert(s.count==n);
    ReturnPath_Command(&s,914,2,RETURN_START,1,now);assert(s.ack==1&&s.state==RETURN_RECORDING);
    ReturnPath_Command(&s,913,2,RETURN_START,1,now);assert(s.result==RETURN_NOT_READY);
    park();ReturnPath_Command(&s,913,3,RETURN_START,7,now);assert(s.result==RETURN_BAD_TASK);
    ReturnPath_Command(&s,913,4,RETURN_START,1,now);assert(s.state==RETURN_RUNNING);
    ReturnPath_Command(&s,913,4,RETURN_SET,1,now);assert(s.state==RETURN_RUNNING);
    ReturnPath_Command(&s,913,5,RETURN_CANCEL,1,now);assert(s.state==RETURN_ABORTED&&s.stop_requested);
    ReturnPath_Command(&s,913,4,RETURN_START,1,now);assert(s.state==RETURN_ABORTED);
    ReturnPath_Command(&s,913,6,RETURN_START,1,now);assert(s.state==RETURN_ABORTED);
    init();ReturnPath_Command(&s,913,2,RETURN_START,1,now);assert(s.result==RETURN_NO_PATH);
    init();tick(40,16,16,16,16,1,0);start();
    ReturnPath_Command(&s,913,1,RETURN_CANCEL,0,now);
    assert(s.state==RETURN_ABORTED&&s.stop_requested&&s.ack==2);
    puts("PASS stationary start, task/boot match, duplicate/old commands, stop and no resume");
}
static void checks_paths(void){
    unsigned i;init();for(i=0;i<20;i++)tick(40,10,10,10,10,1,0);
    start();ideal_return();
    init();for(i=0;i<20;i++)tick(40,10,10,10,10,1,0);
    for(i=0;i<12;i++)tick(40,-10,10,-10,10,1,0);
    for(i=0;i<20;i++)tick(40,10,10,10,10,1,0);
    start();ideal_return();
    init();for(i=0;i<20;i++)tick(40,6,12,6,12,1,0);
    for(i=0;i<20;i++)tick(40,12,6,12,6,1,0);
    for(i=0;i<15;i++)tick(40,-8,-8,-8,-8,1,0);
    start();ideal_return();
    init();tick(40,7,7,7,7,1,0);tick(40,-3,-3,-3,-3,1,0);
    assert(s.count==2&&s.point[1][0]==7);start();ideal_return();
    init();for(i=0;i<4;i++)counts[i]=UINT32_MAX-20;
    s.state=RETURN_IDLE;tick(40,0,0,0,0,1,1);park();
    ReturnPath_Command(&s,s.boot,2,RETURN_SET,1,now);
    for(i=0;i<20;i++)tick(40,10,10,10,10,1,0);
    park();ReturnPath_Command(&s,s.boot,3,RETURN_START,2,now);ideal_return();
    puts("PASS straight, L turn, curves, reverse segments, small reversals and count wrap");
}
static void checks_faults(void){
    unsigned i;init();tick(40,16,16,16,16,1,0);start();tick(40,0,0,0,0,1,0);
    assert(s.state==RETURN_ABORTED&&s.reason==RETURN_MANUAL);zero();
    init();tick(40,16,16,16,16,1,0);start();
    for(i=0;i<40;i++){ReturnPath_Heartbeat(&s,s.boot,s.task,0,now);tick(40,0,0,0,0,1,1);}
    assert(s.reason==RETURN_BROWSER);zero();
    ReturnPath_Heartbeat(&s,s.boot,s.task,1,now);assert(s.state==RETURN_ABORTED);
    init();tick(121,1,1,1,1,1,0);assert(s.reason==RETURN_GAP);zero();
    init();tick(40,1000,0,0,0,1,0);assert(s.reason==RETURN_COUNTS);
    init();tick(40,0,0,0,0,0,1);assert(s.reason==RETURN_FEEDBACK);
    init();for(i=0;i<RETURN_CAPACITY;i++)tick(40,16,16,16,16,1,0);
    assert(s.state==RETURN_ABORTED&&s.reason==RETURN_FULL&&s.count==RETURN_CAPACITY&&s.point[0][0]==0);
    init();tick(40,16,16,16,16,1,0);start();
    for(i=1;i<205;i++){now+=40;ReturnPath_Heartbeat(&s,s.boot,s.task,i,now);ReturnPath_Step(&s,now,counts,1,1);}
    assert(s.reason==RETURN_TIMEOUT);zero();
    puts("PASS takeover, lost/replayed heartbeat, gap, bad encoder, full buffer and no-progress timeout");
}
static void checks_pi(void){
    WheelSpeed w;int16_t demand[4]={120,120,120,120};uint32_t c[4]={0};unsigned i;uint32_t t;
    WheelSpeed_Init(&w);
    for(t=0;t<2000;t+=40)WheelSpeed_Step(&w,t,c,demand,1,1);
    assert(w.fault&WHEEL_FAULT_NO_FEEDBACK);
    for(i=0;i<4;i++)assert(w.output[i]==0);
    WheelSpeed_Init(&w);
    for(t=0;t<1600;t+=40){
        /* Positive PWM should give negative signed RPM. Reverse only the front left. */
        for(i=0;i<4;i++)c[i]+=(uint32_t)((i==0?6:-6)*signs[i]);
        WheelSpeed_Step(&w,t,c,demand,1,1);
    }
    assert(w.fault==WHEEL_FAULT_DIRECTION&&w.fault_wheels==1);
    for(i=0;i<4;i++)assert(w.output[i]==0);
    puts("PASS inner PI stops all wheels on missing feedback or a reversed front-left encoder");
}
static void checks_plant(void){
    WheelSpeed w;unsigned i,j;double speed[4]={0},fraction[4]={0};
    /* Mean signed RPMs at -298 logical PWM in the 24 driving samples, 2026-11-18.
       This plant fixture is independent of the controller's sign configuration. */
    const double rpm_at_minus_298[4]={108.23,104.52,104.40,103.59};
    init();for(i=0;i<30;i++)tick(40,8,12,8,12,1,0);
    for(i=0;i<20;i++)tick(40,-8,8,-8,8,1,0);
    for(i=0;i<30;i++)tick(40,12,8,12,8,1,0);
    start();WheelSpeed_Init(&w);
    for(j=1;j<20000&&s.state==RETURN_RUNNING;j++){
        now+=40;ReturnPath_Heartbeat(&s,s.boot,s.task,j,now);
        for(i=0;i<4;i++){
            int d;speed[i]+=(w.output[i]*rpm_at_minus_298[i]/-298.0-speed[i])*0.5;
            fraction[i]+=speed[i]*WHEEL_SPEED_CPR*40/60000.0;
            d=(int)fraction[i];fraction[i]-=d;counts[i]+=(uint32_t)(d*signs[i]);
        }
        ReturnPath_Step(&s,now,counts,(uint8_t)!w.fault,1);
        WheelSpeed_Step(&w,now,counts,s.demand,WHEEL_PI_RETURN,(uint8_t)(s.state==RETURN_RUNNING));
        for(i=0;i<4;i++)assert(fabs(w.target[i])<=RETURN_MAX_RPM+0.001);
    }
    assert(s.state==RETURN_DONE&&!w.fault);
    for(i=0;i<4;i++)assert(abs((int32_t)(counts[i]-s.point[0][i]))<=RETURN_TOLERANCE_TICKS);
    puts("PASS end-to-end position plus speed PI with unequal first-order wheel models");
}
/* Parked endpoint has just six ticks to unwind. A frictionless model misses
 * this case: the old controller could sit at 2-3 percent PWM for seconds. */
static void checks_short_parked_segment(void)
{
    WheelSpeed w;
    uint32_t pulse=0;
    unsigned i,j;
    int moved=0;
    init();tick(40,6,6,6,6,1,0);start();WheelSpeed_Init(&w);
    for(j=0;j<200 && s.state==RETURN_RUNNING;j++){
        now+=40;
        ReturnPath_Heartbeat(&s,s.boot,s.task,++pulse,now);
        for(i=0;i<4;i++)if(w.output[i]>=52){counts[i]-=(uint32_t)signs[i];moved=1;}
        ReturnPath_Step(&s,now,counts,!w.fault,1);
        WheelSpeed_Step(&w,now,counts,s.demand,WHEEL_PI_RETURN,s.state==RETURN_RUNNING);
    }
    assert(moved && s.state==RETURN_DONE && !w.fault);
    for(i=0;i<4;i++)assert(!w.output[i] && abs((int32_t)counts[i])<=RETURN_TOLERANCE_TICKS);
    puts("PASS short parked endpoint returns with a static-friction threshold");
}

static void checks_encoder_signal(void)
{
    WheelPulse p={0};
    unsigned i;
    int direction, step, total=0;
    uint32_t us=UINT32_MAX-20U;
    assert(WheelPulse_Edge(&p,0,1,us,10,80)==0);
    for(direction=-1;direction<=1;direction+=2){
        for(i=0;i<WHEEL_SPEED_CPR;i++){
            unsigned b=direction>0;
            assert(WheelPulse_Edge(&p,1,b,us,10+i,80)==0);
            step=WheelPulse_Edge(&p,0,!b,us+400,10+i,80);
            assert(step==direction);
            total+=step;
            us+=900;
        }
        assert(total==(direction<0?-(int)WHEEL_SPEED_CPR:0));
    }
    /* Alternate fixed-B chatter and narrow high pulses with changing B. */
    for(i=0;i<4;i++){
        unsigned b=i&1U;
        uint32_t width=b?20:400;
        assert(WheelPulse_Edge(&p,1,b,us,1000+i,80)==0);
        assert(WheelPulse_Edge(&p,0,0,us+width,1000+i,80)==0);
        assert(WheelPulse_Noisy(&p,1000+i)==(i==3));
        us+=900;
    }
    assert(WheelPulse_Noisy(&p,1123) && !WheelPulse_Noisy(&p,1124));
    /* Parked with A high for more than one DWT revolution. */
    assert(WheelPulse_Edge(&p,1,1,100,2000,80)==0);
    assert(WheelPulse_Edge(&p,0,0,110,30000,80)==1);
    init();tick(40,16,16,16,16,1,0);start();
    tick(1,0,0,30,0,1,1);
    assert(s.state==RETURN_ABORTED && s.reason==RETURN_COUNTS && s.count_fault_wheels==4);
    zero();park();ReturnPath_Command(&s,s.boot,3,RETURN_SET,0,now);
    assert(s.count_fault_wheels==0);
    ReturnPath_CountFault(&s,5);
    assert(s.state==RETURN_ABORTED && s.reason==RETURN_COUNTS && s.count_fault_wheels==5);
    init();tick(40,16,16,16,16,1,0);start();
    s.point[s.index][3]-=WHEEL_SPEED_CPR*3U;
    tick(40,0,0,0,0,1,1);
    assert(s.reason==RETURN_COUNTS && s.count_fault_wheels==8);zero();
    puts("PASS pulse polarity, full revolutions, width, chatter, wrap and retained wheel masks");
}

static void checks_return_breakaway(void)
{
    WheelSpeed w;
    uint32_t t,c[4]={0};
    int16_t cmd[4]={0,0,33,0}; /* 3.3 RPM, as at checkpoint 83 in task 5. */
    int max_effort=0;
    WheelSpeed_Init(&w);
    for(t=0;t<=1200;t+=40){
        WheelSpeed_Step(&w,t,c,cmd,WHEEL_PI_RETURN,1);
        if(w.output[2]>max_effort)max_effort=w.output[2];
        if(w.fault)break;
    }
    assert(t<=880 && w.fault==WHEEL_FAULT_NO_FEEDBACK && w.fault_wheels==4);
    assert(max_effort>=52 && max_effort<=(int)RETURN_BREAKAWAY_MAX);
    assert(!w.output[0] && !w.output[1] && !w.output[2] && !w.output[3]);
    WheelSpeed_Init(&w);
    for(t=0;t<=2000;t+=40){
        WheelSpeed_Step(&w,t,c,cmd,WHEEL_PI_RETURN,1);
        assert(!w.fault);
        if(w.output[2]>=52)break; /* Synthetic breakaway threshold. */
    }
    assert(t<800);
    c[2]--;WheelSpeed_Step(&w,t+40,c,cmd,WHEEL_PI_RETURN,1);
    assert(w.output[2]<52 && !w.fault);
    cmd[2]=0;WheelSpeed_Step(&w,t+41,c,cmd,WHEEL_PI_RETURN,1);
    assert(!w.output[2]);
    WheelSpeed_Init(&w);cmd[2]=33;
    for(t=0;t<=920;t+=40){
        WheelSpeed_Step(&w,t,c,cmd,WHEEL_PI_MANUAL,1);
        assert(w.output[2]<52 && !w.fault);
    }
    puts("PASS bounded start assist, release on motion, feedback timeout and manual PI isolation");
}

static void checks_dense_straight(void)
{
    WheelSpeed w;
    double speed[4]={0}, fraction[4]={0};
    unsigned i,j,pass_stops=0,passes=0;
    uint32_t started;
    init();
    for(i=0;i<110;i++)tick(40,16,15,16,15,1,0);
    assert(s.count==111);
    start();started=now;WheelSpeed_Init(&w);
    for(j=0;j<14000 && s.state==RETURN_RUNNING;j++){
        uint16_t before=s.index;
        int any=0;
        now+=40;
        ReturnPath_Heartbeat(&s,s.boot,s.task,j+1,now);
        for(i=0;i<4;i++){
            int pwm=w.output[i], ticks;
            /* Same synthetic load for both versions; breakaway and rolling
             * friction differ and the wheels have slightly different gain. */
            double duty=fabs((double)pwm);
            double threshold=fabs(speed[i])<0.45?50.0:31.0;
            double goal=duty<threshold?0.0:(pwm>0?-1:1)*(duty-30.0)*(0.38+0.01*i);
            speed[i]+=0.5*(goal-speed[i]);
            fraction[i]+=speed[i]*WHEEL_SPEED_CPR*40/60000.0;
            ticks=(int)fraction[i];fraction[i]-=ticks;
            counts[i]+=(uint32_t)(ticks*signs[i]);
        }
        ReturnPath_Step(&s,now,counts,!w.fault,1);
        WheelSpeed_Step(&w,now,counts,s.demand,WHEEL_PI_RETURN,s.state==RETURN_RUNNING);
        for(i=0;i<4;i++){
            assert(fabs(w.target[i])<=RETURN_MAX_RPM+0.001);
            if(s.demand[i])any=1;
        }
        if(s.index!=before){assert(s.index+1==before);passes++;if(!any)pass_stops++;}
    }
    assert(s.state==RETURN_DONE && !w.fault && passes==109);
    for(i=0;i<4;i++)assert(abs((int32_t)counts[i])<=RETURN_TOLERANCE_TICKS);
    printf("Dense straight fixture: %lu ms, %u/%u all-wheel zero commands at pass points\n",
           (unsigned long)(now-started),pass_stops,passes);
    assert(pass_stops==0);
}

static void checks_preview_limits(void)
{
    unsigned i;
    init();tick(40,16,16,16,16,1,0);tick(40,16,16,16,16,1,0);start();
    /* One wheel can reach the point first; the index waits for the laggards. */
    tick(40,-20,-8,-8,-8,1,1);assert(s.index==1);
    tick(40,0,-8,-8,-8,1,1);assert(s.index==0);
    for(i=0;i<4;i++)assert(s.demand[i]>0);
    /* Similar neighboring directions must not skip a wheel that is far ahead. */
    init();tick(80,70,70,70,70,1,0);tick(40,16,16,16,16,1,0);start();
    tick(80,-50,-16,-16,-16,1,1);assert(s.index==1 && s.demand[0]<0);
    /* Even with all wheel signs unchanged, a sharp change of ratios is a corner. */
    init();tick(40,16,4,16,4,1,0);tick(40,16,16,16,16,1,0);start();
    tick(40,-16,-16,-16,-16,1,1);assert(s.index==0);zero();
    /* An actual sign reversal on one pair also requires the exact corner. */
    init();tick(40,16,-16,16,-16,1,0);tick(40,16,16,16,16,1,0);start();
    tick(40,-16,-16,-16,-16,1,1);assert(s.index==0);zero();
    tick(40,0,0,0,0,1,1);assert(s.demand[0]>0 && s.demand[1]<0);
    puts("PASS wheel lag, bounded overshoot, ratio corners and reversals");
}

static float peak_target(void)
{
    float peak=0.0f;
    unsigned i;
    for(i=0;i<4;i++){
        float rpm=(float)abs(s.demand[i])*WHEEL_SPEED_MAX_RPM/CAR_MAX_PWM_PERMILLE;
        if(rpm>peak)peak=rpm;
    }
    return peak;
}

static void check_cruise(int a,int b,int c,int d,int fast)
{
    unsigned i;
    float last=RETURN_APPROACH_RPM;
    init();
    for(i=0;i<8;i++)tick(40,a,b,c,d,1,0);
    start();
    for(i=0;i<16;i++){
        float rpm;
        tick(40,0,0,0,0,1,1);
        rpm=peak_target();
        assert(rpm<=last+0.04f*RETURN_CRUISE_RAMP_RPM_S+0.11f);
        assert(rpm<=RETURN_MAX_RPM && (fast || rpm<=RETURN_APPROACH_RPM));
        last=rpm;
    }
    if(fast)assert(last>RETURN_MAX_RPM-0.11f);
}

static void checks_cruise_and_stop(void)
{
    unsigned i;
    check_cruise(16,15,16,15,1);
    check_cruise(-16,-15,-16,-15,1);
    check_cruise(16,-16,16,-16,0);     /* Spin. */
    check_cruise(-16,16,16,-16,0);     /* Strafe. */
    check_cruise(16,6,16,6,0);         /* Tight constant-radius curve. */
    init();for(i=0;i<40;i++)tick(40,1,1,1,1,1,0);
    start();
    for(i=0;i<16;i++){tick(40,0,0,0,0,1,1);assert(peak_target()<=RETURN_APPROACH_RPM);}
    assert(!s.fast_straight);
    init();tick(40,-16,16,-16,16,1,0);
    for(i=0;i<4;i++)tick(40,16,16,16,16,1,0);
    start();
    for(i=0;i<16;i++)tick(40,0,0,0,0,1,1);
    assert(peak_target()>RETURN_APPROACH_RPM);
    while(s.index>2)tick(40,-16,-16,-16,-16,1,1);
    tick(40,0,0,0,0,1,1);
    assert(peak_target()<=RETURN_APPROACH_RPM && !s.fast_straight);
    tick(40,0,0,0,0,1,0);
    assert(s.state==RETURN_ABORTED && s.reason==RETURN_MANUAL);zero();

    init();tick(40,16,16,16,16,1,0);start();
    tick(40,-16,-16,-16,-16,1,1);
    for(i=0;i<16;i++){
        tick(40,i%2?1:-1,0,0,0,1,1);
        assert(s.state==RETURN_RUNNING);zero();
    }
    for(i=0;i<9;i++)tick(40,0,0,0,0,1,1);
    assert(s.state==RETURN_RUNNING);
    tick(40,0,0,0,0,1,1);assert(s.state==RETURN_DONE);zero();
    puts("PASS cruise ramp, bidirectional straight, slow spin/strafe/curve, corner approach, takeover and stopped arrival");
}

int main(void){checks_cruise_and_stop();checks_dense_straight();checks_preview_limits();checks_short_parked_segment();checks_encoder_signal();checks_return_breakaway();checks_protocol();checks_commands();checks_logged_drive_polarity();checks_paths();checks_faults();checks_pi();checks_plant();puts("All return controller checks passed.");return 0;}
