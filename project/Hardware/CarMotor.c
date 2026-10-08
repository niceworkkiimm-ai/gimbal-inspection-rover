/* V8.9 / 2026-09-15: manual vehicle control only; route replay removed.
 * Manual limits, neutral checks, link/deadline stop and RPM monitoring retained. */
#include "CarMotor.h"
#include "CarConfig.h"
#include "WheelSpeed.h"
#include "WheelEncoder.h"

static CarLogic car;
static WheelSpeed wheel_speed;
static volatile int16_t applied_pwm[4];
static const int8_t feedback_sign[4]=WHEEL_ENCODER_SIGNS;
static volatile uint32_t motor_ms, main_seen_ms, sample_cycles;
static volatile uint8_t initialized, active, sample_live, latched_fault, lease_lost;
static uint16_t pwm_period;
static uint8_t mode_epoch, have_epoch;
static const int8_t motor_sign[4]={CAR_M1_SIGN,CAR_M2_SIGN,CAR_M3_SIGN,CAR_M4_SIGN};
static void BrakeOutputs(void);

uint32_t CarMotor_GetMillis(void) { return motor_ms; }

static void BrakeOutputs(void)
{
    uint8_t i;
    TIM8->CCR1=0;TIM8->CCR2=0;TIM9->CCR1=0;TIM9->CCR2=0;
    TIM8->EGR=TIM_EGR_UG;TIM9->EGR=TIM_EGR_UG;
    for(i=0;i<4;i++)applied_pwm[i]=0;
}
void CarMotor_EmergencyStop(void)
{
    if(!initialized)return;
    latched_fault=1U;
    BrakeOutputs();
}
void TIM7_IRQHandler(void)
{
    if(TIM_GetITStatus(TIM7,TIM_IT_Update)==RESET)return;
    TIM_ClearITPendingBit(TIM7,TIM_IT_Update);
    motor_ms++;
    if(!active)return;
    /* V6.1: PD3 is not fitted. Keep the independent software deadline check. */
    if((uint32_t)(motor_ms-main_seen_ms)>CAR_MAIN_DEADLINE_MS)latched_fault=1U;
    if(!sample_live||(uint32_t)(DWT->CYCCNT-sample_cycles)>
       (SystemCoreClock/1000U)*CAR_LINK_TIMEOUT_MS)lease_lost=1U;
    if(latched_fault||lease_lost)BrakeOutputs();
}
static void SetOutputs(const int16_t logical_pwm[4])
{
    int16_t p[4];
    uint8_t i;
    for(i=0;i<4;i++){
        applied_pwm[i]=logical_pwm[i];
        p[i]=(int16_t)(logical_pwm[i]*motor_sign[i]);
    }
    /* Logic ramps through zero and waits before DIR reversal. */
    if(p[0])GPIO_WriteBit(GPIOC,GPIO_Pin_4,p[0]>0?Bit_SET:Bit_RESET);
    if(p[1])GPIO_WriteBit(GPIOC,GPIO_Pin_5,p[1]>0?Bit_SET:Bit_RESET);
    if(p[2])GPIO_WriteBit(GPIOB,GPIO_Pin_12,p[2]>0?Bit_SET:Bit_RESET);
    if(p[3])GPIO_WriteBit(GPIOB,GPIO_Pin_13,p[3]>0?Bit_SET:Bit_RESET);
    for(i=0;i<4;i++)if(p[i]<0)p[i]=(int16_t)-p[i];
    TIM8->CCR1=(uint32_t)p[0]*pwm_period/1000U;
    TIM8->CCR2=(uint32_t)p[1]*pwm_period/1000U;
    TIM9->CCR1=(uint32_t)p[2]*pwm_period/1000U;
    TIM9->CCR2=(uint32_t)p[3]*pwm_period/1000U;
}
void CarMotor_Init(void)
{
    GPIO_InitTypeDef gpio;
    TIM_TimeBaseInitTypeDef timer;
    TIM_OCInitTypeDef oc;
    NVIC_InitTypeDef nvic;
    RCC_ClocksTypeDef clocks;
    uint32_t apb1_timer,apb2_timer;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB|RCC_AHB1Periph_GPIOC|
                           RCC_AHB1Periph_GPIOE,ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8|RCC_APB2Periph_TIM9,ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7,ENABLE);
    GPIO_StructInit(&gpio);gpio.GPIO_Mode=GPIO_Mode_OUT;gpio.GPIO_OType=GPIO_OType_PP;
    gpio.GPIO_Speed=GPIO_Speed_50MHz;gpio.GPIO_PuPd=GPIO_PuPd_DOWN;
    GPIO_ResetBits(GPIOC,GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7);
    GPIO_ResetBits(GPIOB,GPIO_Pin_12|GPIO_Pin_13);
    GPIO_ResetBits(GPIOE,GPIO_Pin_5|GPIO_Pin_6);
    gpio.GPIO_Pin=GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;GPIO_Init(GPIOC,&gpio);
    gpio.GPIO_Pin=GPIO_Pin_12|GPIO_Pin_13;GPIO_Init(GPIOB,&gpio);
    gpio.GPIO_Pin=GPIO_Pin_5|GPIO_Pin_6;GPIO_Init(GPIOE,&gpio);
    RCC_GetClocksFreq(&clocks);
    apb1_timer=clocks.PCLK1_Frequency*((RCC->CFGR&RCC_CFGR_PPRE1)?2U:1U);
    apb2_timer=clocks.PCLK2_Frequency*((RCC->CFGR&RCC_CFGR_PPRE2)?2U:1U);
    pwm_period=(uint16_t)(apb2_timer/CAR_PWM_HZ);
    TIM_TimeBaseStructInit(&timer);timer.TIM_Prescaler=0;
    timer.TIM_Period=(uint16_t)(pwm_period-1U);
    TIM_TimeBaseInit(TIM8,&timer);TIM_TimeBaseInit(TIM9,&timer);
    TIM_OCStructInit(&oc);oc.TIM_OCMode=TIM_OCMode_PWM1;
    oc.TIM_OutputState=TIM_OutputState_Enable;oc.TIM_Pulse=0;
    oc.TIM_OCPolarity=TIM_OCPolarity_High;oc.TIM_OCIdleState=TIM_OCIdleState_Reset;
    TIM_OC1Init(TIM8,&oc);TIM_OC2Init(TIM8,&oc);TIM_OC1Init(TIM9,&oc);TIM_OC2Init(TIM9,&oc);
    TIM_OC1PreloadConfig(TIM8,TIM_OCPreload_Enable);TIM_OC2PreloadConfig(TIM8,TIM_OCPreload_Enable);
    TIM_OC1PreloadConfig(TIM9,TIM_OCPreload_Enable);TIM_OC2PreloadConfig(TIM9,TIM_OCPreload_Enable);
    BrakeOutputs();
    GPIO_PinAFConfig(GPIOC,GPIO_PinSource6,GPIO_AF_TIM8);
    GPIO_PinAFConfig(GPIOC,GPIO_PinSource7,GPIO_AF_TIM8);
    GPIO_PinAFConfig(GPIOE,GPIO_PinSource5,GPIO_AF_TIM9);
    GPIO_PinAFConfig(GPIOE,GPIO_PinSource6,GPIO_AF_TIM9);
    gpio.GPIO_Mode=GPIO_Mode_AF;gpio.GPIO_PuPd=GPIO_PuPd_DOWN;
    gpio.GPIO_Pin=GPIO_Pin_6|GPIO_Pin_7;GPIO_Init(GPIOC,&gpio);
    gpio.GPIO_Pin=GPIO_Pin_5|GPIO_Pin_6;GPIO_Init(GPIOE,&gpio);
    TIM_CtrlPWMOutputs(TIM8,ENABLE);TIM_Cmd(TIM8,ENABLE);TIM_Cmd(TIM9,ENABLE);
    CarLogic_Init(&car,0);WheelSpeed_Init(&wheel_speed);initialized=1U;
    /* Independent 1ms software safety tick; no SysTick/Delay ownership conflict. */
    TIM_TimeBaseStructInit(&timer);
    timer.TIM_Prescaler=(uint16_t)(apb1_timer/1000000U-1U);timer.TIM_Period=999U;
    TIM_TimeBaseInit(TIM7,&timer);TIM_ClearITPendingBit(TIM7,TIM_IT_Update);
    nvic.NVIC_IRQChannel=TIM7_IRQn;nvic.NVIC_IRQChannelPreemptionPriority=0;
    nvic.NVIC_IRQChannelSubPriority=0;nvic.NVIC_IRQChannelCmd=ENABLE;NVIC_Init(&nvic);
    TIM_ITConfig(TIM7,TIM_IT_Update,ENABLE);TIM_Cmd(TIM7,ENABLE);
}
void CarMotor_Service(WirelessControlMode_t mode,const WirelessControlData_t *data,uint8_t rx_fault)
{
    uint32_t primask=__get_PRIMASK(),now;
    uint8_t fresh=0U,ready=0U,allowed,i;
    uint32_t packet=0U;
    int16_t roll=0,pitch=0;
    int16_t demand[4],output[4];
    EncoderFrame encoder;
    if(!initialized)return;
    __disable_irq();
    now=motor_ms;main_seen_ms=now;
    active=(uint8_t)(mode==WIRELESS_MODE_CAR && CAR_RUNTIME_ENABLE);
    if(mode!=WIRELESS_MODE_CAR){latched_fault=0U;lease_lost=0U;}
    if(active&&rx_fault)latched_fault=1U;
    if(data&&data->protocol==1U&&data->requested_mode==1U&&
       (uint32_t)(DWT->CYCCNT-data->arrival_cycles)<=
       (SystemCoreClock/1000U)*CAR_LINK_TIMEOUT_MS)
    {
        fresh=1U;ready=data->ready;packet=data->packet_count;
        roll=(int16_t)(data->roll*10.0f);pitch=(int16_t)(data->pitch*10.0f);
        sample_cycles=data->arrival_cycles;
    }
    sample_live=fresh;
    if(fresh&&(!have_epoch||mode_epoch!=data->mode_epoch)) {
        mode_epoch=data->mode_epoch;have_epoch=1U;
        CarLogic_Stop(&car,now,CAR_NEUTRAL);
    }
    if(lease_lost){CarLogic_Stop(&car,now,CAR_LINK_LOST);lease_lost=0U;}
    CarLogic_Step(&car,now,(uint8_t)(mode==WIRELESS_MODE_CAR),
                   (uint8_t)CAR_RUNTIME_ENABLE,(uint8_t)!latched_fault,
                   fresh,ready,packet,roll,pitch);
    allowed=(uint8_t)(active&&!latched_fault&&!lease_lost&&fresh&&ready&&
                      car.state!=CAR_FAULT&&car.armed&&!car.paused);
    for(i=0;i<4;i++)demand[i]=car.pwm[i];
    if(!allowed)BrakeOutputs();
    if(!primask)__enable_irq();

    /* Encoder sampling uses the existing atomic snapshot. All RPM/PI arithmetic
     * runs outside the interrupt mask; EXTI counting and TIM7 remain serviceable.
     */
    WheelEncoder_Snapshot(&encoder,0);
    if(mode!=WIRELESS_MODE_CAR)WheelSpeed_ClearFault(&wheel_speed);
    WheelSpeed_Step(&wheel_speed,encoder.milliseconds,encoder.count,demand,
                    WHEEL_SPEED_ENABLE,allowed);
    for(i=0;i<4;i++)output[i]=WHEEL_SPEED_ENABLE?wheel_speed.output[i]:demand[i];

    __disable_irq();
    /* TIM7 may have stopped outputs during the calculation. Never re-enable
     * them from an older foreground decision: recheck latched state and age.
     */
    if(WHEEL_SPEED_ENABLE&&wheel_speed.fault){
        latched_fault=1U;CarLogic_Stop(&car,motor_ms,CAR_FAULT);
    }
    if(!active||!allowed||latched_fault||lease_lost||!sample_live||
       (uint32_t)(DWT->CYCCNT-sample_cycles)>(SystemCoreClock/1000U)*CAR_LINK_TIMEOUT_MS)
        BrakeOutputs();
    else SetOutputs(output);
    if(!primask)__enable_irq();
}
void CarMotor_GetStatus(uint8_t *state,int16_t pwm[4])
{
    uint32_t primask=__get_PRIMASK();
    uint8_t i;
    __disable_irq();
    *state=latched_fault?CAR_FAULT:lease_lost?CAR_LINK_LOST:car.state;
    for(i=0;i<4;i++)pwm[i]=(latched_fault||lease_lost)?0:applied_pwm[i];
    if(!primask)__enable_irq();
}
uint8_t CarMotor_GetHoldReason(void)
{
    return car.hold_reason;
}

void CarMotor_GetWheelSpeed(WheelSpeedFrame *frame,uint32_t sequence)
{
    uint8_t i,fault=wheel_speed.fault,fault_wheels=wheel_speed.fault_wheels;
    uint32_t primask;
    float value;
    frame->sequence=sequence;frame->milliseconds=wheel_speed.last_ms;
    frame->enabled=(uint8_t)WHEEL_SPEED_ENABLE;frame->valid=wheel_speed.valid;
    frame->fault=fault;frame->fault_wheels=fault_wheels;
    frame->limited=wheel_speed.limited;frame->cpr=WHEEL_SPEED_CPR;frame->sign_mask=0;
    frame->dt_ms=(uint16_t)(wheel_speed.sample_dt>65535U?65535U:wheel_speed.sample_dt);
    for(i=0;i<4;i++){
        if(feedback_sign[i]<0)frame->sign_mask|=(uint8_t)(1U<<i);
        frame->target[i]=(int16_t)(wheel_speed.target[i]*10.0f);
        value=wheel_speed.rpm[i]*10.0f;
        if(value>20000.0f)value=20000.0f;
        if(value < -20000.0f)value=-20000.0f;
        frame->measured[i]=(int16_t)value;
    }
    primask=__get_PRIMASK();__disable_irq();
    for(i=0;i<4;i++)frame->pwm[i]=(latched_fault||lease_lost)?0:applied_pwm[i];
    if(!primask)__enable_irq();
}
