/* V8.17 / 2026-11-17: locally recorded reverse return uses the wheel PI.
 * Manual limits, neutral checks and independent link/deadline stop retained. */
#include "CarMotor.h"
#include "CarConfig.h"
#include "WheelSpeed.h"
#include "WheelEncoder.h"
#include "ReturnPath.h"
#include "ReturnProtocol.h"

static ReturnPath return_path;
/* ISR reads only these aligned volatile lease values. */
static volatile uint8_t return_active;
static volatile uint32_t return_lease_ms;

static CarLogic car;
static WheelSpeed wheel_speed;
static volatile int16_t applied_pwm[4];
static const int8_t feedback_sign[4]=WHEEL_ENCODER_SIGNS;
static volatile uint32_t motor_ms, main_seen_ms, sample_cycles;
static uint32_t uart_rx_faults, max_service_gap_ms;
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
    if(return_active&&(uint32_t)(motor_ms-return_lease_ms)>RETURN_BROWSER_TIMEOUT_MS)lease_lost=1U;
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
    CarLogic_Init(&car,0);WheelSpeed_Init(&wheel_speed);ReturnPath_Init(&return_path);
    RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_RNG,ENABLE);
    RNG->CR |= RNG_CR_RNGEN; /* Nonblocking boot nonce; unavailable RNG disables return. */
    initialized=1U;
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
    uint8_t fresh=0U,ready=0U,buttons=0U,allowed,i,centered,was_return,closed_loop,suspect_wheels;
    uint32_t packet=0U;
    int16_t roll=0,pitch=0;
    int16_t demand[4],output[4];
    EncoderFrame encoder;
    if(!initialized)return;
    if(!return_path.boot&&(RNG->SR & RNG_SR_DRDY)&&
       !(RNG->SR & (RNG_SR_CECS|RNG_SR_SECS)))return_path.boot=RNG->DR;
    was_return=(uint8_t)(return_path.state==RETURN_RUNNING);
    __disable_irq();
    now=motor_ms;
    if(active&&mode==WIRELESS_MODE_CAR) {
        uint32_t gap=now-main_seen_ms;
        if(gap>max_service_gap_ms)max_service_gap_ms=gap;
    }
    main_seen_ms=now;
    active=(uint8_t)(mode==WIRELESS_MODE_CAR && CAR_RUNTIME_ENABLE);
    if(mode!=WIRELESS_MODE_CAR){latched_fault=0U;lease_lost=0U;}
    if(active&&rx_fault)uart_rx_faults++;
    if(data&&data->protocol==1U&&data->requested_mode==1U&&
       (uint32_t)(DWT->CYCCNT-data->arrival_cycles)<=
       (SystemCoreClock/1000U)*CAR_LINK_TIMEOUT_MS)
    {
        fresh=1U;ready=data->ready;buttons=data->buttons;packet=data->packet_count;
        roll=(int16_t)(data->roll*10.0f);pitch=(int16_t)(data->pitch*10.0f);
        sample_cycles=data->arrival_cycles;
    }
    sample_live=fresh;
    if(fresh&&(!have_epoch||mode_epoch!=data->mode_epoch)) {
        if(was_return)ReturnPath_Abort(&return_path,RETURN_MANUAL);
        mode_epoch=data->mode_epoch;have_epoch=1U;
        CarLogic_Stop(&car,now,CAR_NEUTRAL);
    }
    if(lease_lost){
        ReturnPath_Abort(&return_path,return_active&&
          (uint32_t)(now-return_lease_ms)>RETURN_BROWSER_TIMEOUT_MS?RETURN_BROWSER:RETURN_LINK);
        CarLogic_Stop(&car,now,CAR_LINK_LOST);lease_lost=0U;
    }
    centered=(uint8_t)(!buttons&&roll-CAR_ROLL_ZERO_TENTHS<=CAR_NEUTRAL_TENTHS&&
        roll-CAR_ROLL_ZERO_TENTHS>=-CAR_NEUTRAL_TENTHS&&
        pitch-CAR_PITCH_ZERO_TENTHS<=CAR_NEUTRAL_TENTHS&&
        pitch-CAR_PITCH_ZERO_TENTHS>=-CAR_NEUTRAL_TENTHS);
    if(was_return&&(!active||!fresh||!ready||!centered||rx_fault||latched_fault)){
        ReturnPath_Abort(&return_path,!fresh?RETURN_LINK:rx_fault||latched_fault?RETURN_FEEDBACK:RETURN_MANUAL);
        CarLogic_Stop(&car,now,CAR_NEUTRAL);
    }
    if(return_path.stop_requested){
        return_path.stop_requested=0;CarLogic_Stop(&car,now,CAR_NEUTRAL);
    }
    CarLogic_StepWithButtons(&car,now,(uint8_t)(mode==WIRELESS_MODE_CAR),
                   (uint8_t)CAR_RUNTIME_ENABLE,(uint8_t)!latched_fault,
                   rx_fault,fresh,ready,packet,roll,pitch,buttons);
    allowed=(uint8_t)(active&&!latched_fault&&!lease_lost&&fresh&&ready&&
                      car.state!=CAR_FAULT&&car.armed&&!car.paused);
    for(i=0;i<4;i++)demand[i]=car.pwm[i];
    if(!allowed)BrakeOutputs();
    if(!primask)__enable_irq();

    /* Encoder sampling uses the existing atomic snapshot. All RPM/PI arithmetic
     * runs outside the interrupt mask; EXTI counting and TIM7 remain serviceable.
     */
    suspect_wheels=WheelEncoder_Snapshot(&encoder,0);
    ReturnPath_CountFault(&return_path,suspect_wheels);
    if(mode!=WIRELESS_MODE_CAR)WheelSpeed_ClearFault(&wheel_speed);
    ReturnPath_Step(&return_path,encoder.milliseconds,encoder.count,
        (uint8_t)(!rx_fault&&!latched_fault&&!wheel_speed.fault&&!suspect_wheels),
        (uint8_t)(allowed&&centered&&wheel_speed.valid));
    return_lease_ms=return_path.lease_ms;
    return_active=(uint8_t)(return_path.state==RETURN_RUNNING);
    closed_loop=(uint8_t)(WHEEL_SPEED_ENABLE||return_active||was_return);
    if(return_active)for(i=0;i<4;i++)demand[i]=return_path.demand[i];
    if(was_return&&!return_active){
        allowed=0;CarLogic_Stop(&car,now,CAR_NEUTRAL);
        for(i=0;i<4;i++)demand[i]=0;
    }
    WheelSpeed_Step(&wheel_speed,encoder.milliseconds,encoder.count,demand,
                    return_active?WHEEL_PI_RETURN:closed_loop,allowed);
    for(i=0;i<4;i++)output[i]=closed_loop?wheel_speed.output[i]:demand[i];

    __disable_irq();
    /* TIM7 may have stopped outputs during the calculation. Never re-enable
     * them from an older foreground decision: recheck latched state and age.
     */
    if(closed_loop&&wheel_speed.fault){
        ReturnPath_Abort(&return_path,RETURN_FEEDBACK);return_active=0;
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
    if(return_active&&!latched_fault&&!lease_lost&&
       (pwm[0]||pwm[1]||pwm[2]||pwm[3]))*state=CAR_MOVING;
    if(!primask)__enable_irq();
}
uint8_t CarMotor_GetHoldReason(void)
{
    return car.hold_reason;
}

void CarMotor_GetDiagnostics(uint32_t *uart_faults,uint32_t *service_gap_ms)
{
    *uart_faults=uart_rx_faults;
    *service_gap_ms=max_service_gap_ms;
}

void CarMotor_GetWheelSpeed(WheelSpeedFrame *frame,uint32_t sequence)
{
    uint8_t i,fault=wheel_speed.fault,fault_wheels=wheel_speed.fault_wheels;
    uint32_t primask;
    float value;
    if(return_path.state==RETURN_ABORTED && return_path.count_fault_wheels){
        fault|=WHEEL_FAULT_COUNTS;
        fault_wheels|=return_path.count_fault_wheels;
    }
    frame->sequence=sequence;frame->milliseconds=wheel_speed.last_ms;
    frame->enabled=wheel_speed.enabled;frame->valid=wheel_speed.valid;
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

void CarMotor_CancelReturn(uint8_t reason)
{
    uint8_t running=(uint8_t)(return_path.state==RETURN_RUNNING);
    if(running){ReturnPath_Abort(&return_path,reason);return_path.stop_requested=1;}
}
void CarMotor_ReturnReceive(const char *line,uint16_t length)
{
    uint32_t v[4];
    if(ReturnFields(line,length,'1',v,4)&&v[2]>=RETURN_SET&&v[2]<=RETURN_CANCEL)
        ReturnPath_Command(&return_path,v[0],v[1],(uint8_t)v[2],v[3],motor_ms);
    else if(ReturnFields(line,length,'3',v,3)){
        ReturnPath_Heartbeat(&return_path,v[0],v[1],v[2],motor_ms);
        return_lease_ms=return_path.lease_ms;
    }
}
int CarMotor_ReturnStatus(char *line,uint16_t capacity)
{
    return ReturnFinish(line,capacity,snprintf(line,capacity,
        "Q2,%lu,%lu,%u,%lu,%u,%u,%u,%u,%u,%u,%lu",
        (unsigned long)return_path.boot,(unsigned long)return_path.ack,
        return_path.result,(unsigned long)return_path.task,return_path.state,
        return_path.reason,return_path.count,return_path.index,return_path.ready,
        return_path.command_action,(unsigned long)return_path.command_task));
}
