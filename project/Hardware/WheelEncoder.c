/* V8.19: validate paired A edges against B on the existing four EXTI lines.
 * M1 A=PE2/B=PF8, M2 A=PE3/B=PF11, M3 A=PF6/B=PD6, M4 A=PF7/B=PD7.
 * 3.3V logic. Driver-fed encoder power must not be paralleled with another rail.
 * Raw sign: B high at A rising => +1; each wheel sign is calibrated in the web UI.
 * ISR only counts. No delay, floating point, printf, or motor control here.
 * Missing/unwired encoders cannot be inferred from a zero count; UI says so.
 */
#include "WheelEncoder.h"
#include "CarMotor.h"
#include "WheelPulse.h"

static volatile uint32_t wheel_count[4];
static volatile WheelPulse pulse[4];
static uint32_t min_high_cycles;

static void CountEdge(uint8_t wheel, uint8_t a, uint8_t b)
{
    int8_t step = WheelPulse_Edge(&pulse[wheel], a, b, DWT->CYCCNT,
                                  CarMotor_GetMillis(), min_high_cycles);
    wheel_count[wheel] += (uint32_t)step;
}

void WheelEncoder_Init(void)
{
    GPIO_InitTypeDef gpio;
    EXTI_InitTypeDef exti;
    NVIC_InitTypeDef nvic;
    min_high_cycles = (SystemCoreClock / 1000000U) * ENCODER_MIN_HIGH_US;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD|RCC_AHB1Periph_GPIOE|
                           RCC_AHB1Periph_GPIOF,ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG,ENABLE);
    GPIO_StructInit(&gpio);
    gpio.GPIO_Mode=GPIO_Mode_IN;gpio.GPIO_PuPd=GPIO_PuPd_DOWN;
    gpio.GPIO_Pin=GPIO_Pin_2|GPIO_Pin_3;GPIO_Init(GPIOE,&gpio);
    gpio.GPIO_Pin=GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_11;GPIO_Init(GPIOF,&gpio);
    gpio.GPIO_Pin=GPIO_Pin_6|GPIO_Pin_7;GPIO_Init(GPIOD,&gpio);
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE,EXTI_PinSource2);
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE,EXTI_PinSource3);
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOF,EXTI_PinSource6);
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOF,EXTI_PinSource7);
    EXTI_ClearITPendingBit(EXTI_Line2|EXTI_Line3|EXTI_Line6|EXTI_Line7);
    EXTI_StructInit(&exti);
    exti.EXTI_Line=EXTI_Line2|EXTI_Line3|EXTI_Line6|EXTI_Line7;
    exti.EXTI_Mode=EXTI_Mode_Interrupt;exti.EXTI_Trigger=EXTI_Trigger_Rising_Falling;
    exti.EXTI_LineCmd=ENABLE;EXTI_Init(&exti);
    /* TIM7 motor deadline and USART retain higher preemption priority. */
    nvic.NVIC_IRQChannelPreemptionPriority=2;
    nvic.NVIC_IRQChannelSubPriority=0;nvic.NVIC_IRQChannelCmd=ENABLE;
    nvic.NVIC_IRQChannel=EXTI2_IRQn;NVIC_Init(&nvic);
    nvic.NVIC_IRQChannel=EXTI3_IRQn;NVIC_Init(&nvic);
    nvic.NVIC_IRQChannel=EXTI9_5_IRQn;NVIC_Init(&nvic);
}

uint8_t WheelEncoder_Snapshot(EncoderFrame *frame,uint32_t sequence)
{
    uint32_t primask=__get_PRIMASK();
    uint8_t i, suspect = 0;
    __disable_irq();
    frame->sequence=sequence;
    frame->milliseconds=CarMotor_GetMillis();
    for(i=0;i<4;i++){
        frame->count[i]=wheel_count[i];
        if(WheelPulse_Noisy(&pulse[i],frame->milliseconds)) suspect |= (uint8_t)(1U<<i);
    }
    if(!primask)__enable_irq();
    return suspect;
}

void EXTI2_IRQHandler(void)
{
    if(EXTI->PR&EXTI_Line2){
        EXTI_ClearITPendingBit(EXTI_Line2);
        CountEdge(0, (GPIOE->IDR&GPIO_Pin_2) != 0, (GPIOF->IDR&GPIO_Pin_8) != 0);
    }
}
void EXTI3_IRQHandler(void)
{
    if(EXTI->PR&EXTI_Line3){
        EXTI_ClearITPendingBit(EXTI_Line3);
        CountEdge(1, (GPIOE->IDR&GPIO_Pin_3) != 0, (GPIOF->IDR&GPIO_Pin_11) != 0);
    }
}
void EXTI9_5_IRQHandler(void)
{
    if(EXTI->PR&EXTI_Line6){
        EXTI_ClearITPendingBit(EXTI_Line6);
        CountEdge(2, (GPIOF->IDR&GPIO_Pin_6) != 0, (GPIOD->IDR&GPIO_Pin_6) != 0);
    }
    if(EXTI->PR&EXTI_Line7){
        EXTI_ClearITPendingBit(EXTI_Line7);
        CountEdge(3, (GPIOF->IDR&GPIO_Pin_7) != 0, (GPIOD->IDR&GPIO_Pin_7) != 0);
    }
}
