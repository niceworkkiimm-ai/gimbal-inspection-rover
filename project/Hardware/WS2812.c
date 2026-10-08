#include "WS2812.h"

/*
 * 30-pixel WS2812 strip on PB6 / TIM4_CH1.
 * TIM4 runs at 84 MHz: 105 timer ticks form one 800 kHz data bit.
 * DMA1 Stream0 Channel2 feeds the next PWM duty at each CH1 compare event.
 */
#define WS2812_TIMER_PERIOD_TICKS  105U
#define WS2812_T0H_TICKS            29U
#define WS2812_T1H_TICKS            59U
#define WS2812_RESET_SLOTS           64U
#define WS2812_DATA_SLOTS          (WS2812_LED_COUNT * 24U)
#define WS2812_TOTAL_SLOTS         (WS2812_DATA_SLOTS + WS2812_RESET_SLOTS)
#define WS2812_RESET_US              80U

typedef enum
{
    WS2812_PHASE_IDLE = 0,
    WS2812_PHASE_SENDING,
    WS2812_PHASE_RESET
} WS2812_Phase_t;

static uint16_t pwm_buffer[WS2812_TOTAL_SLOTS];
static WS2812_Phase_t transfer_phase;
static uint32_t reset_started_cycles;

static uint8_t ScaleBrightness(uint8_t value)
{
    return (uint8_t)(((uint16_t)value * WS2812_BRIGHTNESS + 127U) / 255U);
}

static void EncodeByte(uint16_t *index, uint8_t value)
{
    uint8_t mask;

    for (mask = 0x80U; mask != 0U; mask >>= 1)
    {
        pwm_buffer[*index] = (value & mask) ?
                             WS2812_T1H_TICKS : WS2812_T0H_TICKS;
        (*index)++;
    }
}

static void StopTransfer(void)
{
    TIM_DMACmd(TIM4, TIM_DMA_CC1, DISABLE);
    DMA_Cmd(DMA1_Stream0, DISABLE);
    while (DMA_GetCmdStatus(DMA1_Stream0) != DISABLE);

    TIM_SetCompare1(TIM4, 0U);
    TIM_GenerateEvent(TIM4, TIM_EventSource_Update);
    TIM_Cmd(TIM4, DISABLE);

    DMA_ClearFlag(DMA1_Stream0,
                  DMA_FLAG_FEIF0 | DMA_FLAG_DMEIF0 | DMA_FLAG_TEIF0 |
                  DMA_FLAG_HTIF0 | DMA_FLAG_TCIF0);
    reset_started_cycles = DWT->CYCCNT;
    transfer_phase = WS2812_PHASE_RESET;
}

void WS2812_Init(void)
{
    GPIO_InitTypeDef gpio;
    TIM_TimeBaseInitTypeDef timer;
    TIM_OCInitTypeDef output_compare;
    DMA_InitTypeDef dma;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB |
                           RCC_AHB1Periph_DMA1, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

    /* Hold DIN low before PB6 is connected to the timer alternate function. */
    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin = GPIO_Pin_6;
    gpio.GPIO_Mode = GPIO_Mode_OUT;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_Init(GPIOB, &gpio);
    GPIO_ResetBits(GPIOB, GPIO_Pin_6);

    TIM_TimeBaseStructInit(&timer);
    timer.TIM_Prescaler = 0U;
    timer.TIM_Period = WS2812_TIMER_PERIOD_TICKS - 1U;
    timer.TIM_ClockDivision = TIM_CKD_DIV1;
    timer.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM4, &timer);

    TIM_OCStructInit(&output_compare);
    output_compare.TIM_OCMode = TIM_OCMode_PWM1;
    output_compare.TIM_OutputState = TIM_OutputState_Enable;
    output_compare.TIM_Pulse = 0U;
    output_compare.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC1Init(TIM4, &output_compare);
    TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM4, ENABLE);
    TIM_SetCompare1(TIM4, 0U);
    TIM_GenerateEvent(TIM4, TIM_EventSource_Update);
    TIM_Cmd(TIM4, DISABLE);

    DMA_DeInit(DMA1_Stream0);
    while (DMA_GetCmdStatus(DMA1_Stream0) != DISABLE);
    DMA_StructInit(&dma);
    dma.DMA_Channel = DMA_Channel_2;
    dma.DMA_PeripheralBaseAddr = (uint32_t)&TIM4->CCR1;
    dma.DMA_Memory0BaseAddr = (uint32_t)&pwm_buffer[1];
    dma.DMA_DIR = DMA_DIR_MemoryToPeripheral;
    dma.DMA_BufferSize = WS2812_TOTAL_SLOTS - 1U;
    dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
    dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    dma.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    dma.DMA_Mode = DMA_Mode_Normal;
    dma.DMA_Priority = DMA_Priority_VeryHigh;
    dma.DMA_FIFOMode = DMA_FIFOMode_Disable;
    dma.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
    dma.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    dma.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(DMA1_Stream0, &dma);

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_TIM4);
    gpio.GPIO_Mode = GPIO_Mode_AF;
    GPIO_Init(GPIOB, &gpio);

    transfer_phase = WS2812_PHASE_IDLE;
    (void)WS2812_ShowColor(0U, 0U, 0U);
}

void WS2812_Service(void)
{
    uint32_t reset_cycles;

    if (transfer_phase == WS2812_PHASE_SENDING)
    {
        if (DMA_GetFlagStatus(DMA1_Stream0, DMA_FLAG_TCIF0) != RESET ||
            DMA_GetFlagStatus(DMA1_Stream0, DMA_FLAG_FEIF0) != RESET ||
            DMA_GetFlagStatus(DMA1_Stream0, DMA_FLAG_DMEIF0) != RESET ||
            DMA_GetFlagStatus(DMA1_Stream0, DMA_FLAG_TEIF0) != RESET)
        {
            StopTransfer();
        }
    }
    else if (transfer_phase == WS2812_PHASE_RESET)
    {
        reset_cycles = (SystemCoreClock / 1000000U) * WS2812_RESET_US;
        if ((uint32_t)(DWT->CYCCNT - reset_started_cycles) >= reset_cycles)
            transfer_phase = WS2812_PHASE_IDLE;
    }
}

uint8_t WS2812_IsBusy(void)
{
    return transfer_phase != WS2812_PHASE_IDLE;
}

uint8_t WS2812_ShowColor(uint8_t red, uint8_t green, uint8_t blue)
{
    uint16_t led;
    uint16_t index = 0U;
    uint8_t scaled_red;
    uint8_t scaled_green;
    uint8_t scaled_blue;

    if (transfer_phase != WS2812_PHASE_IDLE)
        return 0U;

    scaled_red = ScaleBrightness(red);
    scaled_green = ScaleBrightness(green);
    scaled_blue = ScaleBrightness(blue);

    for (led = 0U; led < WS2812_LED_COUNT; led++)
    {
        /* WS2812 wire order is GRB, not RGB. */
        EncodeByte(&index, scaled_green);
        EncodeByte(&index, scaled_red);
        EncodeByte(&index, scaled_blue);
    }
    while (index < WS2812_TOTAL_SLOTS)
        pwm_buffer[index++] = 0U;

    TIM_DMACmd(TIM4, TIM_DMA_CC1, DISABLE);
    DMA_Cmd(DMA1_Stream0, DISABLE);
    while (DMA_GetCmdStatus(DMA1_Stream0) != DISABLE);
    DMA_ClearFlag(DMA1_Stream0,
                  DMA_FLAG_FEIF0 | DMA_FLAG_DMEIF0 | DMA_FLAG_TEIF0 |
                  DMA_FLAG_HTIF0 | DMA_FLAG_TCIF0);
    DMA_SetCurrDataCounter(DMA1_Stream0, WS2812_TOTAL_SLOTS - 1U);

    TIM_Cmd(TIM4, DISABLE);
    TIM_SetCounter(TIM4, 0U);
    TIM_SetCompare1(TIM4, pwm_buffer[0]);
    TIM_GenerateEvent(TIM4, TIM_EventSource_Update);

    DMA_Cmd(DMA1_Stream0, ENABLE);
    TIM_DMACmd(TIM4, TIM_DMA_CC1, ENABLE);
    TIM_Cmd(TIM4, ENABLE);
    transfer_phase = WS2812_PHASE_SENDING;
    return 1U;
}
