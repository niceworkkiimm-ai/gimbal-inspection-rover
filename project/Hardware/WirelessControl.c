/* V8.13 / 2026-10-02: PCB swaps the two UART roles.
 * Base ESP now uses USART2, PA2 TX / PA3 RX (AF7); gas uses UART4.
 * 115200 8N1, bounded TX/RX queues, packet-age checks and protocol unchanged.
 * USART2 uses APB1; USART_Init derives the baud divisor from PCLK1.
 */
#include "WirelessControl.h"
#include "HandheldProtocol.h"
#include "CarMotor.h"
#include "RouteLog.h"
#include <stdio.h>

#define RX_LINE_CAPACITY       160U
#define RX_QUEUE_CAPACITY       8U
#define LINK_TIMEOUT_MS       500U
#define TX_CAPACITY          1024U /* V7.1: reserve >512 bytes for priority telemetry/control. */
#define TX_STALL_MS             50U

static volatile uint8_t tx_buffer[TX_CAPACITY];
static volatile uint16_t tx_head, tx_tail;
static volatile uint32_t tx_progress_cycles;
static volatile uint32_t tx_dropped_frames, tx_stalls;
static uint8_t tx_resync;

/* Whole-frame enqueue, main-loop producer / IRQ consumer; never busy-waits. */
static void QueueTx(const char *line, uint16_t length)
{
    uint32_t primask = __get_PRIMASK();
    uint16_t free_bytes, i;
    __disable_irq();
    free_bytes = (uint16_t)((tx_tail - tx_head - 1U) & (TX_CAPACITY - 1U));
    if (length + tx_resync > free_bytes)
    {
        tx_dropped_frames++;
        if (!primask) __enable_irq();
        return; /* Drop the new complete frame, never part of a frame. */
    }
    if (tx_head == tx_tail) tx_progress_cycles = DWT->CYCCNT;
    if (tx_resync)
    {
        tx_buffer[tx_head] = '\n';
        tx_head = (uint16_t)((tx_head + 1U) & (TX_CAPACITY - 1U));
        tx_resync = 0U;
    }
    for (i = 0U; i < length; i++)
    {
        tx_buffer[tx_head] = (uint8_t)line[i];
        tx_head = (uint16_t)((tx_head + 1U) & (TX_CAPACITY - 1U));
    }
    USART_ITConfig(USART2, USART_IT_TXE, ENABLE);
    if (!primask) __enable_irq();
}

static void PollTx(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (tx_head != tx_tail && (uint32_t)(DWT->CYCCNT - tx_progress_cycles) >
        (SystemCoreClock / 1000U) * TX_STALL_MS)
    {
        USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
        tx_tail = tx_head;
        tx_resync = 1U; /* Terminate any partial line before fresh telemetry. */
        tx_stalls++;
    }
    if (!primask) __enable_irq();
}

typedef struct
{
    char data[RX_LINE_CAPACITY];
    uint8_t length;
    uint32_t arrival_cycles;
} RxLine_t;

static volatile RxLine_t rx_queue[RX_QUEUE_CAPACITY];
static volatile uint8_t rx_head;
static volatile uint8_t rx_tail;
static volatile char rx_build[RX_LINE_CAPACITY];
static volatile uint8_t rx_build_length;
static volatile uint8_t rx_discard_line;
static volatile uint8_t rx_fault;
static uint8_t hand_mode_valid, hand_mode, hand_epoch;

static WirelessControlData_t latest_data;
static uint32_t latest_arrival_cycles;
static uint32_t last_sequence;
static volatile uint8_t latest_valid;
static volatile uint8_t sequence_valid;
static WirelessControlMode_t control_mode = WIRELESS_MODE_HANDHELD;

static uint8_t TextEquals(const char *text, uint8_t length,
                          const char *expected)
{
    uint8_t i = 0;

    while (i < length && expected[i] != '\0')
    {
        if (text[i] != expected[i])
            return 0;
        i++;
    }
    return (i == length && expected[i] == '\0');
}

static uint8_t ParseModeFrame(const RxLine_t *line,
                              WirelessControlMode_t *mode)
{
    if (TextEquals(line->data, line->length, "M,HORIZONTAL"))
        *mode = WIRELESS_MODE_SCAN_HORIZONTAL;
    else if (TextEquals(line->data, line->length, "M,VERTICAL"))
        *mode = WIRELESS_MODE_SCAN_VERTICAL;
    else if (TextEquals(line->data, line->length, "M,ALL"))
        *mode = WIRELESS_MODE_SCAN_ALL;
    else if (TextEquals(line->data, line->length, "M,STOP"))
        *mode = WIRELESS_MODE_STOPPED;
    else if (TextEquals(line->data, line->length, "M,HANDHELD"))
        *mode = WIRELESS_MODE_HANDHELD;
    else
        return 0;

    return 1;
}

static uint8_t ParseUnsigned(const char **cursor, const char *end,
                             uint32_t *value, char delimiter)
{
    uint32_t result = 0;
    uint8_t digits = 0;

    while (*cursor < end && **cursor >= '0' && **cursor <= '9')
    {
        uint32_t digit = (uint32_t)(**cursor - '0');
        if (result > 429496729U ||
            (result == 429496729U && digit > 5U))
            return 0;
        result = result * 10U + digit;
        (*cursor)++;
        digits = 1;
    }

    if (!digits || *cursor >= end || **cursor != delimiter)
        return 0;

    (*cursor)++;
    *value = result;
    return 1;
}

static uint8_t ParseFloat(const char **cursor, const char *end, float *value)
{
    float result = 0.0f;
    float fraction_scale = 0.1f;
    uint8_t negative = 0;
    uint8_t digits = 0;

    if (*cursor < end && (**cursor == '-' || **cursor == '+'))
    {
        negative = (**cursor == '-');
        (*cursor)++;
    }

    while (*cursor < end && **cursor >= '0' && **cursor <= '9')
    {
        result = result * 10.0f + (float)(**cursor - '0');
        if (result > 1000.0f)
            return 0;
        (*cursor)++;
        digits = 1;
    }

    if (*cursor < end && **cursor == '.')
    {
        (*cursor)++;
        while (*cursor < end && **cursor >= '0' && **cursor <= '9')
        {
            result += (float)(**cursor - '0') * fraction_scale;
            fraction_scale *= 0.1f;
            (*cursor)++;
            digits = 1;
        }
    }

    if (!digits)
        return 0;

    *value = negative ? -result : result;
    return 1;
}

static uint8_t ParseFrame(const RxLine_t *line, uint32_t *sequence,
                          float *roll, float *pitch)
{
    const char *cursor = line->data;
    const char *end = line->data + line->length;

    if (line->length < 7U || *cursor++ != 'G' || cursor >= end || *cursor++ != ',')
        return 0;

    if (!ParseUnsigned(&cursor, end, sequence, ','))
        return 0;
    if (!ParseFloat(&cursor, end, roll) || cursor >= end || *cursor++ != ',')
        return 0;
    if (!ParseFloat(&cursor, end, pitch) || cursor != end)
        return 0;
    if (*roll < -180.0f || *roll > 180.0f ||
        *pitch < -180.0f || *pitch > 180.0f)
        return 0;

    return 1;
}

static uint8_t PopLine(RxLine_t *line)
{
    uint32_t primask = __get_PRIMASK();
    uint8_t i;

    __disable_irq();
    if (rx_tail == rx_head)
    {
        if (!primask) __enable_irq();
        return 0;
    }

    line->length = rx_queue[rx_tail].length;
    line->arrival_cycles = rx_queue[rx_tail].arrival_cycles;
    for (i = 0; i < line->length; i++)
        line->data[i] = rx_queue[rx_tail].data[i];
    rx_tail = (uint8_t)((rx_tail + 1U) % RX_QUEUE_CAPACITY);
    if (!primask) __enable_irq();
    return 1;
}

void WirelessControl_Init(void)
{
    GPIO_InitTypeDef gpio;
    USART_InitTypeDef usart;
    NVIC_InitTypeDef nvic;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);
    gpio.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &gpio);

    USART_StructInit(&usart);
    usart.USART_BaudRate = 115200;
    usart.USART_WordLength = USART_WordLength_8b;
    usart.USART_StopBits = USART_StopBits_1;
    usart.USART_Parity = USART_Parity_No;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART2, &usart);

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    nvic.NVIC_IRQChannel = USART2_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
    USART_Cmd(USART2, ENABLE);
}

void WirelessControl_Poll(void)
{
    RxLine_t line;

    PollTx();
    /* Expire sequence history BEFORE processing a restarted transmitter. */
    if (latest_valid && (uint32_t)(DWT->CYCCNT-latest_arrival_cycles) >
        (SystemCoreClock/1000U)*LINK_TIMEOUT_MS)
    {
        latest_valid=0U;sequence_valid=0U;
    }
    while (PopLine(&line))
    {
        uint32_t sequence;
        uint32_t delta;
        float roll;
        float pitch;
        WirelessControlMode_t mode;
        HandheldFrame hand;
        uint8_t extended=0U;

        /* Queued control must retain UART arrival age, never rebase to poll time. */
        if ((uint32_t)(DWT->CYCCNT-line.arrival_cycles) >
            (SystemCoreClock/1000U)*250U) continue;
        if (ParseModeFrame(&line, &mode))
        {
            control_mode = mode;
            continue;
        }
        if(line.length>=3U&&line.data[0]=='J'&&line.data[1]=='1'&&line.data[2]==','){
            RouteLog_Query(line.data,line.length);
            continue;
        }
        if (line.length>=3U && line.data[0]=='H' && line.data[1]=='1' && line.data[2]==',')
        {
            if (!HH_Parse(line.data,line.length,&hand)) { rx_fault=1U;continue; }
            extended=1U;sequence=hand.sequence;
            roll=(float)hand.roll/10.0f;pitch=(float)hand.pitch/10.0f;
        }
        else if (!ParseFrame(&line, &sequence, &roll, &pitch)) continue;

        if (sequence_valid)
        {
            delta = sequence - last_sequence;
            if (delta == 0U || delta >= 0x80000000U)
                continue;
        }

        last_sequence = sequence;
        sequence_valid = 1;
        if (extended)
        {
            /* Repeated gimbal packets must NOT cancel web automatic scans.
               A web stop remains stopped until an actual button-mode change. */
            if (!hand_mode_valid || hand_mode!=hand.mode || hand_epoch!=hand.epoch)
            {
                control_mode=hand.mode==1U?WIRELESS_MODE_CAR:
                             hand.mode==2U?WIRELESS_MODE_STOPPED:WIRELESS_MODE_HANDHELD;
                hand_mode=hand.mode;hand_epoch=hand.epoch;hand_mode_valid=1U;
            }
        }
        latest_data.protocol=extended;
        latest_data.ready=extended?hand.ready:1U;
        latest_data.requested_mode=extended?hand.mode:0U;
        latest_data.mode_epoch=extended?hand.epoch:0U;
        latest_data.sequence = sequence;
        latest_data.roll = roll;
        latest_data.pitch = pitch;
        latest_data.packet_count++;
        latest_arrival_cycles = line.arrival_cycles;
        latest_data.arrival_cycles = line.arrival_cycles;
        latest_valid = 1;
    }
}

uint8_t WirelessControl_ConsumeFault(void)
{
    uint32_t primask=__get_PRIMASK();
    uint8_t fault;
    __disable_irq();fault=rx_fault;rx_fault=0U;
    if(!primask)__enable_irq();
    return fault;
}

void WirelessControl_SendVehicle(uint32_t sequence,uint8_t mode,uint8_t state,
                                 const int16_t pwm[4])
{
    char line[80];
    int n=snprintf(line,sizeof(line),"V,%lu,%u,%u,%d,%d,%d,%d\n",
        (unsigned long)sequence,(unsigned)mode,(unsigned)state,
        (int)pwm[0],(int)pwm[1],(int)pwm[2],(int)pwm[3]);
    if(n>0&&n<(int)sizeof(line))QueueTx(line,(uint16_t)n);
}

void WirelessControl_SendEncoder(const EncoderFrame *frame)
{
    char line[96];
    int n=Encoder_Format(line,sizeof(line),frame);
    if(n>0)QueueTx(line,(uint16_t)n);
}

void WirelessControl_SendWheelSpeed(const WheelSpeedFrame *frame)
{
    char line[192];
    int n=WheelSpeedFrame_Format(line,sizeof(line),frame);
    if(n>0)QueueTx(line,(uint16_t)n);
}
void WirelessControl_SendLog(void)
{
    char line[JOURNAL_LINE_MAX];int n;
    /* Main is the only producer. IRQ can only increase free space here.
     * One background line per pass, leaving room for a full normal burst.
     */
    if(((tx_tail-tx_head-1U)&(TX_CAPACITY-1U))<768U)return;
    n=RouteLog_Next(line,sizeof(line));
    if(n>0){QueueTx(line,(uint16_t)n);RouteLog_Sent();}
}

WirelessControlMode_t WirelessControl_GetMode(void)
{
    return control_mode;
}

uint8_t WirelessControl_GetLatest(WirelessControlData_t *data)
{
    uint32_t timeout_cycles = (SystemCoreClock / 1000U) * LINK_TIMEOUT_MS;

    if (!latest_valid || (uint32_t)(DWT->CYCCNT - latest_arrival_cycles) > timeout_cycles)
    {
        latest_valid = 0;
        sequence_valid = 0;
        return 0;
    }

    *data = latest_data;
    return 1;
}

void WirelessControl_SendTelemetry(uint32_t sequence,
                                   int16_t roll_tenths,
                                   int16_t pitch_tenths,
                                   int16_t distance_tenths,
                                   uint8_t distance_valid,
                                   uint8_t link_ok)
{
    char line[64];
    int length;

    length = snprintf(line, sizeof(line), "T,%lu,%d,%d,%d,%u,%u\n",
                      (unsigned long)sequence,
                      (int)roll_tenths,
                      (int)pitch_tenths,
                      (int)distance_tenths,
                      (unsigned int)(distance_valid ? 1U : 0U),
                      (unsigned int)(link_ok ? 1U : 0U));
    if (length <= 0 || length >= (int)sizeof(line))
        return;

    QueueTx(line, (uint16_t)length);
}

void WirelessControl_SendGasTelemetry(uint32_t sequence,
                                      uint8_t channel,
                                      uint8_t online,
                                      uint8_t valid,
                                      uint32_t concentration_raw,
                                      uint8_t unit_code,
                                      uint8_t decimals,
                                      uint8_t status,
                                      uint8_t gas_type)
{
    char line[96];
    int length;

    length = snprintf(line, sizeof(line), "G2,%lu,%u,%u,%u,%lu,%u,%u,%u,%u\n",
                      (unsigned long)sequence,
                      (unsigned int)(channel + 1U),
                      (unsigned int)(online ? 1U : 0U),
                      (unsigned int)(valid ? 1U : 0U),
                      (unsigned long)concentration_raw,
                      (unsigned int)unit_code,
                      (unsigned int)decimals,
                      (unsigned int)status,
                      (unsigned int)gas_type);
    if (length <= 0 || length >= (int)sizeof(line))
        return;

    QueueTx(line, (uint16_t)length);
}

void USART2_IRQHandler(void)
{
    uint32_t status = USART2->SR;
    uint8_t byte;
    uint8_t next_head;
    uint8_t i;

    /* Always service TX before RX's existing early-return paths. */
    if ((status & USART_FLAG_TXE) && (USART2->CR1 & USART_CR1_TXEIE))
    {
        if (tx_tail != tx_head)
        {
            USART_SendData(USART2, tx_buffer[tx_tail]);
            tx_tail = (uint16_t)((tx_tail + 1U) & (TX_CAPACITY - 1U));
            tx_progress_cycles = DWT->CYCCNT;
        }
        if (tx_tail == tx_head) USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
    }

    if (status & (USART_FLAG_ORE | USART_FLAG_NE | USART_FLAG_FE | USART_FLAG_PE))
    {
        (void)USART2->DR;
        rx_build_length = 0;
        rx_discard_line = 1;
        rx_tail=rx_head;
        latest_valid=0U;sequence_valid=0U;
        rx_fault=1U;
        return;
    }

    if (!(status & USART_FLAG_RXNE))
        return;

    byte = (uint8_t)USART2->DR;
    if (byte == '\r')
        return;

    if (byte == '\n')
    {
        if (!rx_discard_line && rx_build_length > 0U)
        {
            next_head = (uint8_t)((rx_head + 1U) % RX_QUEUE_CAPACITY);
            if (next_head == rx_tail)
            {
                rx_tail=rx_head;rx_fault=1U;
                latest_valid=0U;sequence_valid=0U;
            }

            for (i = 0; i < rx_build_length; i++)
                rx_queue[rx_head].data[i] = rx_build[i];
            rx_queue[rx_head].length = rx_build_length;
            rx_queue[rx_head].arrival_cycles = DWT->CYCCNT;
            rx_head = next_head;
        }
        rx_build_length = 0;
        rx_discard_line = 0;
        return;
    }

    if (rx_discard_line)
        return;

    if (rx_build_length >= RX_LINE_CAPACITY)
    {
        rx_build_length = 0;
        rx_discard_line = 1;
        rx_fault=1U;
        return;
    }

    rx_build[rx_build_length++] = (char)byte;
}
