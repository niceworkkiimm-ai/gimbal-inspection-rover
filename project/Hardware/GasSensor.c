/* V8.13 / 2026-10-02: gas UART moved to UART4, PC10 TX / PC11 RX (AF8).
 * 9600 8N1, Modbus requests, CRC checks and sampling timing unchanged.
 * Base ESP now owns USART2 PA2/PA3; each UART has its own IRQ and queues.
 */
#include "GasSensor.h"
#include <string.h>

#define GAS_BAUD_RATE            9600U
#define GAS_RESPONSE_LENGTH      25U
#define GAS_RX_CAPACITY          64U
#define GAS_REQUEST_GAP_MS       200U
#define GAS_RESPONSE_TIMEOUT_MS  120U
#define GAS_INTERBYTE_TIMEOUT_MS 10U
#define GAS_STALE_MS             1500U

typedef struct { uint8_t byte; uint32_t cycles; } GasRxByte_t;
static volatile GasRxByte_t gas_rx_buffer[GAS_RX_CAPACITY];
static volatile uint8_t gas_rx_head, gas_rx_tail, gas_rx_fault;
static volatile uint8_t gas_tx_buffer[8];
static volatile uint8_t gas_tx_position, gas_tx_length;
static uint8_t gas_frame[GAS_RESPONSE_LENGTH];
static uint8_t gas_frame_length, gas_channel, gas_request_channel, gas_waiting;
static uint32_t gas_request_cycles, gas_byte_cycles;
static uint32_t gas_sample_cycles[GAS_SENSOR_CHANNEL_COUNT];
static GasSensorReading_t gas_readings[GAS_SENSOR_CHANNEL_COUNT];

static uint16_t Gas_ModbusCrc(const uint8_t *data, uint8_t length)
{
    uint16_t crc = 0xFFFFU;
    uint8_t i, bit;
    for (i = 0U; i < length; i++)
    {
        crc ^= data[i];
        for (bit = 0U; bit < 8U; bit++)
            crc = (crc & 1U) ? (uint16_t)((crc >> 1) ^ 0xA001U) :
                                (uint16_t)(crc >> 1);
    }
    return crc;
}

static uint8_t Gas_RxPop(uint8_t *byte, uint32_t *cycles)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (gas_rx_tail == gas_rx_head)
    {
        if (!primask) __enable_irq();
        return 0U;
    }
    *byte = gas_rx_buffer[gas_rx_tail].byte;
    *cycles = gas_rx_buffer[gas_rx_tail].cycles;
    gas_rx_tail = (uint8_t)((gas_rx_tail + 1U) % GAS_RX_CAPACITY);
    if (!primask) __enable_irq();
    return 1U;
}

static void Gas_ResetTransport(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    USART_ITConfig(UART4, USART_IT_TXE, DISABLE);
    gas_tx_length = gas_tx_position = 0U;
    gas_rx_tail = gas_rx_head;
    gas_rx_fault = 0U;
    if (!primask) __enable_irq();
    gas_frame_length = 0U;
}

static void Gas_SendRequest(uint8_t address)
{
    uint8_t request[8] = {0U, 3U, 0U, 0U, 0U, 10U, 0U, 0U};
    uint16_t crc;
    uint8_t i;
    request[0] = address;
    crc = Gas_ModbusCrc(request, 6U);
    request[6] = (uint8_t)crc;
    request[7] = (uint8_t)(crc >> 8);
    /* Publish the complete frame before enabling the TX interrupt. */
    for (i = 0U; i < 8U; i++) gas_tx_buffer[i] = request[i];
    gas_tx_position = 0U;
    gas_tx_length = 8U;
    USART_ITConfig(UART4, USART_IT_TXE, ENABLE);
}

static void Gas_MarkInvalid(uint8_t channel)
{
    gas_readings[channel].online = 0U;
    gas_readings[channel].valid = 0U;
}

static uint8_t Gas_ParseResponse(uint8_t channel)
{
    uint16_t parameter, received_crc;
    uint8_t decimal_code, unit, status;
    GasSensorReading_t *reading;
    if (channel >= GAS_SENSOR_CHANNEL_COUNT ||
        gas_frame_length != GAS_RESPONSE_LENGTH ||
        gas_frame[0] != (uint8_t)(channel + 1U) ||
        gas_frame[1] != 3U || gas_frame[2] != 20U)
        return 0U;
    received_crc = (uint16_t)gas_frame[23] | ((uint16_t)gas_frame[24] << 8);
    if (received_crc != Gas_ModbusCrc(gas_frame, 23U)) return 0U;
    parameter = ((uint16_t)gas_frame[3] << 8) | gas_frame[4];
    decimal_code = (uint8_t)((parameter >> 8) & 15U);
    unit = (uint8_t)(parameter >> 12);
    status = gas_frame[14]; /* Register 5: low byte only. */
    reading = &gas_readings[channel];
    reading->address = (uint8_t)(channel + 1U);
    reading->concentration_raw = ((uint32_t)((parameter >> 6) & 3U) << 16) |
                                ((uint32_t)gas_frame[5] << 8) | gas_frame[6];
    reading->unit_code = unit;
    reading->decimals = (uint8_t)(decimal_code >> 2);
    reading->status = status;
    reading->gas_type = gas_frame[19]; /* Register 8: high byte only. */
    reading->online = 1U;
    /* Alarm samples are measurable, NOT safe; faults/warmup are unusable. */
    reading->valid = ((decimal_code & 3U) == 0U) &&
                     (unit == 0U || unit == 2U || unit == 4U || unit == 6U || unit == 8U) &&
                     (status == 1U || status == 4U || status == 5U ||
                      status == 6U || status == 11U || status == 12U);
    /* Temperature/humidity encoding is unverified, deliberately not decoded. */
    return 1U;
}

void GasSensor_Init(void)
{
    GPIO_InitTypeDef gpio;
    USART_InitTypeDef usart;
    NVIC_InitTypeDef nvic;
    uint8_t i;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_UART4);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_UART4);
    gpio.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOC, &gpio);
    USART_StructInit(&usart);
    usart.USART_BaudRate = GAS_BAUD_RATE;
    usart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(UART4, &usart);
    gas_rx_head = gas_rx_tail = gas_rx_fault = 0U;
    gas_tx_length = gas_tx_position = 0U;
    gas_frame_length = gas_channel = gas_request_channel = gas_waiting = 0U;
    gas_request_cycles = DWT->CYCCNT;
    memset(gas_readings, 0, sizeof(gas_readings));
    for (i = 0U; i < GAS_SENSOR_CHANNEL_COUNT; i++)
        gas_readings[i].address = (uint8_t)(i + 1U);
    nvic.NVIC_IRQChannel = UART4_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority = 1;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
    USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);
    USART_Cmd(UART4, ENABLE);
}

void GasSensor_Poll(void)
{
    uint32_t now = DWT->CYCCNT;
    uint32_t cycles_per_ms = SystemCoreClock / 1000U;
    uint32_t arrival;
    uint8_t byte, i, budget = GAS_RX_CAPACITY;
    for (i = 0U; i < GAS_SENSOR_CHANNEL_COUNT; i++)
        if (gas_readings[i].online &&
            (uint32_t)(now - gas_sample_cycles[i]) >= cycles_per_ms * GAS_STALE_MS)
            Gas_MarkInvalid(i); /* Latch offline; never revive at DWT wrap. */
    if (gas_waiting)
    {
        if (gas_rx_fault || (uint32_t)(now - gas_request_cycles) >=
            cycles_per_ms * GAS_RESPONSE_TIMEOUT_MS)
        {
            Gas_MarkInvalid(gas_request_channel);
            Gas_ResetTransport(); /* Cancels a stalled TX without waiting. */
            gas_waiting = 0U;
            return;
        }
        while (budget-- && Gas_RxPop(&byte, &arrival))
        {
            if (gas_frame_length && (uint32_t)(arrival - gas_byte_cycles) >
                cycles_per_ms * GAS_INTERBYTE_TIMEOUT_MS) gas_frame_length = 0U;
            gas_byte_cycles = arrival;
            gas_frame[gas_frame_length++] = byte;
            /* Bounded sliding header search, timestamps are ISR arrival times. */
            while (gas_frame_length &&
                  (gas_frame[0] != (uint8_t)(gas_request_channel + 1U) ||
                  (gas_frame_length >= 2U && gas_frame[1] != 3U && gas_frame[1] != 0x83U) ||
                  (gas_frame_length >= 3U && gas_frame[1] == 3U && gas_frame[2] != 20U)))
            {
                gas_frame_length--;
                memmove(gas_frame, gas_frame + 1, gas_frame_length);
            }
            if (gas_frame_length == 5U && gas_frame[1] == 0x83U)
            {
                uint16_t crc = (uint16_t)gas_frame[3] | ((uint16_t)gas_frame[4] << 8);
                if (crc == Gas_ModbusCrc(gas_frame, 3U))
                {
                    Gas_MarkInvalid(gas_request_channel);
                    gas_waiting = 0U;
                    gas_frame_length = 0U;
                    break;
                }
                gas_frame_length = 0U;
            }
            if (gas_frame_length == GAS_RESPONSE_LENGTH)
            {
                if (Gas_ParseResponse(gas_request_channel))
                {
                    gas_sample_cycles[gas_request_channel] = arrival;
                    gas_waiting = 0U;
                    gas_frame_length = 0U;
                    break;
                }
                gas_frame_length--;
                memmove(gas_frame, gas_frame + 1, gas_frame_length);
            }
        }
        return;
    }
    if ((uint32_t)(now - gas_request_cycles) < cycles_per_ms * GAS_REQUEST_GAP_MS)
        return;
    Gas_ResetTransport();
    gas_request_channel = gas_channel;
    gas_request_cycles = now;
    gas_waiting = 1U;
    gas_channel = (uint8_t)((gas_channel + 1U) % GAS_SENSOR_CHANNEL_COUNT);
    Gas_SendRequest((uint8_t)(gas_request_channel + 1U));
}

uint8_t GasSensor_GetReading(uint8_t channel, GasSensorReading_t *reading)
{
    if (channel >= GAS_SENSOR_CHANNEL_COUNT || reading == 0) return 0U;
    if (gas_readings[channel].online &&
        (uint32_t)(DWT->CYCCNT - gas_sample_cycles[channel]) >=
        (SystemCoreClock / 1000U) * GAS_STALE_MS)
        Gas_MarkInvalid(channel);
    *reading = gas_readings[channel];
    return 1U;
}

void UART4_IRQHandler(void)
{
    uint32_t status = UART4->SR;
    uint8_t byte, next_head;
    /* Service TX independently of RX/errors. */
    if ((status & USART_FLAG_TXE) && (UART4->CR1 & USART_CR1_TXEIE))
    {
        if (gas_tx_position < gas_tx_length)
            USART_SendData(UART4, gas_tx_buffer[gas_tx_position++]);
        if (gas_tx_position >= gas_tx_length)
        {
            USART_ITConfig(UART4, USART_IT_TXE, DISABLE);
            gas_tx_length = 0U;
        }
    }
    if (status & (USART_FLAG_ORE | USART_FLAG_NE | USART_FLAG_FE | USART_FLAG_PE))
    {
        (void)UART4->DR;
        gas_rx_fault = 1U;
    }
    else if (status & USART_FLAG_RXNE)
    {
        byte = (uint8_t)UART4->DR;
        next_head = (uint8_t)((gas_rx_head + 1U) % GAS_RX_CAPACITY);
        if (next_head == gas_rx_tail) gas_rx_fault = 1U;
        else
        {
            gas_rx_buffer[gas_rx_head].byte = byte;
            gas_rx_buffer[gas_rx_head].cycles = DWT->CYCCNT;
            gas_rx_head = next_head;
        }
    }
}
