#include "ESP8266.h"
#include <stdio.h>

#define ESP8266_LINE_MAX 32

static volatile ESP8266_Mode_t control_mode = ESP_MODE_IMU;
static volatile float wireless_roll;
static volatile float wireless_pitch;
static char rx_line[ESP8266_LINE_MAX];
static uint8_t rx_length;

static uint8_t ParseAngle(const char **text, float *value)
{
    uint8_t negative = 0;
    uint8_t digits = 0;
    int32_t result = 0;

    if (**text == '-')
    {
        negative = 1;
        (*text)++;
    }
    else if (**text == '+')
    {
        (*text)++;
    }

    while ((**text >= '0') && (**text <= '9'))
    {
        result = result * 10 + (**text - '0');
        (*text)++;
        digits = 1;
    }

    if (!digits || (result > 90)) return 0;
    *value = negative ? -(float)result : (float)result;
    return 1;
}

static uint8_t TextEquals(const char *text, const char *expected)
{
    while (*text && *expected)
    {
        if (*text++ != *expected++) return 0;
    }
    return (*text == '\0' && *expected == '\0');
}

static void ProcessLine(char *line)
{
    const char *cursor;
    float roll;
    float pitch;

    if (TextEquals(line, "MODE,IMU"))
    {
        control_mode = ESP_MODE_IMU;
        return;
    }

    if (TextEquals(line, "MODE,WIFI"))
    {
        control_mode = ESP_MODE_WIFI;
        return;
    }

    if (TextEquals(line, "SCAN,HORIZONTAL"))
    {
        control_mode = ESP_MODE_SCAN_HORIZONTAL;
        return;
    }

    if (TextEquals(line, "SCAN,VERTICAL"))
    {
        control_mode = ESP_MODE_SCAN_VERTICAL;
        return;
    }

    if (TextEquals(line, "SCAN,ALL"))
    {
        control_mode = ESP_MODE_SCAN_ALL;
        return;
    }

    if (TextEquals(line, "SCAN,STOP"))
    {
        control_mode = ESP_MODE_WIFI;
        return;
    }

    cursor = line;
    if (cursor[0] != 'C' || cursor[1] != 'T' || cursor[2] != 'R' ||
        cursor[3] != 'L' || cursor[4] != ',') return;
    cursor += 5;

    if (!ParseAngle(&cursor, &roll) || *cursor++ != ',') return;
    if (!ParseAngle(&cursor, &pitch) || *cursor != '\0') return;

    /* Assign only after a complete, valid command has been received. */
    wireless_roll = roll;
    wireless_pitch = pitch;
    control_mode = ESP_MODE_WIFI;
}

void ESP8266_Init(uint32_t baudrate)
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
    usart.USART_BaudRate = baudrate;
    usart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART2, &usart);
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

    nvic.NVIC_IRQChannel = USART2_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 2;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
    USART_Cmd(USART2, ENABLE);
}

void ESP8266_RxByte(uint8_t byte)
{
    if (byte == '\r') return;

    if (byte == '\n')
    {
        rx_line[rx_length] = '\0';
        ProcessLine(rx_line);
        rx_length = 0;
        return;
    }

    if (rx_length < ESP8266_LINE_MAX - 1)
    {
        rx_line[rx_length++] = (char)byte;
    }
    else
    {
        /* A too-long line cannot be a valid command; discard it. */
        rx_length = 0;
    }
}

ESP8266_Mode_t ESP8266_GetControl(float *roll, float *pitch)
{
    ESP8266_Mode_t mode;

    __disable_irq();
    mode = control_mode;
    *roll = wireless_roll;
    *pitch = wireless_pitch;
    __enable_irq();
    return mode;
}

void ESP8266_SendStatus(float roll, float pitch, float distance)
{
    char message[64];
    int length;
    int index;

    length = sprintf(message, "STATUS,%d,%d,%d\r\n", (int)roll, (int)pitch, (int)distance);
    for (index = 0; index < length; index++)
    {
        while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
        USART_SendData(USART2, (uint16_t)message[index]);
    }
}
