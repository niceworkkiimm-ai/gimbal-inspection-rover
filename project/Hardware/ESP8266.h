#ifndef __ESP8266_H
#define __ESP8266_H

#include "stm32f4xx.h"

typedef enum
{
    ESP_MODE_IMU = 0,
    ESP_MODE_WIFI,
    ESP_MODE_SCAN_HORIZONTAL,
    ESP_MODE_SCAN_VERTICAL,
    ESP_MODE_SCAN_ALL
} ESP8266_Mode_t;

/* USART2: PA2=STM32 TX -> ESP RX, PA3=STM32 RX <- ESP TX. */
void ESP8266_Init(uint32_t baudrate);
void ESP8266_RxByte(uint8_t byte);
ESP8266_Mode_t ESP8266_GetControl(float *roll, float *pitch);
void ESP8266_SendStatus(float roll, float pitch, float distance);

#endif
