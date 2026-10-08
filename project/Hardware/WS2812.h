#ifndef __WS2812_H
#define __WS2812_H

#include "stm32f4xx.h"

#define WS2812_SEGMENT_1_COUNT    15U
#define WS2812_SEGMENT_2_COUNT    15U
#define WS2812_LED_COUNT          (WS2812_SEGMENT_1_COUNT + \
                                   WS2812_SEGMENT_2_COUNT)
#define WS2812_BRIGHTNESS         64U  /* 64/255, approximately 25 percent. */

void WS2812_Init(void);
void WS2812_Service(void);
uint8_t WS2812_IsBusy(void);
uint8_t WS2812_ShowColor(uint8_t red, uint8_t green, uint8_t blue);

#endif
