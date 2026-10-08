#ifndef __TFT_ST7735_H
#define __TFT_ST7735_H

#include "stm32f4xx.h"

#define TFT_WIDTH   128U
#define TFT_HEIGHT  160U

#define TFT_BLACK       0x0000U
#define TFT_WHITE       0xFFFFU
#define TFT_RED         0xF800U
#define TFT_GREEN       0x07E0U
#define TFT_BLUE        0x001FU
#define TFT_YELLOW      0xFFE0U
#define TFT_CYAN        0x07FFU
#define TFT_DARK_GRAY   0x4208U
#define TFT_LIGHT_GRAY  0xC618U

void TFT_Init(void);
void TFT_Clear(uint16_t color);
void TFT_FillRect(uint16_t x, uint16_t y, uint16_t width, uint16_t height,
                  uint16_t color);
void TFT_DrawString(uint16_t x, uint16_t y, const char *text,
                    uint16_t foreground, uint16_t background);
void TFT_DrawSigned3(uint16_t x, uint16_t y, int32_t value,
                     uint16_t foreground, uint16_t background);
void TFT_DrawUnsigned3(uint16_t x, uint16_t y, uint32_t value,
                       uint16_t foreground, uint16_t background);

#endif
