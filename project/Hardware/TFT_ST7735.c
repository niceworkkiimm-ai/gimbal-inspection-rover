#include "TFT_ST7735.h"
#include "OLED_Font.h"
#include "Delay.h"

/* M144Z-M4 independent GPIOs: PG6=SCL, PG7=SDA, PG11=DC, PG13=RST.
 * CS is tied to GND; LEDA is powered externally through a resistor. */
#define TFT_PORT       GPIOG
#define TFT_SCL_PIN    GPIO_Pin_6
#define TFT_SDA_PIN    GPIO_Pin_7
#define TFT_DC_PIN     GPIO_Pin_11
#define TFT_RST_PIN    GPIO_Pin_13

#define TFT_X_OFFSET   2U
#define TFT_Y_OFFSET   1U

#define PIN_HIGH(pin)  (TFT_PORT->BSRRL = (uint16_t)(pin))
#define PIN_LOW(pin)   (TFT_PORT->BSRRH = (uint16_t)(pin))

static void TFT_SPI_Delay(void)
{
    uint32_t start_cycles = DWT->CYCCNT;

    while ((uint32_t)(DWT->CYCCNT - start_cycles) < 32U);
}

static void TFT_WriteByte(uint8_t data)
{
    uint8_t i;

    for (i = 0; i < 8U; i++)
    {
        PIN_LOW(TFT_SCL_PIN);
        if (data & 0x80U) PIN_HIGH(TFT_SDA_PIN);
        else              PIN_LOW(TFT_SDA_PIN);
        TFT_SPI_Delay();
        PIN_HIGH(TFT_SCL_PIN);
        TFT_SPI_Delay();
        data <<= 1;
    }

    PIN_LOW(TFT_SCL_PIN);
}

static void TFT_WriteCommand(uint8_t command)
{
    PIN_LOW(TFT_DC_PIN);
    TFT_WriteByte(command);
}

static void TFT_WriteData8(uint8_t data)
{
    PIN_HIGH(TFT_DC_PIN);
    TFT_WriteByte(data);
}

static void TFT_BeginData(void)
{
    PIN_HIGH(TFT_DC_PIN);
}

static void TFT_EndData(void)
{
}

static void TFT_WriteColor(uint16_t color)
{
    TFT_WriteByte((uint8_t)(color >> 8));
    TFT_WriteByte((uint8_t)color);
}

static void TFT_SetWindow(uint16_t x_start, uint16_t y_start,
                          uint16_t x_end, uint16_t y_end)
{
    x_start += TFT_X_OFFSET;
    x_end += TFT_X_OFFSET;
    y_start += TFT_Y_OFFSET;
    y_end += TFT_Y_OFFSET;

    TFT_WriteCommand(0x2AU);
    TFT_WriteData8(0x00U);
    TFT_WriteData8((uint8_t)x_start);
    TFT_WriteData8(0x00U);
    TFT_WriteData8((uint8_t)x_end);

    TFT_WriteCommand(0x2BU);
    TFT_WriteData8(0x00U);
    TFT_WriteData8((uint8_t)y_start);
    TFT_WriteData8(0x00U);
    TFT_WriteData8((uint8_t)y_end);
    TFT_WriteCommand(0x2CU);
}

static void TFT_GPIO_Init(void)
{
    GPIO_InitTypeDef gpio;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);
    gpio.GPIO_Pin = TFT_SCL_PIN | TFT_SDA_PIN |
                    TFT_DC_PIN | TFT_RST_PIN;
    gpio.GPIO_Mode = GPIO_Mode_OUT;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_Speed = GPIO_Speed_25MHz;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(TFT_PORT, &gpio);

    PIN_LOW(TFT_SCL_PIN);
    PIN_HIGH(TFT_SDA_PIN);
    PIN_HIGH(TFT_DC_PIN);
    PIN_HIGH(TFT_RST_PIN);
}

void TFT_Init(void)
{
    TFT_GPIO_Init();
    Delay_ms(200);
    PIN_HIGH(TFT_RST_PIN);
    Delay_ms(20);
    PIN_LOW(TFT_RST_PIN);
    Delay_ms(20);
    PIN_HIGH(TFT_RST_PIN);
    Delay_ms(150);

    TFT_WriteCommand(0x01U);
    Delay_ms(150);

    TFT_WriteCommand(0x11U);
    Delay_ms(150);

    TFT_WriteCommand(0xB1U);
    TFT_WriteData8(0x01U); TFT_WriteData8(0x2CU); TFT_WriteData8(0x2DU);
    TFT_WriteCommand(0xB2U);
    TFT_WriteData8(0x01U); TFT_WriteData8(0x2CU); TFT_WriteData8(0x2DU);
    TFT_WriteCommand(0xB3U);
    TFT_WriteData8(0x01U); TFT_WriteData8(0x2CU); TFT_WriteData8(0x2DU);
    TFT_WriteData8(0x01U); TFT_WriteData8(0x2CU); TFT_WriteData8(0x2DU);
    TFT_WriteCommand(0xB4U); TFT_WriteData8(0x07U);

    TFT_WriteCommand(0xC0U);
    TFT_WriteData8(0xA2U); TFT_WriteData8(0x02U); TFT_WriteData8(0x84U);
    TFT_WriteCommand(0xC1U); TFT_WriteData8(0xC5U);
    TFT_WriteCommand(0xC2U); TFT_WriteData8(0x0AU); TFT_WriteData8(0x00U);
    TFT_WriteCommand(0xC3U); TFT_WriteData8(0x8AU); TFT_WriteData8(0x2AU);
    TFT_WriteCommand(0xC4U); TFT_WriteData8(0x8AU); TFT_WriteData8(0xEEU);
    TFT_WriteCommand(0xC5U); TFT_WriteData8(0x0EU);
    TFT_WriteCommand(0x36U); TFT_WriteData8(0xC0U);

    TFT_WriteCommand(0xE0U);
    TFT_WriteData8(0x0FU); TFT_WriteData8(0x1AU); TFT_WriteData8(0x0FU);
    TFT_WriteData8(0x18U); TFT_WriteData8(0x2FU); TFT_WriteData8(0x28U);
    TFT_WriteData8(0x20U); TFT_WriteData8(0x22U); TFT_WriteData8(0x1FU);
    TFT_WriteData8(0x1BU); TFT_WriteData8(0x23U); TFT_WriteData8(0x37U);
    TFT_WriteData8(0x00U); TFT_WriteData8(0x07U); TFT_WriteData8(0x02U);
    TFT_WriteData8(0x10U);

    TFT_WriteCommand(0xE1U);
    TFT_WriteData8(0x0FU); TFT_WriteData8(0x1BU); TFT_WriteData8(0x0FU);
    TFT_WriteData8(0x17U); TFT_WriteData8(0x33U); TFT_WriteData8(0x2CU);
    TFT_WriteData8(0x29U); TFT_WriteData8(0x2EU); TFT_WriteData8(0x30U);
    TFT_WriteData8(0x30U); TFT_WriteData8(0x39U); TFT_WriteData8(0x3FU);
    TFT_WriteData8(0x00U); TFT_WriteData8(0x07U); TFT_WriteData8(0x03U);
    TFT_WriteData8(0x10U);

    TFT_WriteCommand(0x2AU);
    TFT_WriteData8(0x00U); TFT_WriteData8(0x00U);
    TFT_WriteData8(0x00U); TFT_WriteData8(0x7FU);
    TFT_WriteCommand(0x2BU);
    TFT_WriteData8(0x00U); TFT_WriteData8(0x00U);
    TFT_WriteData8(0x00U); TFT_WriteData8(0x9FU);
    TFT_WriteCommand(0xF0U); TFT_WriteData8(0x01U);
    TFT_WriteCommand(0xF6U); TFT_WriteData8(0x00U);
    TFT_WriteCommand(0x3AU); TFT_WriteData8(0x05U);
    TFT_WriteCommand(0x29U);
    Delay_ms(20);

    TFT_Clear(TFT_BLACK);
}

void TFT_FillRect(uint16_t x, uint16_t y, uint16_t width, uint16_t height,
                  uint16_t color)
{
    uint32_t pixels;

    if (x >= TFT_WIDTH || y >= TFT_HEIGHT || width == 0U || height == 0U)
        return;
    if (x + width > TFT_WIDTH) width = TFT_WIDTH - x;
    if (y + height > TFT_HEIGHT) height = TFT_HEIGHT - y;

    TFT_SetWindow(x, y, (uint16_t)(x + width - 1U),
                  (uint16_t)(y + height - 1U));
    pixels = (uint32_t)width * height;
    TFT_BeginData();
    while (pixels--) TFT_WriteColor(color);
    TFT_EndData();
}

void TFT_Clear(uint16_t color)
{
    TFT_FillRect(0U, 0U, TFT_WIDTH, TFT_HEIGHT, color);
}

static void TFT_DrawChar(uint16_t x, uint16_t y, char character,
                         uint16_t foreground, uint16_t background)
{
    uint8_t row;
    uint8_t column;
    uint8_t upper;
    uint8_t lower;
    uint8_t index;

    if (character < ' ' || character > '~' || x + 8U > TFT_WIDTH ||
        y + 16U > TFT_HEIGHT)
        return;

    index = (uint8_t)(character - ' ');
    TFT_SetWindow(x, y, (uint16_t)(x + 7U), (uint16_t)(y + 15U));
    TFT_BeginData();
    for (row = 0; row < 16U; row++)
    {
        for (column = 0; column < 8U; column++)
        {
            upper = OLED_F8x16[index][column];
            lower = OLED_F8x16[index][column + 8U];
            if (row < 8U)
                TFT_WriteColor((upper & (1U << row)) ? foreground : background);
            else
                TFT_WriteColor((lower & (1U << (row - 8U))) ? foreground : background);
        }
    }
    TFT_EndData();
}

void TFT_DrawString(uint16_t x, uint16_t y, const char *text,
                    uint16_t foreground, uint16_t background)
{
    while (*text != '\0' && x + 8U <= TFT_WIDTH)
    {
        TFT_DrawChar(x, y, *text, foreground, background);
        x += 8U;
        text++;
    }
}

void TFT_DrawSigned3(uint16_t x, uint16_t y, int32_t value,
                     uint16_t foreground, uint16_t background)
{
    char text[5];
    uint32_t magnitude;

    if (value > 999) value = 999;
    if (value < -999) value = -999;
    text[0] = value < 0 ? '-' : '+';
    magnitude = value < 0 ? (uint32_t)(-value) : (uint32_t)value;
    text[1] = (char)('0' + magnitude / 100U);
    text[2] = (char)('0' + (magnitude / 10U) % 10U);
    text[3] = (char)('0' + magnitude % 10U);
    text[4] = '\0';
    TFT_DrawString(x, y, text, foreground, background);
}

void TFT_DrawUnsigned3(uint16_t x, uint16_t y, uint32_t value,
                       uint16_t foreground, uint16_t background)
{
    char text[4];

    if (value > 999U) value = 999U;
    text[0] = (char)('0' + value / 100U);
    text[1] = (char)('0' + (value / 10U) % 10U);
    text[2] = (char)('0' + value % 10U);
    text[3] = '\0';
    TFT_DrawString(x, y, text, foreground, background);
}
