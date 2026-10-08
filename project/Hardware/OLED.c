#include "OLED.h"
#include "OLED_Font.h"

/* 软件I2C: PB10=SCL, PB11=SDA（已测试OK） */
#define SCL_H()  GPIO_SetBits(GPIOB, GPIO_Pin_10)
#define SCL_L()  GPIO_ResetBits(GPIOB, GPIO_Pin_10)
#define SDA_H()  GPIO_SetBits(GPIOB, GPIO_Pin_11)
#define SDA_L()  GPIO_ResetBits(GPIOB, GPIO_Pin_11)

static void I2C_Delay(void) { volatile uint32_t i; for (i = 0; i < 20; i++) __NOP(); }

static void OLED_GPIO_Init(void)
{
    GPIO_InitTypeDef g;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    g.GPIO_Pin   = GPIO_Pin_10 | GPIO_Pin_11;
    g.GPIO_Mode  = GPIO_Mode_OUT;
    g.GPIO_OType = GPIO_OType_OD;
    g.GPIO_Speed = GPIO_Speed_50MHz;
    g.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOB, &g);
    SCL_H(); SDA_H();
}

static void I2C_Start(void)  { SDA_H(); I2C_Delay(); SCL_H(); I2C_Delay(); SDA_L(); I2C_Delay(); SCL_L(); I2C_Delay(); }
static void I2C_Stop(void)   { SDA_L(); I2C_Delay(); SCL_H(); I2C_Delay(); SDA_H(); I2C_Delay(); }

static uint8_t I2C_SendByte(uint8_t byte)
{
    uint8_t i, ack;
    for (i = 0; i < 8; i++)
    {
        if (byte & 0x80) SDA_H(); else SDA_L();
        byte <<= 1;
        I2C_Delay(); SCL_H(); I2C_Delay(); SCL_L(); I2C_Delay();
    }
    SDA_H(); I2C_Delay(); SCL_H(); I2C_Delay();          /* 读ACK */
    ack = (GPIOB->IDR & GPIO_Pin_11) ? 1 : 0;            /* 0=ACK, 1=NACK */
    SCL_L(); I2C_Delay();
    return ack;
}

static void OLED_WriteCommand(uint8_t cmd)
{
    I2C_Start(); I2C_SendByte(0x78); I2C_SendByte(0x00); I2C_SendByte(cmd); I2C_Stop();
}

static void OLED_WriteData(uint8_t data)
{
    I2C_Start(); I2C_SendByte(0x78); I2C_SendByte(0x40); I2C_SendByte(data); I2C_Stop();
}

static void OLED_SetCursor(uint8_t Y, uint8_t X)
{
    OLED_WriteCommand(0xB0 | Y);
    OLED_WriteCommand(0x10 | ((X & 0xF0) >> 4));
    OLED_WriteCommand(0x00 | (X & 0x0F));
}

void OLED_Clear(void)
{
    uint8_t i, j;
    for (j = 0; j < 8; j++) { OLED_SetCursor(j, 0); for (i = 0; i < 128; i++) OLED_WriteData(0x00); }
}

void OLED_ShowChar(uint8_t Line, uint8_t Column, char Char)
{
    uint8_t i;
    OLED_SetCursor((Line - 1) * 2, (Column - 1) * 8);
    for (i = 0; i < 8; i++) OLED_WriteData(OLED_F8x16[Char - ' '][i]);
    OLED_SetCursor((Line - 1) * 2 + 1, (Column - 1) * 8);
    for (i = 0; i < 8; i++) OLED_WriteData(OLED_F8x16[Char - ' '][i + 8]);
}

void OLED_ShowString(uint8_t Line, uint8_t Column, char *String)
{
    uint8_t i;
    for (i = 0; String[i] != '\0'; i++) OLED_ShowChar(Line, Column + i, String[i]);
}

static uint32_t OLED_Pow(uint32_t X, uint32_t Y)
{
    uint32_t R = 1; while (Y--) R *= X; return R;
}

void OLED_ShowNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
    uint8_t i;
    for (i = 0; i < Length; i++) OLED_ShowChar(Line, Column + i, Number / OLED_Pow(10, Length - i - 1) % 10 + '0');
}

void OLED_ShowSignedNum(uint8_t Line, uint8_t Column, int32_t Number, uint8_t Length)
{
    uint8_t i; uint32_t N;
    if (Number >= 0) { OLED_ShowChar(Line, Column, '+'); N = Number; }
    else             { OLED_ShowChar(Line, Column, '-'); N = -Number; }
    for (i = 0; i < Length; i++) OLED_ShowChar(Line, Column + i + 1, N / OLED_Pow(10, Length - i - 1) % 10 + '0');
}

void OLED_ShowHexNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
    uint8_t i, s;
    for (i = 0; i < Length; i++) { s = Number / OLED_Pow(16, Length - i - 1) % 16; OLED_ShowChar(Line, Column + i, s < 10 ? s + '0' : s - 10 + 'A'); }
}

void OLED_ShowBinNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
    uint8_t i;
    for (i = 0; i < Length; i++) OLED_ShowChar(Line, Column + i, Number / OLED_Pow(2, Length - i - 1) % 2 + '0');
}

void OLED_Init(void)
{
    uint32_t i;
    for (i = 0; i < 2000000; i++) __NOP();

    OLED_GPIO_Init();

    OLED_WriteCommand(0xAE); OLED_WriteCommand(0xD5); OLED_WriteCommand(0x80);
    OLED_WriteCommand(0xA8); OLED_WriteCommand(0x3F);
    OLED_WriteCommand(0xD3); OLED_WriteCommand(0x00);
    OLED_WriteCommand(0x40); OLED_WriteCommand(0xA1); OLED_WriteCommand(0xC8);
    OLED_WriteCommand(0xDA); OLED_WriteCommand(0x12);
    OLED_WriteCommand(0x81); OLED_WriteCommand(0xCF);
    OLED_WriteCommand(0xD9); OLED_WriteCommand(0xF1);
    OLED_WriteCommand(0xDB); OLED_WriteCommand(0x30);
    OLED_WriteCommand(0xA4); OLED_WriteCommand(0xA6);
    OLED_WriteCommand(0x8D); OLED_WriteCommand(0x14);
    OLED_WriteCommand(0xAF);
    OLED_Clear();
}
