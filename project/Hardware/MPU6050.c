#include "MPU6050.h"
#include <math.h>

#define RAD_TO_DEG 57.295779513082320876798154814105

/* MPU6050 寄存器 */
#define WHO_AM_I_REG     0x75
#define PWR_MGMT_1_REG   0x6B
#define SMPLRT_DIV_REG   0x19
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG   0x41
#define GYRO_CONFIG_REG  0x1B
#define GYRO_XOUT_H_REG  0x43

#define MPU6050_ADDR  0xD0   /* 8位地址 (0x68<<1) */
const double Accel_Z_corrector = 14418.0;

static uint32_t timer;

Kalman_t KalmanX = { .Q_angle = 0.001f, .Q_bias = 0.003f, .R_measure = 0.03f };
Kalman_t KalmanY = { .Q_angle = 0.001f, .Q_bias = 0.003f, .R_measure = 0.03f };

/* ========== 软件I2C (PB10=SCL, PB11=SDA) ========== */
#define SCL_H()  GPIO_SetBits(GPIOB, GPIO_Pin_10)
#define SCL_L()  GPIO_ResetBits(GPIOB, GPIO_Pin_10)
#define SDA_H()  GPIO_SetBits(GPIOB, GPIO_Pin_11)
#define SDA_L()  GPIO_ResetBits(GPIOB, GPIO_Pin_11)
#define SDA_READ() ((GPIOB->IDR & GPIO_Pin_11) ? 1 : 0)

static void I2C_Delay(void) { volatile uint32_t i; for (i = 0; i < 20; i++) __NOP(); }

static void MPU_I2C_Init(void)
{
    GPIO_InitTypeDef g;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    g.GPIO_Pin   = GPIO_Pin_10 | GPIO_Pin_11;
    g.GPIO_Mode  = GPIO_Mode_OUT;
    g.GPIO_OType = GPIO_OType_OD;      /* 开漏（需要外部上拉，OLED模块自带） */
    g.GPIO_Speed = GPIO_Speed_50MHz;
    g.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOB, &g);
    SCL_H(); SDA_H();
}

static void I2C_Start(void)  { SDA_H(); I2C_Delay(); SCL_H(); I2C_Delay(); SDA_L(); I2C_Delay(); SCL_L(); I2C_Delay(); }
static void I2C_Stop(void)   { SDA_L(); I2C_Delay(); SCL_H(); I2C_Delay(); SDA_H(); I2C_Delay(); }

static uint8_t I2C_SendByte(uint8_t byte)
{
    uint8_t i;
    for (i = 0; i < 8; i++)
    {
        if (byte & 0x80) SDA_H(); else SDA_L();
        byte <<= 1;
        I2C_Delay(); SCL_H(); I2C_Delay(); SCL_L(); I2C_Delay();
    }
    SDA_H(); I2C_Delay(); SCL_H(); I2C_Delay();
    uint8_t ack = SDA_READ();
    SCL_L(); I2C_Delay();
    return ack;
}

static uint8_t I2C_ReadByte(uint8_t ack)
{
    uint8_t i, byte = 0;
    SDA_H();
    for (i = 0; i < 8; i++)
    {
        byte <<= 1;
        SCL_H(); I2C_Delay();
        if (SDA_READ()) byte |= 1;
        SCL_L(); I2C_Delay();
    }
    if (ack) SDA_H(); else SDA_L();  /* ACK(L)=继续, NACK(H)=停止 */
    I2C_Delay(); SCL_H(); I2C_Delay(); SCL_L(); I2C_Delay();
    SDA_H();
    return byte;
}

/* ========== I2C 读写寄存器 ========== */

static void I2C_WriteReg(uint8_t reg, uint8_t data)
{
    I2C_Start(); I2C_SendByte(MPU6050_ADDR); I2C_SendByte(reg); I2C_SendByte(data); I2C_Stop();
}

static uint8_t I2C_ReadRegs(uint8_t reg, uint8_t *buf, uint8_t len)
{
    uint8_t i;
    I2C_Start(); I2C_SendByte(MPU6050_ADDR); I2C_SendByte(reg); I2C_Stop();
    I2C_Delay(); I2C_Delay();   /* 让总线充分释放 */
    I2C_Start(); I2C_SendByte(MPU6050_ADDR | 1);  /* 读模式 */
    for (i = 0; i < len; i++) buf[i] = I2C_ReadByte(i == len - 1);  /* 最后一字节NACK */
    I2C_Stop();
    return 0;
}

/* ========== DWT 定时器 ========== */
static uint32_t GetTick_ms(void)
{
    static uint8_t init = 0;
    if (!init) { CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; DWT->CYCCNT = 0; DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk; init = 1; }
    return DWT->CYCCNT / (SystemCoreClock / 1000);
}

/* ========== MPU6050 函数 ========== */

uint8_t MPU6050_Init(void)
{
    uint8_t check;

    MPU_I2C_Init();

    /* 读 WHO_AM_I */
    I2C_ReadRegs(WHO_AM_I_REG, &check, 1);
    if (check != 0x68) return 1;

    I2C_WriteReg(PWR_MGMT_1_REG,   0x00);   /* 唤醒 */
    I2C_WriteReg(SMPLRT_DIV_REG,   0x07);   /* 采样率 */
    I2C_WriteReg(ACCEL_CONFIG_REG, 0x00);   /* ±2g */
    I2C_WriteReg(GYRO_CONFIG_REG,  0x00);   /* ±250°/s */
    return 0;
}

void MPU6050_Read_Accel(MPU6050_t *d)
{
    uint8_t buf[6];
    I2C_ReadRegs(ACCEL_XOUT_H_REG, buf, 6);
    d->Accel_X_RAW = (int16_t)(buf[0] << 8 | buf[1]);
    d->Accel_Y_RAW = (int16_t)(buf[2] << 8 | buf[3]);
    d->Accel_Z_RAW = (int16_t)(buf[4] << 8 | buf[5]);
    d->Ax = d->Accel_X_RAW / 16384.0;
    d->Ay = d->Accel_Y_RAW / 16384.0;
    d->Az = d->Accel_Z_RAW / Accel_Z_corrector;
}

void MPU6050_Read_Gyro(MPU6050_t *d)
{
    uint8_t buf[6];
    I2C_ReadRegs(GYRO_XOUT_H_REG, buf, 6);
    d->Gyro_X_RAW = (int16_t)(buf[0] << 8 | buf[1]);
    d->Gyro_Y_RAW = (int16_t)(buf[2] << 8 | buf[3]);
    d->Gyro_Z_RAW = (int16_t)(buf[4] << 8 | buf[5]);
    d->Gx = d->Gyro_X_RAW / 131.0;
    d->Gy = d->Gyro_Y_RAW / 131.0;
    d->Gz = d->Gyro_Z_RAW / 131.0;
}

void MPU6050_Read_Temp(MPU6050_t *d)
{
    uint8_t buf[2]; int16_t temp;
    I2C_ReadRegs(TEMP_OUT_H_REG, buf, 2);
    temp = (int16_t)(buf[0] << 8 | buf[1]);
    d->Temperature = (float)(temp / 340.0 + 36.53);
}

void MPU6050_Read_All(MPU6050_t *d)
{
    uint8_t buf[14]; int16_t temp;
    I2C_ReadRegs(ACCEL_XOUT_H_REG, buf, 14);

    d->Accel_X_RAW = (int16_t)(buf[0] << 8 | buf[1]);
    d->Accel_Y_RAW = (int16_t)(buf[2] << 8 | buf[3]);
    d->Accel_Z_RAW = (int16_t)(buf[4] << 8 | buf[5]);
    temp           = (int16_t)(buf[6] << 8 | buf[7]);
    d->Gyro_X_RAW  = (int16_t)(buf[8] << 8 | buf[9]);
    d->Gyro_Y_RAW  = (int16_t)(buf[10] << 8 | buf[11]);
    d->Gyro_Z_RAW  = (int16_t)(buf[12] << 8 | buf[13]);

    d->Ax = d->Accel_X_RAW / 16384.0;
    d->Ay = d->Accel_Y_RAW / 16384.0;
    d->Az = d->Accel_Z_RAW / Accel_Z_corrector;
    d->Temperature = (float)(temp / 340.0 + 36.53);
    d->Gx = d->Gyro_X_RAW / 131.0;
    d->Gy = d->Gyro_Y_RAW / 131.0;
    d->Gz = d->Gyro_Z_RAW / 131.0;

    /* 卡尔曼滤波 */
    double dt = (double)(GetTick_ms() - timer) / 1000.0;
    timer = GetTick_ms();

    double roll, roll_sqrt = sqrt((double)d->Accel_X_RAW * d->Accel_X_RAW + (double)d->Accel_Z_RAW * d->Accel_Z_RAW);
    if (roll_sqrt != 0.0) roll = atan((double)d->Accel_Y_RAW / roll_sqrt) * RAD_TO_DEG;
    else roll = 0.0;

    double pitch = atan2(-d->Accel_X_RAW, d->Accel_Z_RAW) * RAD_TO_DEG;

    if ((pitch < -90 && d->KalmanAngleY > 90) || (pitch > 90 && d->KalmanAngleY < -90))
    {
        KalmanY.angle = pitch;
        d->KalmanAngleY = pitch;
    }
    else
        d->KalmanAngleY = Kalman_getAngle(&KalmanY, pitch, d->Gy, dt);

    if (fabs(d->KalmanAngleY) > 90) d->Gx = -d->Gx;
    d->KalmanAngleX = Kalman_getAngle(&KalmanX, roll, d->Gx, dt);
}

double Kalman_getAngle(Kalman_t *K, double newAngle, double newRate, double dt)
{
    double rate = newRate - K->bias;
    K->angle += dt * rate;
    K->P[0][0] += dt * (dt * K->P[1][1] - K->P[0][1] - K->P[1][0] + K->Q_angle);
    K->P[0][1] -= dt * K->P[1][1];
    K->P[1][0] -= dt * K->P[1][1];
    K->P[1][1] += K->Q_bias * dt;
    double S = K->P[0][0] + K->R_measure;
    double K0 = K->P[0][0] / S, K1 = K->P[1][0] / S;
    double y = newAngle - K->angle;
    K->angle += K0 * y; K->bias += K1 * y;
    double P00 = K->P[0][0], P01 = K->P[0][1];
    K->P[0][0] -= K0 * P00; K->P[0][1] -= K0 * P01;
    K->P[1][0] -= K1 * P00; K->P[1][1] -= K1 * P01;
    return K->angle;
}
