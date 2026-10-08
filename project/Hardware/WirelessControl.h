#ifndef __WIRELESS_CONTROL_H
#define __WIRELESS_CONTROL_H

#include "stm32f4xx.h"
#include "EncoderProtocol.h"
#include "WheelSpeedProtocol.h"

typedef struct
{
    uint32_t sequence;
    uint32_t packet_count;
    float roll;
    float pitch;
    uint32_t arrival_cycles;
    uint8_t protocol, ready, requested_mode, mode_epoch;
} WirelessControlData_t;

typedef enum
{
    WIRELESS_MODE_HANDHELD = 0,
    WIRELESS_MODE_SCAN_HORIZONTAL,
    WIRELESS_MODE_SCAN_VERTICAL,
    WIRELESS_MODE_SCAN_ALL,
    WIRELESS_MODE_STOPPED,
    WIRELESS_MODE_CAR
} WirelessControlMode_t;

void WirelessControl_Init(void);
void WirelessControl_SendEncoder(const EncoderFrame *frame);
void WirelessControl_SendWheelSpeed(const WheelSpeedFrame *frame);
void WirelessControl_Poll(void);
uint8_t WirelessControl_GetLatest(WirelessControlData_t *data);
WirelessControlMode_t WirelessControl_GetMode(void);
uint8_t WirelessControl_ConsumeFault(void);
void WirelessControl_SendVehicle(uint32_t sequence, uint8_t mode, uint8_t state,
                                 const int16_t pwm[4]);
void WirelessControl_SendTelemetry(uint32_t sequence,
                                   int16_t roll_tenths,
                                   int16_t pitch_tenths,
                                   int16_t distance_tenths,
                                   uint8_t distance_valid,
                                   uint8_t link_ok);
void WirelessControl_SendGasTelemetry(uint32_t sequence,
                                      uint8_t channel,
                                      uint8_t online,
                                      uint8_t valid,
                                      uint32_t concentration_raw,
                                      uint8_t unit_code,
                                      uint8_t decimals,
                                      uint8_t status,
                                      uint8_t gas_type);

void WirelessControl_SendLog(void);
#endif
