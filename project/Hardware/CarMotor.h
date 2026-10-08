#ifndef CAR_MOTOR_H
#define CAR_MOTOR_H
#include "WirelessControl.h"
#include "CarLogic.h"
#include "WheelSpeedProtocol.h"
void CarMotor_Init(void);
uint32_t CarMotor_GetMillis(void);
void CarMotor_Service(WirelessControlMode_t mode,const WirelessControlData_t *data,uint8_t rx_fault);
void CarMotor_GetStatus(uint8_t *state,int16_t pwm[4]);
void CarMotor_GetWheelSpeed(WheelSpeedFrame *frame,uint32_t sequence);
uint8_t CarMotor_GetHoldReason(void);
void CarMotor_EmergencyStop(void);
#endif
