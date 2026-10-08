#ifndef WHEEL_SPEED_H
#define WHEEL_SPEED_H
#include <stdint.h>
#include "WheelSpeedConfig.h"
enum { WHEEL_FAULT_TIMING=1, WHEEL_FAULT_COUNTS=2,
       WHEEL_FAULT_DIRECTION=4, WHEEL_FAULT_NO_FEEDBACK=8 };
typedef struct {
    uint32_t last_ms, count[4], sample_dt, window_ms[WHEEL_SPEED_WINDOW];
    int32_t window_ticks[WHEEL_SPEED_WINDOW][4];
    uint32_t started_ms[4], moved_ms[4], wrong_ms[4], zero_ms[4];
    float rpm[4], target[4], integral[4], effort[4];
    int16_t output[4];
    int8_t previous_sign[4];
    uint8_t started[4], wrong[4];
    uint8_t initialized, valid, head, size, enabled, fault, fault_wheels, limited;
} WheelSpeed;
void WheelSpeed_Init(WheelSpeed *s);
void WheelSpeed_ClearFault(WheelSpeed *s);
/* Call from foreground with interrupts enabled. No allocation, delay or I/O.
 * demand is the original slew-limited manual input, not measured wheel speed.
 * allowed=0 zeros outputs/integrators immediately but keeps a fault latched.
 */
void WheelSpeed_Step(WheelSpeed *s,uint32_t now,const uint32_t counts[4],
                     const int16_t demand[4],uint8_t enabled,uint8_t allowed);
#endif
