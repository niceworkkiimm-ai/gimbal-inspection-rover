#ifndef CAR_LOGIC_H
#define CAR_LOGIC_H
#include <stdint.h>
typedef enum {
    CAR_DISABLED=0, CAR_INACTIVE=1, CAR_NEUTRAL=2, CAR_READY=3,
    CAR_MOVING=4, CAR_LINK_LOST=5, CAR_FAULT=6
} CarStateCode;
/* Local TFT detail only; the existing vehicle telemetry state codes stay 0..6. */
typedef enum {
    CAR_HOLD_NONE=0, CAR_HOLD_CENTER, CAR_HOLD_INPUT, CAR_HOLD_TILT,
    CAR_HOLD_RECOVER
} CarHoldReason;
typedef struct {
    int16_t pwm[4]; /* 1 FL, 2 FR, 3 RL, 4 RR, logical wheel-forward signs. */
    int8_t previous_sign[4];
    uint32_t zero_since[4], last_step, neutral_since, last_packet;
    uint32_t pause_since, recovery_since;
    uint8_t state, armed, neutral_count, have_packet, hold_reason;
    uint8_t paused, recovery_count;
} CarLogic;
void CarLogic_Init(CarLogic *s, uint32_t now);
void CarLogic_Stop(CarLogic *s, uint32_t now, uint8_t state);
void CarLogic_Mix(int16_t forward, int16_t left, int16_t out[4]);
void CarLogic_Step(CarLogic *s, uint32_t now, uint8_t active, uint8_t enabled,
                   uint8_t safe, uint8_t fresh, uint8_t ready, uint32_t packet,
                   int16_t roll, int16_t pitch);
#endif
