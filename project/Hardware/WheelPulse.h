/* One signed count per complete A high pulse. The B level across its two
 * edges distinguishes shaft travel from a contact bouncing at a single phase.
 * The wire protocol and calibrated counts per revolution stay at 1x. */
#ifndef WHEEL_PULSE_H
#define WHEEL_PULSE_H
#include "HandheldProtocol.h"

#define ENCODER_MIN_HIGH_US 80U
#define ENCODER_REJECT_WINDOW_MS 40U
#define ENCODER_REJECT_LIMIT 4U
#define ENCODER_NOISE_HOLD_MS 120U

typedef struct {
    uint32_t rise_cycles, rise_ms, window_ms, last_noise_ms;
    uint8_t have_rise, rise_b, rejects, noisy;
} WheelPulse;

HH_INLINE void WheelPulse_Reject(volatile WheelPulse *p, uint32_t ms)
{
    if (!p->rejects || ms - p->window_ms >= ENCODER_REJECT_WINDOW_MS) {
        p->window_ms = ms;
        p->rejects = 0;
    }
    if (p->rejects < ENCODER_REJECT_LIMIT) ++p->rejects;
    if (p->rejects == ENCODER_REJECT_LIMIT) {
        p->noisy = 1;
        p->last_noise_ms = ms;
    }
}

HH_INLINE int8_t WheelPulse_Edge(volatile WheelPulse *p, uint8_t a, uint8_t b,
                                    uint32_t cycles, uint32_t ms, uint32_t min_cycles)
{
    if (a) {
        if (p->have_rise) WheelPulse_Reject(p, ms);
        p->have_rise = 1;
        p->rise_b = b;
        p->rise_cycles = cycles;
        p->rise_ms = ms;
        return 0;
    }
    if (!p->have_rise) return 0; /* Includes initialization with A already high. */
    p->have_rise = 0;
    if (p->rise_b == b ||
        (ms - p->rise_ms <= 1U && cycles - p->rise_cycles < min_cycles)) {
        WheelPulse_Reject(p, ms);
        return 0;
    }
    return p->rise_b ? 1 : -1;
}

HH_INLINE uint8_t WheelPulse_Noisy(volatile WheelPulse *p, uint32_t ms)
{
    if (ms - p->last_noise_ms > ENCODER_NOISE_HOLD_MS) p->noisy = 0;
    return p->noisy;
}
#endif
