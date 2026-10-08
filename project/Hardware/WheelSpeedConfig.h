/* V8.9 / 2026-09-15: route startup settings removed; manual settings unchanged.
 * No new pins. Default ENABLE=0 monitors RPM without modifying manual PWM.
 * Setting ENABLE=1 selects an optional PI controller; it requires bench validation.
 */
#ifndef WHEEL_SPEED_CONFIG_H
#define WHEEL_SPEED_CONFIG_H
#ifndef WHEEL_SPEED_ENABLE
#define WHEEL_SPEED_ENABLE 0
#endif
#define WHEEL_SPEED_CPR 374U /* A rising only: 11 PPR x 34; verify by turning a wheel. */
#define WHEEL_ENCODER_SIGNS {1,-1,1,-1}
#define WHEEL_SPEED_MAX_RPM 30.0f /* Initial low-speed request cap, NOT guaranteed actual speed. */
#define WHEEL_SPEED_PERIOD_MS 40U
#define WHEEL_SPEED_MAX_GAP_MS 120U
#define WHEEL_SPEED_WINDOW 3U
#define WHEEL_SPEED_SANITY_RPM 2000U /* Reject impossible count jumps, not a rated motor speed. */
/* Starting values for supervised bench tuning, not measured motor parameters.
 * Output: permille duty. Error: RPM. Integral uses actual elapsed seconds.
 */
#define WHEEL_SPEED_FF 4.0f
#define WHEEL_SPEED_KP 1.5f
#define WHEEL_SPEED_KI 1.0f
#define WHEEL_SPEED_INTEGRAL_LIMIT 150.0f
#define WHEEL_SPEED_OUTPUT_SLEW 800.0f /* permille / second */
#define WHEEL_SPEED_START_GRACE_MS 350U
#define WHEEL_SPEED_REVERSE_GRACE_MS 350U
#define WHEEL_SPEED_STALL_MS 800U
#define WHEEL_SPEED_WRONG_DIRECTION_MS 240U
#if WHEEL_SPEED_ENABLE != 0 && WHEEL_SPEED_ENABLE != 1
#error WHEEL_SPEED_ENABLE must be 0 (monitor) or 1 (closed-loop).
#endif
#if WHEEL_SPEED_CPR < 1 || WHEEL_SPEED_CPR > 1000000
#error Invalid 1x encoder counts per revolution.
#endif
#if WHEEL_SPEED_PERIOD_MS < 10 || WHEEL_SPEED_MAX_GAP_MS < WHEEL_SPEED_PERIOD_MS
#error Invalid wheel feedback timing.
#endif
#endif
