/* V6.4 transient recovery + driving direction fix (2026-09-08).
 * Motion enabled at the user's request; maximum duty is 30%, not measured RPM.
 * PD3 NC feedback is removed: no hardware emergency-stop input is fitted.
 * Keep wheels raised/frame secured; retain access to motor-power disconnect.
 * Electrical ratings and actual wheel directions still require physical checks.
 * PWM: M1 PC6/TIM8_CH1, M2 PC7/TIM8_CH2, M3 PE5/TIM9_CH1, M4 PE6/TIM9_CH2.
 * DIR: M1 PC4, M2 PC5, M3 PB12, M4 PB13. PD3: unused, leave unconnected.
 * Four PWM inputs each need external 10k pulldown at the driver (safe MCU reset).
 * Do not connect battery voltage to any MCU signal pin.
 */
#ifndef CAR_CONFIG_H
#define CAR_CONFIG_H
#ifndef CAR_RUNTIME_ENABLE
#define CAR_RUNTIME_ENABLE 1
#endif
/* This enables the trial, not a claim of completed hardware commissioning. */
#define CAR_MAX_PWM_PERMILLE 300 /* Duty limit, NOT measured speed or current limit. */
/* Same window for stopping and rearming; no stopped-but-cannot-rearm gap. */
#define CAR_DEADZONE_TENTHS 80
#define CAR_NEUTRAL_TENTHS 80
#define CAR_FULL_TILT_TENTHS 300
#define CAR_UNSAFE_TILT_TENTHS 550
#define CAR_ROLL_ZERO_TENTHS 0
#define CAR_PITCH_ZERO_TENTHS (-20)
/* V6.4: physical test showed forward tilt drove backward and left tilt drove
 * right, so both input-axis signs are flipped together. The V6.2 mirrored
 * right-pair polarity is kept; only the pitch/roll mapping signs change.
 */
#define CAR_FORWARD_SIGN 1
#define CAR_TURN_LEFT_SIGN (-1)
/* V6.2: mirrored left/right motors; right pair polarity corrected.
 * Physical wheel direction remains to be checked after flashing.
 */
#define CAR_M1_SIGN 1
#define CAR_M2_SIGN (-1)
#define CAR_M3_SIGN 1
#define CAR_M4_SIGN (-1)
#define CAR_PWM_HZ 20000U
#define CAR_CONTROL_PERIOD_MS 10U
#define CAR_MAIN_DEADLINE_MS 120U
#define CAR_LINK_TIMEOUT_MS 250U
#define CAR_NEUTRAL_HOLD_MS 400U
#define CAR_NEUTRAL_PACKETS 10U
/* A brief bad sample stops PWM, but need not demand another neutral gesture.
 * Both the interruption and 3-packet recovery must fit inside this window.
 * Real link expiry, mode changes, faults and sustained bad input still disarm.
 */
#define CAR_TRANSIENT_WINDOW_MS 300U
#define CAR_RECOVERY_HOLD_MS 40U
#define CAR_RECOVERY_PACKETS 3U
#define CAR_RECOVERY_TILT_TENTHS 500
#define CAR_REVERSE_PAUSE_MS 100U
#define CAR_SLEW_PER_SECOND 1000U
#if CAR_MAX_PWM_PERMILLE < 1 || CAR_MAX_PWM_PERMILLE > 300
#error Vehicle trial duty must remain within 1..300 permille.
#endif
#if CAR_NEUTRAL_TENTHS > CAR_DEADZONE_TENTHS || CAR_DEADZONE_TENTHS >= CAR_FULL_TILT_TENTHS
#error Neutral must stay inside the stop deadzone and below full tilt.
#endif
#if CAR_RECOVERY_HOLD_MS >= CAR_TRANSIENT_WINDOW_MS || CAR_RECOVERY_PACKETS < 2
#error Transient recovery needs multiple fresh samples inside its time window.
#endif
#if CAR_RECOVERY_TILT_TENTHS < CAR_FULL_TILT_TENTHS || CAR_RECOVERY_TILT_TENTHS >= CAR_UNSAFE_TILT_TENTHS
#error Recovery tilt must include normal control range and stay below the tilt trip.
#endif
#if (CAR_M1_SIGN != 1 && CAR_M1_SIGN != -1) || (CAR_M2_SIGN != 1 && CAR_M2_SIGN != -1) || (CAR_M3_SIGN != 1 && CAR_M3_SIGN != -1) || (CAR_M4_SIGN != 1 && CAR_M4_SIGN != -1)
#error Motor direction signs must be +1 or -1.
#endif
#endif
