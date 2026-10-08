#ifndef HANDHELD_INPUT_CONFIG_H
#define HANDHELD_INPUT_CONFIG_H
#include <stdint.h>
/* 手持V1.5 / 项目V8.8 / 2026-09-15；仅反转摇杆左右，前后/体感/保护参数不变。
 * 普通双电位器摇杆、ADS1115均用3.3V；模拟/数字线不得输入5V。
 * ADS VDD->3V3, GND->GND, SDA->D2(GPIO4), SCL->D1(GPIO5), ADDR->GND(0x48)。
 * 摇杆VCC->3V3, GND->GND, VRx/X->ADS A0, VRy/Y->ADS A1, SW->D5(GPIO14)。
 * ADS的A0/A1不是NodeMCU的A0；A2/A3/ALERT不接。旧独立按键拆除。
 * SW按下接地，使用内部上拉；若模块SW并非3.3V低有效输出，先核对模块。
 * MPU/OLED继续并接D1/D2，不拔原线；所有I2C上拉只到3.3V。
 * 初次开机松开摇杆稳定约1秒，中心只在合理范围内校准，不追踪移动中的中点。
 * 用户已确认摇杆改接3.3V后就绪；按左右反向反馈将kXSign改为-1，kYSign仍为1。
 * 重新烧录后架空核对方向；仅摇杆X轴反转，不影响体感云台/体感小车。
 * 保留STM32车控8度死区/30度满量程/30% PWM，不将ADC原值直接当电机占空比。
 */
namespace HandheldInputConfig {
constexpr uint8_t kAdsAddress = 0x48;
constexpr int kXSign = -1;
constexpr int kYSign = 1;
constexpr int32_t kAdcFull3V3 = 26400; // PGA +/-4.096V: 125uV/LSB, 3.3V=26400.
constexpr int32_t kExpectedCenter = 13200;
constexpr int32_t kCenterWindow = 2000; // 1.65V +/-0.25V; not proof of physically released stick.
constexpr int32_t kCalibrationJitter = 160; // 20mV peak-to-peak over the center hold.
constexpr uint32_t kCalibrationHoldMs = 400;
constexpr unsigned kDeadzonePercent = 8;
constexpr uint32_t kSampleFreshMs = 100;
constexpr uint32_t kNeutralHoldMs = 100; // STM32 adds its own 400ms/10-packet check.
constexpr int16_t kCarPitchZeroTenths = -20; // Matches existing CarConfig.h.
constexpr int16_t kCarRollZeroTenths = 0;
constexpr int16_t kCarDeadzoneTenths = 80;
constexpr int16_t kCarFullTenths = 300;
static_assert(kXSign == 1 || kXSign == -1, "X direction must be +/-1");
static_assert(kYSign == 1 || kYSign == -1, "Y direction must be +/-1");
}
#endif
