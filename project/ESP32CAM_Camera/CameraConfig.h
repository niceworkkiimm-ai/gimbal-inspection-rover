#include "secrets.h"
#pragma once
/*
 * 摄像头固件 V1.3 / 2026-10-25
 * 本次：相对V1.2画面旋转180度（同时切换上下翻转与左右镜像）。
 * 上限10帧/秒、分辨率、完整接线及控制链路不变。
 * 实物：ESP-32S / ESP32-CAM + ESP32-CAM-MB(CH340C)，不是 ESP32-S3。
 * 依据：实物照片、烧录器识别为 ESP32、Arduino-ESP32 3.3.11 官方
 * CameraWebServer/camera_pins.h 的 CAMERA_MODEL_AI_THINKER 映射。
 *
 * 接线（正常使用与烧录均不改原STM32/基座/手持端接线）：
 * 1. 摄像头排线 -> 本板插座，断电插拔并锁紧。
 * 2. 摄像头板 -> ESP32-CAM-MB底板，对齐排针标识，不反插/错位。
 * 3. 底板Micro-USB -> 电脑USB数据口（烧录）或稳定5V USB电源（运行）。
 * 4. 脱离底板装车：稳压5V -> 明确标注5V的针脚；电源GND -> GND。
 *    不使用旁边有3V3/5V选择焊盘的VCC脚，不把5V接3V3。
 *    建议5V电源额定至少1A留余量；不从USB-TTL弱3.3V输出给摄像头供电。
 *    USB供电与外部5V不要同时接；不接电机Vin/电池原电压。
 * 5. 使用底板USB时，不再接外置TTL。备用TTL信号仅允许3.3V逻辑：
 *    TTL TX -> U0R/GPIO3；TTL RX <- U0T/GPIO1；TTL GND -> GND。
 * 6. 下载：按住底板IO0，按一下RST，进入下载后松开IO0，再Upload；
 *    正常运行：IO0松开（GPIO0不接地），按RST。RST不能长期接地。
 *    GPIO0运行时是摄像头XCLK，不可保持下载短接；GPIO2不外接高电平。
 * 7. 与STM32/基座ESP/手持端不接TX/RX等信号，仅WiFi通信。
 *    独立供电且无信号线无需额外共地；使用TTL时必须与TTL共地。
 *
 * Arduino IDE：esp32 by Espressif Systems 3.3.11
 * Board=AI Thinker ESP32-CAM；CPU=240MHz(WiFi/BT)；
 * Flash Mode=DIO；Flash Frequency=40MHz；Partition Scheme=Huge APP。
 * 此板定义默认4MB Flash并启用PSRAM，无独立PSRAM/Flash Size菜单；
 * 不用S3的16MB/OPI PSRAM/USB CDC选项。容量需实物日志确认，不能从照片保证。
 * 本版不使用TF卡，GPIO4闪光灯保持关闭。画面方向装车后实测。
 * 只做摄像头服务，不发送车辆控制命令。编译通过不代表实物出图已验证。
 */
#if !defined(CONFIG_IDF_TARGET_ESP32)
#error "Select AI Thinker ESP32-CAM, NOT ESP32S3 Dev Module or ESP8266."
#endif
#if !defined(BOARD_HAS_PSRAM)
#error "This firmware requires PSRAM. Select AI Thinker ESP32-CAM."
#endif

static const char CAMERA_VERSION[] = "1.3";
// 网页协议保持1.0，兼容原查看器；V8.5两页支持10帧。此处不是固件版本。
static const char CAMERA_API_VERSION[] = "1.0";
static const char CAMERA_WIFI_SSID[] = "GimbalCtrl";
static const char CAMERA_WIFI_PASSWORD[] = GIMBAL_WIFI_PASSWORD;
static const char CAMERA_HOSTNAME[] = "rover-camera";

// 官方 CAMERA_MODEL_AI_THINKER；D0..D7对应Y2..Y9，均为板内连线。
static const int CAM_PWDN = 32, CAM_RESET = -1;
static const int CAM_XCLK = 0, CAM_SDA = 26, CAM_SCL = 27;
static const int CAM_D0 = 5, CAM_D1 = 18, CAM_D2 = 19, CAM_D3 = 21;
static const int CAM_D4 = 36, CAM_D5 = 39, CAM_D6 = 34, CAM_D7 = 35;
static const int CAM_VSYNC = 25, CAM_HREF = 23, CAM_PCLK = 22;
static const int CAM_FLASH_LED = 4;

// V1.2为VFLIP=1、HMIRROR=0；两项同时取反，使当前画面整体旋转180度。
static const int CAMERA_VFLIP = 0;
static const int CAMERA_HMIRROR = 1;
static const int CAMERA_JPEG_QUALITY = 18;
static const unsigned CAMERA_MIN_INTERVAL_MS = 100; // 全局最多10帧/秒，实际帧率受WiFi限制。
static const unsigned CAMERA_MAX_FPS = 1000 / CAMERA_MIN_INTERVAL_MS;
static const unsigned CAMERA_VIEWER_LEASE_MS = 2500; // 单查看者。
static const unsigned CAMERA_MAX_FRAME_AGE_MS = 500;
