#include "secrets.h"
#pragma once
/*
 * 摄像头固件 V1.0 / 项目网页 V8.3 / 2026-09-13
 * 依据：桌面 ESP32-S3CAM开发板/产品说明书.doc 与商家 ESP32S3CAM 例程。
 * 仅适用资料中的 ESP32-S3 + 16MB Flash + 8MB PSRAM 摄像头板。
 * 商家选择 CAMERA_MODEL_ESP32S3_EYE；不是普通 AI Thinker ESP32-CAM 引脚。
 *
 * 接线：摄像头排线接本板摄像头座，断电插拔并锁紧。
 * 供电：手册写使用5V充电器，但图中小板无USB，VCC排针额定值未明确。
 * 若配有厂家供电/烧录底板，通过该底板已确认的USB输入供电。
 * 没有底板时先核对实物/厂家VCC额定值，不猜电压、不从STM32 GPIO取电。
 * 本版不接 STM32 UART/I2C/DCMI，不接手持端，不占用主控新引脚。
 * UART烧录信号（仅3.3V TTL）：TX->板RX/GPIO44；RX<-板TX/GPIO43；GND共地。
 * GPIO0->GND后复位进入下载，完成后断开GPIO0-GND再复位；RST不长期接地。
 * 与ESP-01S不同，不要求把GPIO2拉高；本版USB CDC On Boot选择Disabled。
 * 烧录：ESP32S3 Dev Module，Flash Size=16MB，PSRAM=OPI PSRAM。
 * 已通过Arduino-ESP32 3.3.0编译，Flash Mode=DIO 80MHz；实物供电/图传尚待验证。
 * 使用本文件集中修改热点、安装方向和画面参数。不要修改电机程序。
 */
#if !defined(CONFIG_IDF_TARGET_ESP32S3)
#error "Select ESP32S3 Dev Module, not ESP32/ESP8266."
#endif

static const char CAMERA_VERSION[] = "1.0";
static const char CAMERA_WIFI_SSID[] = "GimbalCtrl";
static const char CAMERA_WIFI_PASSWORD[] = GIMBAL_WIFI_PASSWORD;
static const char CAMERA_HOSTNAME[] = "rover-camera";

// 商家 camera_pins.h 的 CAMERA_MODEL_ESP32S3_EYE 映射。
static const int CAM_PWDN = -1, CAM_RESET = -1;
static const int CAM_XCLK = 15, CAM_SDA = 4, CAM_SCL = 5;
static const int CAM_D0 = 11, CAM_D1 = 9, CAM_D2 = 8, CAM_D3 = 10;
static const int CAM_D4 = 12, CAM_D5 = 18, CAM_D6 = 17, CAM_D7 = 16;
static const int CAM_VSYNC = 6, CAM_HREF = 7, CAM_PCLK = 13;

static const int CAMERA_VFLIP = 1;  // 与商家 S3 例程一致；装车后实测上下方向。
static const int CAMERA_HMIRROR = 0; // 不默认镜像，装车后核对左右。
static const int CAMERA_JPEG_QUALITY = 18; // 数字越大压缩越强。
static const unsigned CAMERA_MIN_INTERVAL_MS = 200; // 全局最多约5帧/秒。
static const unsigned CAMERA_VIEWER_LEASE_MS = 2500; // 单查看者，停止后自动释放。
static const unsigned CAMERA_MAX_FRAME_AGE_MS = 500;
