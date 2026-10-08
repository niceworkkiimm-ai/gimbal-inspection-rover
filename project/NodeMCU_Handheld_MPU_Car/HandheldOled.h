/* Handheld V1.6 (2026-09-20): SSD1306 status + joystick diagnostics.
 * SDA=D2/GPIO4, SCL=D1/GPIO5, 3.3V logic. Shares Wire with MPU6050/ADS1115.
 * No display()/whole-frame blocking loop and no extra Arduino library needed.
 * V1.6: adds an MPU state code + hint text line, and short first retries
 * (250/500/750 ms) before falling back to the old 5 s probe interval, so a
 * single bus hiccup no longer blacks the screen for five seconds.
 */
#ifndef HANDHELD_OLED_H
#define HANDHELD_OLED_H
#include <stdint.h>
#include "HandheldJoystickDiagnostics.h"

struct HandheldOledStatus {
  uint8_t mode = 0; // Local requested mode, NOT an acknowledgement from STM32.
  bool connected = false;
  bool mpuReady = false;
  // V1.6: MPU 初始化/校准状态码，仅用于触发重绘；提示文字见 mpuHintText。
  uint8_t mpuState = 0;
  const char *mpuHintText = nullptr;
  bool anglesValid = false;
  bool controlReady = false;
  bool buttonHeld = false;
  bool joystickReady = false;
  int16_t joystickX = 0, joystickY = 0; // Stick displacement %, not wrist angles.
  JoystickDiagnostic joystickDiagnostic = JoystickDiagnostic::WaitingSample;
  JoystickDiagnostic joystickLastError = JoystickDiagnostic::Ready;
  bool joystickHasRaw = false, joystickFresh = false;
  int16_t joystickRawX = 0, joystickRawY = 0;
  uint8_t joystickCalibration = 0, joystickErrorStage = 0, joystickWireResult = 0;
  uint32_t joystickErrors = 0;
  int16_t rssi = -100;
  float roll = 0;
  float pitch = 0;
};

class HandheldOled {
public:
  void begin(uint8_t sda, uint8_t scl, uint32_t nowMs);
  void setStatus(const HandheldOledStatus &status);
  // At most ONE I2C transaction, only with >=6 ms until next control sample.
  void service(uint32_t nextControlUs);
  bool present() const { return phase_ == Ready; }
  uint8_t address() const { return address_; }
  uint32_t errors() const { return errors_; }
  uint32_t maxTransferUs() const { return maxTransferUs_; }

private:
  enum Phase { ProbePrimary, ProbeSecondary, Initialize, Ready };
  Phase phase_ = ProbePrimary;
  HandheldOledStatus status_;
  uint8_t sda_ = 4, scl_ = 5, address_ = 0x3c, initStep_ = 0;
  uint8_t buffer_[1024] = {}, sent_[1024] = {};
  bool known_[64] = {};
  uint8_t cursor_ = 0;
  int16_t pendingBlock_ = -1;
  bool renderPending_ = true, displayOn_ = false;
  uint32_t nextProbeMs_ = 0, lastRenderMs_ = 0;
  uint32_t errors_ = 0, maxTransferUs_ = 0;
  uint8_t consecutiveErrors_ = 0;
  bool transfer(uint8_t control, const uint8_t *data, uint8_t length);
  bool probe();
  void failed();
  void render();
  void renderJoystick();
  void pixel(int x, int y);
  void ascii(int x, int y, const char *text);
  void text(int x, int y, const char *text, uint8_t size);
};
#endif
