#include "secrets.h"
/*
 * NodeMCU V3 CH340G 手持端 V1.6，项目接线说明 V8.8（2026-09-20）。
 * 兼容当前 STM32 V8.2 / 基座 ESP V4.8；本次仅需烧录手持 NodeMCU，接线不变。
 * V1.6：MPU6050 初始化/校准改为分步状态机，setup() 不再阻塞等待 MPU。
 * 旧版问题：一次性采集300次样本，且 I2C 每时钟位允许拉伸1500us；PCB 上总线
 * 一旦偏慢或接触不良，单次14字节读取最坏可累计上百毫秒，300次即几十秒，
 * 期间 OLED/按键/UDP 全部停摆；拔掉 MPU 跳过校准反而立刻正常。
 * 现在：每个20ms空档最多一次 I2C 事务；连续5次读取失败或超8s预算立即判失败；
 * 重试计时从失败结束时刻开始；MPU 持续拉低 SDA 时先做一次9脉冲总线恢复。
 * MPU 时钟拉伸上限1500us->250us，并记录每次事务耗时，慢事务按失败处理。
 * 屏幕第4行显示 MPU 配置中/校准中/无响应/配置失败/读取失败/校准失败/校准超时。
 * 串口新增 MPU_INIT_START / MPU_WHO_AM_I / MPU_FAIL 原因 / MPU_READY / I2C_BUS_RECOVER。
 * 唯一副作用：校准完成那一拍会多一次 MPU 读取，帧间隔出现一次约1.7ms的偏移，之后恢复20ms。
 * V1.5：仅将摇杆kXSign由1改为-1，修正左右反向；前后与两种体感模式不变。
 * 用户已确认摇杆VCC改接3.3V后正常就绪；下方全部接线及现有保护保持不变。
 * V1.4：摇杆页显示E00~E15具体原因、X/Y原始ADC、校准进度与上次采样错误。
 * 无新增I2C读写/扫描，采样与校准门槛、H1协议和回中/失联保护不变。
 * 接线复核不变：D1/D2共用总线，ADDR接GND，X/Y接ADS A0/A1，SW接D5。
 * 若SDA/SCL共用总线被持续拉低，OLED也无法更新；旧画面不能当成实时状态。
 * V1.3：摇杆短按循环 体感云台 -> 体感小车 -> 摇杆小车；长按0.8秒停车。
 * H1协议不改：本地摇杆模式3发送车模式1，切换epoch递增，让STM32停车并重新确认回中。
 * ADS1115非阻塞轮询，与MPU/OLED分时共用Wire；摇杆模式不读取/重校准MPU。
 * V1.2：新增 SSD1306 128x64 四针I2C OLED，0x3C/0x3D自动探测地址。
 * 屏幕显示本地云台/小车选择、停车请求、WiFi信号、R/P姿态及手持状态；
 * 模式/就绪仅代表手持请求和输入状态，不代表基座已接收或电机已执行。
 * 屏幕按16字节分块，在20ms控制周期的空隙刷新；不新增整屏等待延时。
 * 没有OLED时继续遥控；若共用SDA/SCL被短路，MPU也会受影响，请检查接线。
 * 2026-09-08：按键35ms稳定消抖后才影响ready；MPU连续3次失败才重新初始化。
 * 单次读取失败仍发无效帧，由STM32暂停输出并判断是否可短时自动恢复。
 * 板载 USB 负责烧录/5V供电，无需外接USB-TTL；3V3不能接5V。
 * 不用USB时：稳定5V正极->VIN，负极->GND；不要和USB并联供电。
 * EN/RST保持板载电路，不用外接；正常运行GPIO0/GPIO2不要接地。
 * MPU6050 VCC->3V3，GND->GND，SDA->D2(GPIO4)，SCL->D1(GPIO5)，AD0->GND。
 * MPU6050 INT/XDA/XCL不接；OLED VCC->3V3、GND->GND（屏幕需支持3.3V）。
 * OLED SDA->D2(GPIO4)、SCL->D1(GPIO5)，与MPU对应引脚并接，不拔MPU线。
 * I2C上拉只能到3.3V；不是SPI屏驱动，SH1106/其他分辨率须另外匹配。
 * ADS1115 VDD->3V3、GND->GND、SDA->D2、SCL->D1、ADDR->GND(0x48)。
 * 摇杆VCC->3V3、GND->GND、X/VRx->ADS A0、Y/VRy->ADS A1；不是NodeMCU A0。
 * 摇杆SW(按下接地)->D5(GPIO14)，内部上拉；旧独立按键拆除，SW不能输出5V。
 * ADS A2/A3/ALERT不接；所有模块共地、I2C上拉到3.3V。5V专用摇杆须先核对。
 * 短按松开切换三个模式；长按0.8秒软件停车，再短按回体感云台。
 * 上电手持水平静止、摇杆松开约1秒校准；切入摇杆小车先回中，再推动。
 * 摇杆中点/方向/死区集中在HandheldInputConfig.h，正负方向需架空实测。
 * 断网后回云台，不自动恢复小车模式。I2C错误/旧摇杆样本使ready=0并要求重新回中。
 * 指示灯：云台每秒短闪，小车请求常亮，停止半秒闪；传感器/网络异常快闪。
 * 灯表示请求模式，不代表STM32电机已解锁；实际状态看TFT/控制网页。
 */
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <math.h>
#include "HandheldProtocol.h"
#include "HandheldOled.h"
#include "HandheldJoystick.h"
#include "HandheldInputConfig.h"

namespace {
constexpr char kApName[] = "GimbalCtrl";
constexpr char kApPassword[] = GIMBAL_WIFI_PASSWORD;
const IPAddress kBaseIp(192, 168, 4, 1);
constexpr uint16_t kUdpPort = 4210;

constexpr uint8_t kMpuAddress = 0x68;
constexpr uint8_t kSdaPin = 4;
constexpr uint8_t kSclPin = 5;
constexpr uint8_t kButtonPin = 14;
constexpr uint32_t kSendPeriodUs = 20000;

// V1.6 MPU 分步初始化参数。
// MPU6050 正常不需要拉伸 SCL；250us/位已远大于 100kHz 标称的 10us/位。
// 旧值 1500us/位 会让"总线偏慢"时单次读取卡住上百毫秒，是几十秒无响应的主因之一。
constexpr uint32_t kMpuStretchLimitUs = 250;
constexpr uint32_t kMpuBaseStretchLimitUs = 1500;
// 事务耗时上限：正常 14 字节读取约 1.5~2ms，超过上限说明总线被异常拉伸。
constexpr uint32_t kMpuReadLimitBaseUs = 3000;
constexpr uint32_t kMpuReadLimitPerByteUs = 300;
constexpr uint16_t kMpuCalibrationSamples = 300;
constexpr uint16_t kMpuCalibrationMinValid = kMpuCalibrationSamples * 9 / 10; // 270
constexpr uint8_t kMpuMaxConsecutiveReadFailures = 5;
constexpr uint32_t kMpuSampleGapMs = 3;
constexpr uint32_t kMpuCalibrationBudgetMs = 8000;
constexpr uint32_t kMpuConfigSettleMs = 50;
constexpr uint32_t kMpuRetryMs = 2000;
// 一次空闲时隙至少留出这么多时间才做 MPU 事务，保证 OLED/摇杆优先。
constexpr uint32_t kMpuServiceMarginUs = 2500;

enum MpuPhase : uint8_t {
  MpuPhaseIdle = 0,      // 尚未开始，或等待重试
  MpuPhaseConfigure,     // 正在读写配置寄存器
  MpuPhaseCalibrate,     // 正在采集静止样本
  MpuPhaseReady,         // 就绪，允许体感控制
  MpuPhaseNoAnswer,      // WHO_AM_I 无应答或 ID 不符
  MpuPhaseConfigFail,    // 配置寄存器写入失败
  MpuPhaseReadFail,      // 连续读取失败
  MpuPhaseCalibFail,     // 静止判定失败或有效样本不足
  MpuPhaseCalibTimeout   // 校准超出时间预算
};

struct MpuAttempt {
  MpuPhase phase = MpuPhaseIdle;
  uint8_t step = 0;
  uint16_t samples = 0;
  uint16_t validSamples = 0;
  uint8_t consecutiveFailures = 0;
  int32_t sumX = 0;
  int32_t sumY = 0;
  bool stationary = true;
  uint32_t attemptStartMs = 0;
  uint32_t sampleDueMs = 0;
  uint32_t retryAtMs = 0;
};

struct KalmanState {
  float qAngle = 0.001f;
  float qBias = 0.003f;
  float rMeasure = 0.03f;
  float angle = 0.0f;
  float bias = 0.0f;
  float p00 = 0.0f;
  float p01 = 0.0f;
  float p10 = 0.0f;
  float p11 = 0.0f;
};

WiFiUDP udp;
HandheldOled oled;
HandheldJoystick joystick;
bool joystickArmed=false,joystickNeutralStarted=false;
uint32_t joystickNeutralSinceMs=0;
bool oledAnglesValid = false;
bool oledControlReady = false;
float oledRoll = 0, oledPitch = 0;
uint32_t oledSampleMs = 0;
KalmanState kalmanX;
KalmanState kalmanY;
float gyroOffsetX = 0.0f;
float gyroOffsetY = 0.0f;
uint32_t sequence = 0;
uint8_t requestedMode = 0; // Local: 0 tilt gimbal, 1 tilt car, 2 stop, 3 joystick car.
uint8_t modeEpoch = 0;
bool buttonHeld = false;
uint32_t healthySinceMs = 0;
bool healthyStarted = false;
uint32_t lastSampleUs = 0;
bool mpuReady = false;
uint8_t mpuReadFailures = 0;
bool filterInitialized = false;
MpuAttempt mpuAttempt;
uint32_t mpuSlowTransactions = 0;
uint8_t mpuBusRecoveries = 0;
int16_t lastAx = 0;
int16_t lastAy = 0;
int16_t lastAz = 0;
int16_t lastGx = 0;
int16_t lastGy = 0;
int16_t lastGz = 0;

// SDA/SCL 只要有一条被拉低，就说明总线被占用或卡死，先退出，绝不在这里等。
bool i2cLinesIdle() {
  return digitalRead(kSdaPin) != LOW && digitalRead(kSclPin) != LOW;
}

// V1.6：释放卡死的总线。从机在半个字节后复位时可能一直拉低 SDA，
// 标准做法是主机补 9 个 SCL 脉冲再发 STOP，让从机回到空闲态。
// 只在准备开始一次新事务前调用，最多每个初始化尝试一次。
bool recoverI2cBus() {
  if (i2cLinesIdle()) return true;
  if (mpuBusRecoveries >= 1) return false; // 每轮尝试只恢复一次，避免反复占用总线
  ++mpuBusRecoveries;
  Serial.println("I2C_BUS_RECOVER");
  pinMode(kSdaPin, INPUT_PULLUP);
  pinMode(kSclPin, OUTPUT);
  digitalWrite(kSclPin, HIGH);
  delayMicroseconds(5);
  for (uint8_t i = 0; i < 9 && digitalRead(kSdaPin) == LOW; ++i) {
    digitalWrite(kSclPin, LOW);
    delayMicroseconds(5);
    digitalWrite(kSclPin, HIGH);
    delayMicroseconds(5);
  }
  pinMode(kSdaPin, OUTPUT);
  digitalWrite(kSdaPin, LOW);
  delayMicroseconds(5);
  digitalWrite(kSclPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(kSdaPin, HIGH);
  delayMicroseconds(5);
  pinMode(kSdaPin, INPUT_PULLUP);
  pinMode(kSclPin, INPUT_PULLUP);
  Wire.begin(kSdaPin, kSclPin);
  Wire.setClock(100000);
  Wire.setClockStretchLimit(kMpuBaseStretchLimitUs);
  const bool idle = i2cLinesIdle();
  Serial.println(idle ? "I2C_BUS_RECOVERED" : "I2C_BUS_STILL_LOW");
  return idle;
}

bool writeRegister(uint8_t reg, uint8_t value) {
  if (!i2cLinesIdle()) return false;
  Wire.setClockStretchLimit(kMpuStretchLimitUs);
  Wire.beginTransmission(kMpuAddress);
  Wire.write(reg);
  Wire.write(value);
  const uint8_t result = Wire.endTransmission();
  Wire.setClockStretchLimit(kMpuBaseStretchLimitUs);
  return result == 0;
}

bool readRegisters(uint8_t reg, uint8_t *data, size_t length) {
  if (!i2cLinesIdle()) {
    ++mpuSlowTransactions;
    return false;
  }
  const uint32_t startUs = micros();
  bool ok = false;
  Wire.setClockStretchLimit(kMpuStretchLimitUs);
  Wire.beginTransmission(kMpuAddress);
  Wire.write(reg);
  if (Wire.endTransmission(false) == 0) {
    const size_t received = Wire.requestFrom(kMpuAddress, length, true);
    if (received == length) {
      for (size_t i = 0; i < length; ++i) {
        data[i] = static_cast<uint8_t>(Wire.read());
      }
      ok = true;
    }
  }
  while (Wire.available()) Wire.read(); // 清掉残留字节，避免污染下一次事务
  Wire.setClockStretchLimit(kMpuBaseStretchLimitUs);
  const uint32_t elapsedUs = micros() - startUs;
  if (ok && elapsedUs > kMpuReadLimitBaseUs + kMpuReadLimitPerByteUs * length) {
    ++mpuSlowTransactions; // 读到数据但耗时异常：仍按失败处理，避免拖慢主循环
    return false;
  }
  return ok;
}

bool readRaw(int16_t &ax, int16_t &ay, int16_t &az,
             int16_t &gx, int16_t &gy, int16_t &gz) {
  uint8_t data[14];
  if (!readRegisters(0x3B, data, sizeof(data))) {
    return false;
  }

  ax = static_cast<int16_t>((data[0] << 8) | data[1]);
  ay = static_cast<int16_t>((data[2] << 8) | data[3]);
  az = static_cast<int16_t>((data[4] << 8) | data[5]);
  gx = static_cast<int16_t>((data[8] << 8) | data[9]);
  gy = static_cast<int16_t>((data[10] << 8) | data[11]);
  gz = static_cast<int16_t>((data[12] << 8) | data[13]);
  return true;
}

// V1.6：分步初始化。每个调用只做一次 I2C 事务或一次采样，
// 中间回到 loop() 刷新 OLED、处理按键和 UDP，不再整段阻塞。
void mpuFailAttempt(MpuPhase phase, const char *reason) {
  mpuAttempt.phase = phase;
  // 关键修正：从"失败结束"开始计时，而不是从校准开始前开始计时。
  mpuAttempt.retryAtMs = millis() + kMpuRetryMs;
  Serial.printf("MPU_FAIL %s samples=%u valid=%u stationary=%u repeats=%u slow=%lu\n",
                reason, mpuAttempt.samples, mpuAttempt.validSamples,
                mpuAttempt.stationary ? 1U : 0U, mpuAttempt.consecutiveFailures,
                static_cast<unsigned long>(mpuSlowTransactions));
}

void mpuStartAttempt(uint32_t now) {
  mpuAttempt.phase = MpuPhaseConfigure;
  mpuAttempt.step = 0;
  mpuAttempt.samples = 0;
  mpuAttempt.validSamples = 0;
  mpuAttempt.consecutiveFailures = 0;
  mpuAttempt.sumX = 0;
  mpuAttempt.sumY = 0;
  mpuAttempt.stationary = true;
  mpuAttempt.attemptStartMs = now;
  mpuAttempt.sampleDueMs = now;
  mpuBusRecoveries = 0;
  recoverI2cBus();
  Serial.println("MPU_INIT_START");
}

void mpuFinishAttempt() {
  if (!mpuAttempt.stationary || mpuAttempt.validSamples < kMpuCalibrationMinValid) {
    mpuFailAttempt(MpuPhaseCalibFail, "CALIBRATION_REJECTED");
    return;
  }
  gyroOffsetX = static_cast<float>(mpuAttempt.sumX) / mpuAttempt.validSamples / 131.0f;
  gyroOffsetY = static_cast<float>(mpuAttempt.sumY) / mpuAttempt.validSamples / 131.0f;
  mpuReadFailures = 0;
  filterInitialized = false;
  lastSampleUs = micros();
  mpuReady = true;
  mpuAttempt.phase = MpuPhaseReady;
  mpuAttempt.retryAtMs = 0;
  Serial.printf("MPU_READY samples=%u valid=%u slow=%lu\n",
                mpuAttempt.samples, mpuAttempt.validSamples,
                static_cast<unsigned long>(mpuSlowTransactions));
}

void mpuServiceConfigure(uint32_t now) {
  switch (mpuAttempt.step) {
    case 0: {
      uint8_t identity = 0;
      if (!readRegisters(0x75, &identity, 1)) {
        mpuFailAttempt(MpuPhaseNoAnswer, "WHO_AM_I_NO_ANSWER");
        return;
      }
      if (identity != 0x68) {
        mpuAttempt.phase = MpuPhaseNoAnswer;
        mpuAttempt.retryAtMs = millis() + kMpuRetryMs;
        Serial.printf("MPU_WHO_AM_I=0x%02X expect=0x68\n", identity);
        return;
      }
      ++mpuAttempt.step;
      return;
    }
    case 1:
      if (!writeRegister(0x6B, 0x01)) { mpuFailAttempt(MpuPhaseConfigFail, "PWR_MGMT_1"); return; }
      ++mpuAttempt.step; return;
    case 2:
      if (!writeRegister(0x19, 0x04)) { mpuFailAttempt(MpuPhaseConfigFail, "SMPLRT_DIV"); return; }
      ++mpuAttempt.step; return;
    case 3:
      if (!writeRegister(0x1A, 0x03)) { mpuFailAttempt(MpuPhaseConfigFail, "CONFIG"); return; }
      ++mpuAttempt.step; return;
    case 4:
      if (!writeRegister(0x1C, 0x00)) { mpuFailAttempt(MpuPhaseConfigFail, "ACCEL_CONFIG"); return; }
      ++mpuAttempt.step; return;
    case 5:
      if (!writeRegister(0x1B, 0x00)) { mpuFailAttempt(MpuPhaseConfigFail, "GYRO_CONFIG"); return; }
      ++mpuAttempt.step; return;
    case 6: // 等 50ms 让内部寄存器稳定，然后开始静止采样
      mpuAttempt.sampleDueMs = now + kMpuConfigSettleMs;
      ++mpuAttempt.step;
      return;
    case 7:
      if (static_cast<int32_t>(now - mpuAttempt.sampleDueMs) < 0) return;
      mpuAttempt.phase = MpuPhaseCalibrate;
      mpuAttempt.attemptStartMs = now;
      mpuAttempt.sampleDueMs = now;
      mpuAttempt.stationary = true;
      Serial.println("MPU_CALIBRATION_KEEP_STILL");
      return;
    default:
      mpuFailAttempt(MpuPhaseConfigFail, "STEP_OVERFLOW");
      return;
  }
}

void mpuServiceCalibrate(uint32_t now) {
  if (static_cast<int32_t>(now - mpuAttempt.sampleDueMs) < 0) return;
  if (now - mpuAttempt.attemptStartMs >= kMpuCalibrationBudgetMs) {
    mpuFailAttempt(MpuPhaseCalibTimeout, "CALIBRATION_TIMEOUT");
    return;
  }
  mpuAttempt.sampleDueMs = now + kMpuSampleGapMs;

  int16_t ax, ay, az, gx, gy, gz;
  if (!readRaw(ax, ay, az, gx, gy, gz)) {
    if (mpuAttempt.consecutiveFailures < 255) ++mpuAttempt.consecutiveFailures;
    // 连续失败直接结束本轮，不再死等300次；之后按 kMpuRetryMs 重试。
    if (mpuAttempt.consecutiveFailures >= kMpuMaxConsecutiveReadFailures) {
      mpuFailAttempt(MpuPhaseReadFail, "CALIBRATION_READ_FAIL");
    }
    return;
  }
  mpuAttempt.consecutiveFailures = 0;
  ++mpuAttempt.samples;

  const float norm = sqrtf(float(ax)*ax + float(ay)*ay + float(az)*az)/16384.0f;
  if (norm < 0.85f || norm > 1.15f ||
      abs(gx) > 1200 || abs(gy) > 1200 || abs(gz) > 1200) {
    mpuAttempt.stationary = false;
  }
  mpuAttempt.sumX += gx;
  mpuAttempt.sumY += gy;
  ++mpuAttempt.validSamples;

  if (mpuAttempt.samples >= kMpuCalibrationSamples) mpuFinishAttempt();
}

// 只在 loop() 的空闲时隙调用，留足时间给 OLED/摇杆/按键。
void serviceMpuInit(uint32_t deadlineUs) {
  if (mpuReady) return;
  if (static_cast<int32_t>(deadlineUs - micros()) < static_cast<int32_t>(kMpuServiceMarginUs)) return;
  const uint32_t now = millis();
  if (mpuAttempt.phase == MpuPhaseConfigure) { mpuServiceConfigure(now); return; }
  if (mpuAttempt.phase == MpuPhaseCalibrate) { mpuServiceCalibrate(now); return; }
  if (static_cast<int32_t>(now - mpuAttempt.retryAtMs) < 0) return;
  mpuStartAttempt(now);
}

const char *mpuPhaseHint() {
  switch (mpuAttempt.phase) {
    case MpuPhaseConfigure: return "MPU 配置中";
    case MpuPhaseCalibrate: return "MPU 校准中";
    case MpuPhaseReady: return "MPU 就绪";
    case MpuPhaseNoAnswer: return "MPU 无响应";
    case MpuPhaseConfigFail: return "MPU 配置失败";
    case MpuPhaseReadFail: return "MPU 读取失败";
    case MpuPhaseCalibFail: return "MPU 校准失败";
    case MpuPhaseCalibTimeout: return "MPU 校准超时";
    default: return "MPU 未就绪";
  }
}

float updateKalman(KalmanState &state, float measuredAngle,
                   float measuredRate, float dt) {
  const float rate = measuredRate - state.bias;
  state.angle += dt * rate;

  state.p00 += dt * (dt * state.p11 - state.p01 - state.p10 + state.qAngle);
  state.p01 -= dt * state.p11;
  state.p10 -= dt * state.p11;
  state.p11 += state.qBias * dt;

  const float innovationVariance = state.p00 + state.rMeasure;
  const float gain0 = state.p00 / innovationVariance;
  const float gain1 = state.p10 / innovationVariance;
  const float innovation = measuredAngle - state.angle;

  state.angle += gain0 * innovation;
  state.bias += gain1 * innovation;

  const float oldP00 = state.p00;
  const float oldP01 = state.p01;
  state.p00 -= gain0 * oldP00;
  state.p01 -= gain0 * oldP01;
  state.p10 -= gain1 * oldP00;
  state.p11 -= gain1 * oldP01;
  return state.angle;
}

bool readFilteredAngles(float &roll, float &pitch) {
  int16_t ax, ay, az, gxRaw, gyRaw, gzRaw;
  if (!readRaw(ax, ay, az, gxRaw, gyRaw, gzRaw)) {
    return false;
  }

  lastAx = ax;
  lastAy = ay;
  lastAz = az;
  lastGx = gxRaw;
  lastGy = gyRaw;
  lastGz = gzRaw;

  const float rollDenominator = sqrtf(static_cast<float>(ax) * ax +
                                      static_cast<float>(az) * az);
  const float accelRoll = rollDenominator > 0.0f
                              ? atanf(static_cast<float>(ay) / rollDenominator) * 57.2957795f
                              : 0.0f;
  const float accelPitch = atan2f(-static_cast<float>(ax),
                                  static_cast<float>(az)) * 57.2957795f;

  float gyroX = static_cast<float>(gxRaw) / 131.0f - gyroOffsetX;
  const float gyroY = static_cast<float>(gyRaw) / 131.0f - gyroOffsetY;

  const uint32_t nowUs = micros();
  float dt = static_cast<float>(nowUs - lastSampleUs) / 1000000.0f;
  lastSampleUs = nowUs;
  if (dt < 0.001f || dt > 0.1f) {
    dt = 0.02f;
  }

  if (!filterInitialized) {
    kalmanX.angle = accelRoll;
    kalmanY.angle = accelPitch;
    filterInitialized = true;
  }

  if ((accelPitch < -90.0f && kalmanY.angle > 90.0f) ||
      (accelPitch > 90.0f && kalmanY.angle < -90.0f)) {
    kalmanY.angle = accelPitch;
    pitch = accelPitch;
  } else {
    pitch = updateKalman(kalmanY, accelPitch, gyroY, dt);
  }

  if (fabsf(pitch) > 90.0f) {
    gyroX = -gyroX;
  }
  roll = updateKalman(kalmanX, accelRoll, gyroX, dt);
  return isfinite(roll) && isfinite(pitch);
}

bool readControlAngles(float &roll,float &pitch) {
  if(readFilteredAngles(roll,pitch)) {
    mpuReadFailures=0;
    return true;
  }
  healthyStarted=false;
  if(mpuReadFailures<3)mpuReadFailures++;
  if(mpuReadFailures>=3) {
    // 交回分步状态机重新初始化，绝不在控制路径里做阻塞校准。
    mpuReady=false;
    mpuAttempt.phase=MpuPhaseReadFail;
    mpuAttempt.retryAtMs=millis()+kMpuRetryMs;
    mpuAttempt.consecutiveFailures=0;
    Serial.println("MPU_READ_ERROR_REINIT");
  } else Serial.println("MPU_SAMPLE_RETRY");
  return false;
}

void connectToBase() {
  static uint32_t lastAttempt = 0;
  static bool attempted = false;
  if (WiFi.status() == WL_CONNECTED) return;
  if (!attempted || millis()-lastAttempt >= 5000) {
    lastAttempt=millis();attempted=true;
    WiFi.begin(kApName,kApPassword);
    Serial.println("WIFI_CONNECTING");
  }
}

void updateButton() {
  static int lastRaw=HIGH, stable=HIGH;
  static uint32_t changedAt=0, pressedAt=0;
  static bool longHandled=false;
  const uint32_t now=millis();
  const int raw=digitalRead(kButtonPin);
  if(raw!=lastRaw){lastRaw=raw;changedAt=now;}
  if(now-changedAt>=35 && raw!=stable) {
    stable=raw;
    if(stable==LOW){pressedAt=now;longHandled=false;healthyStarted=false;
      joystickArmed=false;joystickNeutralStarted=false;oledControlReady=false;}
    else if(!longHandled) {
      requestedMode=requestedMode==0?1:requestedMode==1?3:0;
      modeEpoch++;
      healthyStarted=false;
      joystickArmed=false;joystickNeutralStarted=false;filterInitialized=false;
      oledAnglesValid=false;oledControlReady=false;
      Serial.println(requestedMode==1?"MODE,TILT_VEHICLE":requestedMode==3?"MODE,JOYSTICK_VEHICLE":"MODE,TILT_GIMBAL");
    }
  }
  if(stable==LOW&&!longHandled&&now-pressedAt>=800) {
    requestedMode=2;modeEpoch++;longHandled=true;healthyStarted=false;
    joystickArmed=false;joystickNeutralStarted=false;oledControlReady=false;
    Serial.println("MODE,SOFTWARE_STOP");
  }
  // Ignore sub-35ms contact noise; a confirmed press still invalidates motion.
  buttonHeld=(stable==LOW);
}

void sendFrame(bool valid,float roll,float pitch) {
  if(WiFi.status()!=WL_CONNECTED)return;
  HandheldFrame frame;
  frame.sequence=sequence++;frame.mode=requestedMode==3?1:requestedMode;frame.epoch=modeEpoch;frame.ready=valid?1:0;
  frame.roll=valid?(int16_t)lroundf(roll*10.0f):0;
  frame.pitch=valid?(int16_t)lroundf(pitch*10.0f):0;
  char message[64];
  int length=HH_Format(message,sizeof(message),&frame);
  if(length>0) {
    udp.beginPacket(kBaseIp,kUdpPort);
    udp.write(reinterpret_cast<const uint8_t *>(message),length);
    udp.endPacket();
  }
}

// Called only while waiting for the next control deadline, never during MPU I2C.
void updateOled(uint32_t nextControlUs) {
  static uint32_t lastRssiMs=0;
  static bool rssiKnown=false;
  static int16_t rssi=-100;
  const uint32_t now=millis();
  const bool connected=WiFi.status()==WL_CONNECTED;
  if(!connected)rssiKnown=false;
  else if(!rssiKnown || now-lastRssiMs>=1000) {
    rssi=static_cast<int16_t>(WiFi.RSSI());lastRssiMs=now;rssiKnown=true;
  }
  HandheldOledStatus state;
  state.mode=requestedMode;state.connected=connected;state.mpuReady=mpuReady;
  // V1.6：屏幕显示 MPU 具体状态，不必连接串口也能看出卡在哪一步。
  state.mpuState=mpuReady?static_cast<uint8_t>(MpuPhaseReady)
                         :static_cast<uint8_t>(mpuAttempt.phase);
  state.mpuHintText=mpuPhaseHint();
  state.joystickReady=joystick.fresh()&&joystick.calibrated();
  state.joystickX=joystick.xPercent();state.joystickY=joystick.yPercent();
  state.joystickDiagnostic=joystick.diagnostic();state.joystickLastError=joystick.lastError();
  state.joystickHasRaw=joystick.hasRaw();state.joystickFresh=joystick.fresh();
  state.joystickRawX=joystick.rawX();state.joystickRawY=joystick.rawY();
  state.joystickCalibration=joystick.calibrationPercent();state.joystickErrors=joystick.errors();
  state.joystickErrorStage=joystick.lastErrorStage();state.joystickWireResult=joystick.lastWireResult();
  state.anglesValid=oledAnglesValid && now-oledSampleMs<=120;
  state.controlReady=oledControlReady && state.anglesValid && connected && !buttonHeld;
  state.buttonHeld=buttonHeld;state.rssi=rssi;state.roll=oledRoll;state.pitch=oledPitch;
  oled.setStatus(state);
  oled.service(nextControlUs);
}

}

void setup() {
  Serial.begin(115200);
  Serial.println("HANDHELD V1.5 THREE MODES / ADS1115 / OLED DIAGNOSTICS");
  Serial.setTimeout(5);
  pinMode(kButtonPin,INPUT_PULLUP);
  pinMode(LED_BUILTIN,OUTPUT);
  digitalWrite(LED_BUILTIN,HIGH);

  Wire.begin(kSdaPin, kSclPin);
  Wire.setClock(100000);
  Wire.setClockStretchLimit(1500);
  oled.begin(kSdaPin,kSclPin,millis()); // Does not block or reinitialize Wire.
  joystick.begin(kSdaPin,kSclPin); // Shares Wire; register operations only in idle slots.

  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.setSleepMode(WIFI_NONE_SLEEP);
  connectToBase();
  udp.begin(4211);

  // V1.6：这里不再阻塞初始化 MPU。先让 OLED/WiFi/按键/摇杆立即工作，
  // MPU 的配置与静止校准交给 loop() 空闲时隙的 serviceMpuInit() 推进。
  mpuReady = false;
  mpuAttempt.phase = MpuPhaseIdle;
  mpuAttempt.retryAtMs = 0;
  Serial.println("HANDHELD_READY_MPU_INIT_DEFERRED");
}

void loop() {
  static uint32_t nextSendUs=micros();
  static bool wasConnected=false;
  updateButton();
  connectToBase();
  bool connected=WiFi.status()==WL_CONNECTED;
  if(wasConnected&&!connected) {
    requestedMode=0;modeEpoch++;healthyStarted=false;filterInitialized=false;
    joystickArmed=false;joystickNeutralStarted=false;
    Serial.println("LINK_LOST_RETURN_GIMBAL");
  }
  wasConnected=connected;
  const uint32_t nowUs=micros();
  if(static_cast<int32_t>(nowUs-nextSendUs)<0){
    joystick.service(nextSendUs);updateOled(nextSendUs);
    // 只在体感模式推进 MPU 初始化，摇杆模式下不占用共用 I2C 带宽。
    if(requestedMode==0||requestedMode==1)serviceMpuInit(nextSendUs);
    yield();return;
  }
  nextSendUs=nowUs+kSendPeriodUs;
  // Stop is independent of MPU health: do not enter a blocking calibration here.
  if(requestedMode==2) {
    sendFrame(false,0,0);oledControlReady=false;oledAnglesValid=false;
    digitalWrite(LED_BUILTIN,(millis()/500)%2?LOW:HIGH);yield();return;
  }
  // Joystick car uses fresh ADC pairs only. Wrist tilt must not affect it.
  if(requestedMode==3) {
    const bool usable=connected&&!buttonHeld&&joystick.fresh()&&joystick.calibrated();
    if(!usable){joystickArmed=false;joystickNeutralStarted=false;}
    else if(!joystickArmed) {
      if(!joystick.centered())joystickNeutralStarted=false;
      else {
        if(!joystickNeutralStarted){joystickNeutralStarted=true;joystickNeutralSinceMs=millis();}
        if(uint32_t(millis()-joystickNeutralSinceMs)>=HandheldInputConfig::kNeutralHoldMs)
          joystickArmed=true;
      }
    }
    const bool valid=usable&&joystickArmed;
    const float roll=joystick.rollTenths()/10.0f,pitch=joystick.pitchTenths()/10.0f;
    oledRoll=roll;oledPitch=pitch;oledSampleMs=millis();
    oledAnglesValid=joystick.fresh()&&joystick.calibrated();oledControlReady=valid;
    sendFrame(valid,roll,pitch);
    digitalWrite(LED_BUILTIN,valid?LOW:((millis()/100)%2?LOW:HIGH));
    if(sequence%25==0)Serial.printf("MODE=JOYSTICK READY=%u X=%d Y=%d ADC=%d,%d CAL=%u ERR=%lu\n",
      valid?1:0,joystick.xPercent(),joystick.yPercent(),joystick.rawX(),joystick.rawY(),
      joystick.calibrated()?1:0,(unsigned long)joystick.errors());
    yield();return;
  }
  if(!mpuReady) {
    // 初始化/校准由空闲时隙推进，这里只上报"未就绪"，不能阻塞。
    oledAnglesValid=false;oledControlReady=false;
    healthyStarted=false;
    sendFrame(false,0,0);
    digitalWrite(LED_BUILTIN,(millis()/100)%2?LOW:HIGH);
    yield();return;
  }
  float roll=0,pitch=0;
  if(!readControlAngles(roll,pitch)) {
    oledAnglesValid=false;oledControlReady=false;
    sendFrame(false,0,0);yield();return;
  }
  const float norm=sqrtf(float(lastAx)*lastAx+float(lastAy)*lastAy+float(lastAz)*lastAz)/16384.0f;
  bool healthy=connected&&!buttonHeld&&norm>0.5f&&norm<1.8f;
  if(!healthy)healthyStarted=false;
  else if(!healthyStarted){healthyStarted=true;healthySinceMs=millis();}
  bool valid=healthy&&healthyStarted&&millis()-healthySinceMs>=100;
  oledRoll=roll;oledPitch=pitch;oledSampleMs=millis();
  oledAnglesValid=true;oledControlReady=valid;
  sendFrame(valid,roll,pitch);
  bool light=!valid?(millis()/100)%2:
    requestedMode==1?true:requestedMode==2?(millis()/500)%2:millis()%1000<100;
  digitalWrite(LED_BUILTIN,light?LOW:HIGH);
  if(sequence%25==0) {
    Serial.printf("MODE=%u READY=%u ROLL=%.1f PITCH=%.1f\n",
                  requestedMode,valid?1:0,roll,pitch);
  }
  yield();
}
