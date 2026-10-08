#include "secrets.h"
/*
 * 手持端接线:
 *   ESP-01S VCC、EN/CH_PD -> 稳定3.3V，GND -> MPU6050 GND
 *   MPU6050 VCC           -> 3.3V
 *   MPU6050 SDA           -> ESP GPIO0
 *   MPU6050 SCL           -> ESP GPIO2
 *   MPU6050 AD0           -> GND（地址0x68）
 *   MPU6050 INT/XDA/XCL   -> 不接
 *   GPIO0运行时不能接GND；烧录时才接GND
 */
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <math.h>

namespace {
constexpr char kApName[] = "GimbalCtrl";
constexpr char kApPassword[] = GIMBAL_WIFI_PASSWORD;
const IPAddress kBaseIp(192, 168, 4, 1);
constexpr uint16_t kUdpPort = 4210;

constexpr uint8_t kMpuAddress = 0x68;
constexpr uint8_t kSdaPin = 0;
constexpr uint8_t kSclPin = 2;
constexpr uint32_t kSendPeriodUs = 20000;

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
KalmanState kalmanX;
KalmanState kalmanY;
float gyroOffsetX = 0.0f;
float gyroOffsetY = 0.0f;
uint32_t sequence = 0;
uint32_t lastSampleUs = 0;
uint32_t lastMpuRetryMs = 0;
bool mpuReady = false;
bool filterInitialized = false;
int16_t lastAx = 0;
int16_t lastAy = 0;
int16_t lastAz = 0;
int16_t lastGx = 0;
int16_t lastGy = 0;
int16_t lastGz = 0;

bool writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(kMpuAddress);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool readRegisters(uint8_t reg, uint8_t *data, size_t length) {
  Wire.beginTransmission(kMpuAddress);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  const size_t received = Wire.requestFrom(kMpuAddress, length, true);
  if (received != length) {
    return false;
  }

  for (size_t i = 0; i < length; ++i) {
    data[i] = Wire.read();
  }
  return true;
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

void calibrateGyro() {
  constexpr uint16_t kSamples = 300;
  int32_t sumX = 0;
  int32_t sumY = 0;
  uint16_t validSamples = 0;

  Serial.println("CALIBRATING_KEEP_STILL");
  for (uint16_t i = 0; i < kSamples; ++i) {
    int16_t ax, ay, az, gx, gy, gz;
    if (readRaw(ax, ay, az, gx, gy, gz)) {
      sumX += gx;
      sumY += gy;
      ++validSamples;
    }
    delay(3);
  }

  if (validSamples > 0) {
    gyroOffsetX = static_cast<float>(sumX) / validSamples / 131.0f;
    gyroOffsetY = static_cast<float>(sumY) / validSamples / 131.0f;
  }
  Serial.println("CALIBRATION_DONE");
}

bool initializeMpu() {
  uint8_t identity = 0;
  if (!readRegisters(0x75, &identity, 1) || identity != 0x68) {
    return false;
  }

  if (!writeRegister(0x6B, 0x01) ||
      !writeRegister(0x19, 0x04) ||
      !writeRegister(0x1A, 0x03) ||
      !writeRegister(0x1C, 0x00) ||
      !writeRegister(0x1B, 0x00)) {
    return false;
  }

  delay(50);
  calibrateGyro();
  filterInitialized = false;
  lastSampleUs = micros();
  return true;
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

void connectToBase() {
  if (WiFi.status() == WL_CONNECTED) {
    return;
  }

  WiFi.disconnect();
  WiFi.begin(kApName, kApPassword);
  Serial.print("WIFI_CONNECTING");
  const uint32_t deadline = millis() + 8000;
  while (WiFi.status() != WL_CONNECTED &&
         static_cast<int32_t>(millis() - deadline) < 0) {
    delay(250);
    Serial.print('.');
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("WIFI_CONNECTED,");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("WIFI_RETRY");
  }
}
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(5);

  Wire.begin(kSdaPin, kSclPin);
  Wire.setClock(100000);

  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.setSleepMode(WIFI_NONE_SLEEP);
  connectToBase();

  mpuReady = initializeMpu();
  Serial.println(mpuReady ? "HANDHELD_READY" : "MPU_NOT_FOUND");
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    connectToBase();
  }

  if (!mpuReady) {
    if (millis() - lastMpuRetryMs >= 1000) {
      lastMpuRetryMs = millis();
      mpuReady = initializeMpu();
      if (mpuReady) {
        Serial.println("MPU_CONNECTED");
      }
    }
    delay(10);
    return;
  }

  static uint32_t nextSendUs = micros();
  const uint32_t nowUs = micros();
  if (static_cast<int32_t>(nowUs - nextSendUs) < 0) {
    yield();
    return;
  }
  nextSendUs = nowUs + kSendPeriodUs;

  float roll = 0.0f;
  float pitch = 0.0f;
  if (!readFilteredAngles(roll, pitch)) {
    mpuReady = false;
    Serial.println("MPU_READ_ERROR");
    return;
  }

  char rollText[16];
  char pitchText[16];
  dtostrf(roll, 1, 2, rollText);
  dtostrf(pitch, 1, 2, pitchText);

  char message[64];
  const int length = snprintf(message, sizeof(message), "G,%lu,%s,%s\n",
                              static_cast<unsigned long>(sequence++),
                              rollText, pitchText);
  if (length > 0 && WiFi.status() == WL_CONNECTED) {
    udp.beginPacket(kBaseIp, kUdpPort);
    udp.write(reinterpret_cast<const uint8_t *>(message), length);
    udp.endPacket();
  }

  if ((sequence % 25U) == 0U) {
    Serial.print("RAW,");
    Serial.print(lastAx);
    Serial.print(',');
    Serial.print(lastAy);
    Serial.print(',');
    Serial.print(lastAz);
    Serial.print(',');
    Serial.print(lastGx);
    Serial.print(',');
    Serial.print(lastGy);
    Serial.print(',');
    Serial.print(lastGz);
    Serial.print(",ANGLE,");
    Serial.print(roll, 2);
    Serial.print(',');
    Serial.println(pitch, 2);
  }
}
