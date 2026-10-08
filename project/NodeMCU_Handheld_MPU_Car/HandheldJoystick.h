#ifndef HANDHELD_JOYSTICK_H
#define HANDHELD_JOYSTICK_H
#include <stdint.h>
#include "HandheldJoystickDiagnostics.h"
// ADS1115 V1.4: diagnostics do not change sampling/arming limits.
class HandheldJoystick {
public:
  void begin(uint8_t sda, uint8_t scl);
  // One register operation at most, only in an idle slot >=6ms.
  void service(uint32_t nextControlUs);
  bool fresh() const;
  bool calibrated() const { return calibrated_; }
  bool centered() const;
  int16_t rollTenths() const;
  int16_t pitchTenths() const;
  int16_t xPercent() const;
  int16_t yPercent() const;
  int16_t rawX() const { return rawX_; }
  int16_t rawY() const { return rawY_; }
  uint32_t errors() const { return errors_; }
  JoystickDiagnostic diagnostic() const;
  JoystickDiagnostic lastError() const { return lastError_; }
  bool hasRaw() const { return hasRaw_; }
  uint8_t calibrationPercent() const;
  uint8_t lastErrorStage() const { return lastErrorStage_; }
  uint8_t lastWireResult() const { return lastWireResult_; }
private:
  uint8_t sda_=4,scl_=5,stage_=0;
  uint16_t config_=0;
  int16_t pendingX_=0,rawX_=0,rawY_=0,centerX_=13200,centerY_=13200;
  int16_t minX_=0,maxX_=0,minY_=0,maxY_=0;
  int32_t sumX_=0,sumY_=0;
  uint16_t centerSamples_=0;
  uint32_t nextPairUs_=0,conversionUs_=0,pairUs_=0,lastPairMs_=0,centerSinceMs_=0,errors_=0;
  bool valid_=false,calibrated_=false;
  bool hasRaw_=false,calibrationUnstable_=false,serviceSeen_=false;
  uint32_t lastServiceMs_=0;
  JoystickDiagnostic diagnostic_=JoystickDiagnostic::WaitingSample;
  JoystickDiagnostic lastError_=JoystickDiagnostic::Ready;
  JoystickDiagnostic ioFailure_=JoystickDiagnostic::ReadFailed;
  uint8_t wireResult_=0,lastWireResult_=0,lastErrorStage_=0;
  bool writeConfig(uint16_t value);
  bool readRegister(uint8_t reg,uint16_t &value);
  void recordError(JoystickDiagnostic reason);
  void failed(JoystickDiagnostic reason);
  void acceptPair();
  static int16_t normalized(int16_t raw,int16_t center);
  static int16_t controlAngle(int16_t normalizedValue);
};
#endif
