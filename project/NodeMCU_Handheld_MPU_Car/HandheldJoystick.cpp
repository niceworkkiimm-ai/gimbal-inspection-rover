/* ADS1115 implementation V1.4, TI SBAS444E (Dec 2024), sections 7.5/8.1.3.
 * Adds error classification/raw-value visibility, no extra I2C transactions.
 * VDD=3.3V; AIN0/1 single-ended, PGA +/-4.096V, single-shot, 860SPS.
 * Config: A0=0xC3E3, A1=0xD3E3. OS polled; verify MUX/PGA/MODE/DR.
 * ADS cannot drive SCL (TI 7.5.2); use 15us per-bit stretch like the OLED.
 * Conversion timeout30ms / pair50ms are elapsed deadlines, never blocking waits.
 * I2C errors invalidate both axes. Analog wire break/short at a plausible voltage
 * cannot always be detected by software; keep the motor-power cutoff accessible.
 */
#include <Arduino.h>
#include <Wire.h>
#include <stdlib.h>
#include "HandheldJoystick.h"
#include "HandheldInputConfig.h"
using namespace HandheldInputConfig;

void HandheldJoystick::begin(uint8_t sda,uint8_t scl) {
  sda_=sda;scl_=scl;stage_=0;nextPairUs_=micros();
  valid_=calibrated_=false;centerSamples_=0;errors_=0;
  hasRaw_=calibrationUnstable_=serviceSeen_=false;lastServiceMs_=millis();
  diagnostic_=JoystickDiagnostic::WaitingSample;lastError_=JoystickDiagnostic::Ready;
  wireResult_=lastWireResult_=lastErrorStage_=0;
}
bool HandheldJoystick::writeConfig(uint16_t value) {
  const uint32_t start=micros();
  Wire.setClockStretchLimit(15);
  Wire.beginTransmission(kAdsAddress);Wire.write(1);
  Wire.write(static_cast<uint8_t>(value>>8));Wire.write(static_cast<uint8_t>(value));
  const uint8_t result=Wire.endTransmission();
  Wire.setClockStretchLimit(1500);
  wireResult_=result;
  if(result!=0){ioFailure_=result==2?JoystickDiagnostic::NoAck:JoystickDiagnostic::WriteFailed;return false;}
  if(micros()-start>=6000){ioFailure_=JoystickDiagnostic::TransferTimeout;return false;}
  if(!digitalRead(sda_) || !digitalRead(scl_)){ioFailure_=JoystickDiagnostic::BusLow;return false;}
  return true;
}
bool HandheldJoystick::readRegister(uint8_t reg,uint16_t &value) {
  const uint32_t start=micros();bool ok=false;
  Wire.setClockStretchLimit(15);
  Wire.beginTransmission(kAdsAddress);Wire.write(reg);
  wireResult_=Wire.endTransmission(false);
  if(wireResult_==0 && Wire.requestFrom(kAdsAddress,size_t(2),true)==2) {
    const uint8_t high=static_cast<uint8_t>(Wire.read()),low=static_cast<uint8_t>(Wire.read());
    value=(uint16_t(high)<<8)|low;ok=true;
  }
  Wire.setClockStretchLimit(1500);
  if(!ok){ioFailure_=wireResult_==2?JoystickDiagnostic::NoAck:JoystickDiagnostic::ReadFailed;return false;}
  if(micros()-start>=6000){ioFailure_=JoystickDiagnostic::TransferTimeout;return false;}
  if(!digitalRead(sda_) || !digitalRead(scl_)){ioFailure_=JoystickDiagnostic::BusLow;return false;}
  return true;
}
void HandheldJoystick::recordError(JoystickDiagnostic reason) {
  lastError_=reason;lastErrorStage_=stage_;lastWireResult_=wireResult_;++errors_;
}
void HandheldJoystick::failed(JoystickDiagnostic reason) {
  diagnostic_=reason;recordError(reason);
  valid_=false;stage_=0;nextPairUs_=micros()+500000;
  centerSamples_=0; // Retain an already learned center, never relearn while driving.
  calibrationUnstable_=false;
}
bool HandheldJoystick::fresh() const {
  return valid_ && uint32_t(millis()-lastPairMs_)<=kSampleFreshMs;
}
JoystickDiagnostic HandheldJoystick::diagnostic() const {
  // Pure observation: never updates valid_, calibrated_, arming or ADC scheduling.
  if(uint32_t(millis()-lastServiceMs_)>kSampleFreshMs)return JoystickDiagnostic::NoService;
  if(valid_ && !fresh())return JoystickDiagnostic::Stale;
  return diagnostic_;
}
uint8_t HandheldJoystick::calibrationPercent() const {
  if(calibrated_)return 100;
  if(!fresh() || !centerSamples_)return 0;
  uint32_t timePart=uint32_t(millis()-centerSinceMs_)*100/kCalibrationHoldMs;
  uint32_t samplePart=uint32_t(centerSamples_)*100/12;
  uint32_t percent=timePart<samplePart?timePart:samplePart;
  return static_cast<uint8_t>(percent>99?99:percent);
}
int16_t HandheldJoystick::normalized(int16_t raw,int16_t center) {
  const int32_t delta=int32_t(raw)-center;
  const int32_t span=delta>=0?kAdcFull3V3-center:center;
  if(span<=0)return 0;
  int32_t n=delta*1000/span;
  if(n>1000)n=1000;
  if(n < -1000)n=-1000;
  return static_cast<int16_t>(n);
}
bool HandheldJoystick::centered() const {
  return fresh() && calibrated_ &&
    abs(normalized(rawX_,centerX_))<=int(kDeadzonePercent*10) &&
    abs(normalized(rawY_,centerY_))<=int(kDeadzonePercent*10);
}
int16_t HandheldJoystick::controlAngle(int16_t value) {
  const int32_t n=abs(value),dead=kDeadzonePercent*10;
  if(n<=dead)return 0;
  const int32_t angle=kCarDeadzoneTenths+(n-dead)*(kCarFullTenths-kCarDeadzoneTenths)/(1000-dead);
  return static_cast<int16_t>(value<0?-angle:angle);
}
int16_t HandheldJoystick::xPercent() const { return kXSign*normalized(rawX_,centerX_)/10; }
int16_t HandheldJoystick::yPercent() const { return kYSign*normalized(rawY_,centerY_)/10; }
int16_t HandheldJoystick::rollTenths() const {
  // Existing STM32 CAR_TURN_LEFT_SIGN=-1: positive roll requests right turn.
  return kCarRollZeroTenths+controlAngle(kXSign*normalized(rawX_,centerX_));
}
int16_t HandheldJoystick::pitchTenths() const {
  return kCarPitchZeroTenths+controlAngle(kYSign*normalized(rawY_,centerY_));
}
void HandheldJoystick::acceptPair() {
  // +/-4.096V scaling does NOT permit inputs above the 3.3V supply.
  if(rawX_ < -16 || rawY_ < -16 || rawX_>27200 || rawY_>27200){failed(JoystickDiagnostic::ValueRange);return;}
  if(rawX_<0)rawX_=0; // ADC offset at a legitimate 0V endpoint.
  if(rawY_<0)rawY_=0;
  lastPairMs_=millis();valid_=true;
  if(calibrated_){diagnostic_=JoystickDiagnostic::Ready;return;}
  if(abs(int32_t(rawX_)-kExpectedCenter)>kCenterWindow ||
     abs(int32_t(rawY_)-kExpectedCenter)>kCenterWindow){
    centerSamples_=0;calibrationUnstable_=false;diagnostic_=JoystickDiagnostic::CenterRange;return;
  }
  if(!centerSamples_ || abs(int(rawX_)-minX_)>kCalibrationJitter ||
     abs(int(rawX_)-maxX_)>kCalibrationJitter || abs(int(rawY_)-minY_)>kCalibrationJitter ||
     abs(int(rawY_)-maxY_)>kCalibrationJitter) {
    calibrationUnstable_=centerSamples_!=0;
    diagnostic_=calibrationUnstable_?JoystickDiagnostic::Unstable:JoystickDiagnostic::Calibrating;
    centerSamples_=1;centerSinceMs_=millis();sumX_=rawX_;sumY_=rawY_;
    minX_=maxX_=rawX_;minY_=maxY_=rawY_;return;
  }
  if(rawX_<minX_)minX_=rawX_;
  if(rawX_>maxX_)maxX_=rawX_;
  if(rawY_<minY_)minY_=rawY_;
  if(rawY_>maxY_)maxY_=rawY_;
  sumX_+=rawX_;sumY_+=rawY_;++centerSamples_;
  diagnostic_=calibrationUnstable_?JoystickDiagnostic::Unstable:JoystickDiagnostic::Calibrating;
  if(centerSamples_>=12 && uint32_t(millis()-centerSinceMs_)>=kCalibrationHoldMs) {
    centerX_=static_cast<int16_t>(sumX_/centerSamples_);
    centerY_=static_cast<int16_t>(sumY_/centerSamples_);calibrated_=true;
    diagnostic_=JoystickDiagnostic::Ready;calibrationUnstable_=false;
  }
}
void HandheldJoystick::service(uint32_t deadline) {
  const uint32_t now=micros();
  if(static_cast<int32_t>(deadline-now)<6000)return;
  // A starved OLED cannot render E15 while the same idle slot is unavailable.
  // Preserve L15 when service resumes between pairs, without changing validity,
  // calibration, backoff or control. Mid-pair gaps retain the existing E09 path.
  if(serviceSeen_ && stage_==0 && uint32_t(millis()-lastServiceMs_)>kSampleFreshMs){
    wireResult_=0;recordError(JoystickDiagnostic::NoService);
  }
  serviceSeen_=true;
  lastServiceMs_=millis();
  if(stage_==0 && static_cast<int32_t>(now-nextPairUs_)<0)return;
  wireResult_=0;
  if(!digitalRead(sda_) || !digitalRead(scl_)){failed(JoystickDiagnostic::BusLow);return;}
  uint16_t value=0;
  if(stage_ && uint32_t(now-pairUs_)>50000){failed(JoystickDiagnostic::PairTimeout);return;}
  if(stage_==0 || stage_==3) {
    config_=stage_==0?0xC3E3:0xD3E3;
    if(!writeConfig(config_)){failed(ioFailure_);return;}
    if(stage_==0){pairUs_=micros();nextPairUs_=pairUs_+20000;}
    conversionUs_=micros();++stage_;return;
  }
  if(stage_==1 || stage_==4) {
    if(uint32_t(now-conversionUs_)<2000)return;
    if(uint32_t(now-conversionUs_)>30000){failed(JoystickDiagnostic::ConversionTimeout);return;}
    if(!readRegister(1,value)){failed(ioFailure_);return;}
    if((value&0x7FFF)!=(config_&0x7FFF)){failed(JoystickDiagnostic::ConfigMismatch);return;}
    if(value&0x8000)++stage_;
    return;
  }
  if(!readRegister(0,value)){failed(ioFailure_);return;}
  if(stage_==2){pendingX_=static_cast<int16_t>(value);stage_=3;}
  else {rawX_=pendingX_;rawY_=static_cast<int16_t>(value);hasRaw_=true;stage_=0;acceptPair();}
}
