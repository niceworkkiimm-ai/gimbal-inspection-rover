/* Handheld V1.6 (2026-09-20): MPU status hint + progressive probe backoff.
 * V1.5: version label update for joystick X reversal; diagnostics unchanged.
 * Init register values checked against Adafruit_SSD1306 2.5.15.
 * Each update writes only 16 bytes; command and data run in separate idle slots.
 * MPU stays at its original 100 kHz / 1500 us stretch limit. OLED temporarily
 * uses a 15 us per-bit stretch limit (SSD1306 normally does not stretch SCL).
 * ESP8266 core 3.1.2 applies this limit per bit, NOT per whole transaction.
 * An absent screen retries at 5 s; a low bus is not probed. An actual short on
 * the shared wires can still affect the MPU: software cannot isolate wiring.
 */
#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "HandheldOled.h"
#include "HandheldOledFont.h"

namespace {
const uint32_t kIdleMarginUs = 6000;
const uint32_t kOledStretchUs = 15;
const uint32_t kMpuStretchUs = 1500;
const uint32_t kRefreshMs = 200;
struct InitCommand { uint8_t length, command, value; };
const InitCommand kInit[] = {
  {1,0xae,0}, {2,0xd5,0x80}, {2,0xa8,0x3f}, {2,0xd3,0x00},
  {1,0x40,0}, {2,0x8d,0x14}, {2,0x20,0x02}, // PAGE mode
  {1,0xa1,0}, {1,0xc8,0}, {2,0xda,0x12}, {2,0x81,0x7f},
  {2,0xd9,0xf1}, {2,0xdb,0x40}, {1,0xa4,0}, {1,0xa6,0}, {1,0x2e,0}
}; // 0xaf is sent only after all 1024 initial pixels have been written.

struct AsciiGlyph { char code; uint8_t columns[5]; };
const AsciiGlyph kAscii[] PROGMEM = {
  {' ',{0,0,0,0,0}}, {'+',{0x08,0x08,0x3e,0x08,0x08}},
  {'-',{0x08,0x08,0x08,0x08,0x08}}, {'.',{0,0x60,0x60,0,0}},
  {':',{0,0x36,0x36,0,0}}, {'?',{0x02,0x01,0x51,0x09,0x06}},
  {'0',{0x3e,0x51,0x49,0x45,0x3e}}, {'1',{0,0x42,0x7f,0x40,0}},
  {'2',{0x42,0x61,0x51,0x49,0x46}}, {'3',{0x21,0x41,0x45,0x4b,0x31}},
  {'4',{0x18,0x14,0x12,0x7f,0x10}}, {'5',{0x27,0x45,0x45,0x45,0x39}},
  {'6',{0x3c,0x4a,0x49,0x49,0x30}}, {'7',{0x01,0x71,0x09,0x05,0x03}},
  {'8',{0x36,0x49,0x49,0x49,0x36}}, {'9',{0x06,0x49,0x49,0x29,0x1e}},
  {'F',{0x7f,0x09,0x09,0x09,0x01}}, {'I',{0,0x41,0x7f,0x41,0}},
  {'M',{0x7f,0x02,0x0c,0x02,0x7f}}, {'P',{0x7f,0x09,0x09,0x09,0x06}},
  {'R',{0x7f,0x09,0x19,0x29,0x46}}, {'U',{0x3f,0x40,0x40,0x40,0x3f}},
  {'W',{0x3f,0x40,0x38,0x40,0x3f}}, {'i',{0,0x44,0x7d,0x40,0}}
  ,{'X',{0x63,0x14,0x08,0x14,0x63}}, {'Y',{0x07,0x08,0x70,0x08,0x07}},
  {'A',{0x7e,0x11,0x11,0x11,0x7e}}, {'D',{0x7f,0x41,0x41,0x22,0x1c}},
  {'S',{0x46,0x49,0x49,0x49,0x31}}, {'E',{0x7f,0x49,0x49,0x49,0x41}},
  {'L',{0x7f,0x40,0x40,0x40,0x40}}, {'N',{0x7f,0x04,0x08,0x10,0x7f}},
  {'C',{0x3e,0x41,0x41,0x41,0x22}}, {'V',{0x1f,0x20,0x40,0x20,0x1f}}
};

const char *joystickLabel(JoystickDiagnostic code) {
  switch(code) {
    case JoystickDiagnostic::Ready:return "手持就绪";
    case JoystickDiagnostic::WaitingSample:return "等待首次采样";
    case JoystickDiagnostic::NoAck:return "ADS无响应";
    case JoystickDiagnostic::WriteFailed:return "写入失败";
    case JoystickDiagnostic::ReadFailed:return "读取失败";
    case JoystickDiagnostic::BusLow:return "总线被拉低";
    case JoystickDiagnostic::TransferTimeout:return "总线传输超时";
    case JoystickDiagnostic::ConfigMismatch:return "配置读回异常";
    case JoystickDiagnostic::ConversionTimeout:return "转换等待超时";
    case JoystickDiagnostic::PairTimeout:return "两轴采样超时";
    case JoystickDiagnostic::ValueRange:return "原始数据越界";
    case JoystickDiagnostic::Stale:return "采样数据过期";
    case JoystickDiagnostic::CenterRange:return "中点超出范围";
    case JoystickDiagnostic::Unstable:return "中点读数不稳";
    case JoystickDiagnostic::Calibrating:return "等待中点校准";
    case JoystickDiagnostic::NoService:return "采样未获调度";
  }
  return "诊断状态异常";
}

bool enoughTime(uint32_t deadline) {
  return static_cast<int32_t>(deadline-micros()) >= static_cast<int32_t>(kIdleMarginUs);
}
}

void HandheldOled::begin(uint8_t sda, uint8_t scl, uint32_t nowMs) {
  sda_=sda; scl_=scl; phase_=ProbePrimary; address_=0x3c;
  nextProbeMs_=nowMs+200; renderPending_=true; pendingBlock_=-1;
  initStep_=cursor_=0; displayOn_=false; errors_=maxTransferUs_=0;
  memset(known_,0,sizeof(known_));
}

void HandheldOled::setStatus(const HandheldOledStatus &s) {
  if(s.mode!=status_.mode || s.connected!=status_.connected ||
     s.mpuReady!=status_.mpuReady || s.mpuState!=status_.mpuState ||
     s.anglesValid!=status_.anglesValid ||
     s.controlReady!=status_.controlReady || s.buttonHeld!=status_.buttonHeld ||
     s.joystickReady!=status_.joystickReady)
    renderPending_=true;
  status_=s;
}

bool HandheldOled::probe() {
  const uint32_t start=micros();
  Wire.setClockStretchLimit(kOledStretchUs);
  Wire.beginTransmission(address_);
  const uint8_t result=Wire.endTransmission();
  Wire.setClockStretchLimit(kMpuStretchUs);
  const uint32_t elapsed=micros()-start;
  if(elapsed>maxTransferUs_)maxTransferUs_=elapsed;
  return result==0 && elapsed<kIdleMarginUs && digitalRead(sda_) && digitalRead(scl_);
}

bool HandheldOled::transfer(uint8_t control,const uint8_t *data,uint8_t length) {
  const uint32_t start=micros();
  Wire.setClockStretchLimit(kOledStretchUs);
  Wire.beginTransmission(address_);
  Wire.write(control);
  Wire.write(data,length);
  const uint8_t result=Wire.endTransmission();
  Wire.setClockStretchLimit(kMpuStretchUs);
  const uint32_t elapsed=micros()-start;
  if(elapsed>maxTransferUs_)maxTransferUs_=elapsed;
  return result==0 && elapsed<kIdleMarginUs && digitalRead(sda_) && digitalRead(scl_);
}

void HandheldOled::failed() {
  ++errors_;
  if(consecutiveErrors_<255)++consecutiveErrors_;
  phase_=ProbePrimary; address_=0x3c;
  // V1.6：偶发错误先快速重试，连续失败再退避到5s，避免一次抖动就黑屏5秒。
  const uint32_t backoffMs=consecutiveErrors_<=3?250U*consecutiveErrors_:5000U;
  nextProbeMs_=millis()+backoffMs; pendingBlock_=-1; displayOn_=false;
  memset(known_,0,sizeof(known_));
}

void HandheldOled::service(uint32_t nextControlUs) {
  if(!enoughTime(nextControlUs))return;
  const uint32_t now=millis();
  if((phase_==ProbePrimary || phase_==ProbeSecondary) &&
     static_cast<int32_t>(now-nextProbeMs_)<0)return;
  if(!digitalRead(sda_) || !digitalRead(scl_)){failed();return;}
  if(phase_==ProbePrimary || phase_==ProbeSecondary) {
    if(probe()) {consecutiveErrors_=0;phase_=Initialize;initStep_=0;renderPending_=true;}
    else if(phase_==ProbePrimary){phase_=ProbeSecondary;address_=0x3d;}
    else failed();
    return;
  }
  if(phase_==Initialize) {
    const InitCommand &c=kInit[initStep_];
    const uint8_t bytes[]={c.command,c.value};
    if(!transfer(0x00,bytes,c.length)){failed();return;}
    if(++initStep_==sizeof(kInit)/sizeof(kInit[0]))phase_=Ready;
    return;
  }
  if(renderPending_ || now-lastRenderMs_>=kRefreshMs) {
    render(); lastRenderMs_=now; renderPending_=false;
    pendingBlock_=-1; cursor_=0;
  }
  if(!enoughTime(nextControlUs))return;
  if(pendingBlock_>=0) {
    const uint16_t offset=static_cast<uint16_t>(pendingBlock_)*16;
    if(!transfer(0x40,buffer_+offset,16)){failed();return;}
    memcpy(sent_+offset,buffer_+offset,16);known_[pendingBlock_]=true;
    cursor_=static_cast<uint8_t>((pendingBlock_+1)%64);pendingBlock_=-1;
    return;
  }
  for(uint8_t i=0;i<64;++i) {
    const uint8_t block=(cursor_+i)%64;
    const uint16_t offset=static_cast<uint16_t>(block)*16;
    if(known_[block] && memcmp(buffer_+offset,sent_+offset,16)==0)continue;
    const uint8_t column=offset%128, page=offset/128;
    const uint8_t position[]={static_cast<uint8_t>(0xb0|page),
      static_cast<uint8_t>(column&0x0f),static_cast<uint8_t>(0x10|(column>>4))};
    if(!transfer(0x00,position,sizeof(position))){failed();return;}
    pendingBlock_=block;return;
  }
  if(!displayOn_) {
    const uint8_t on=0xaf;
    if(!transfer(0x00,&on,1)){failed();return;}
    displayOn_=true;
  }
}

void HandheldOled::pixel(int x,int y) {
  if(x>=0 && x<128 && y>=0 && y<64)buffer_[(y/8)*128+x]|=1U<<(y%8);
}

void HandheldOled::ascii(int x,int y,const char *value) {
  for(;*value && x<128;++value,x+=6) {
    for(unsigned i=0;i<sizeof(kAscii)/sizeof(kAscii[0]);++i) {
      if(static_cast<char>(pgm_read_byte(&kAscii[i].code))!=*value)continue;
      for(uint8_t col=0;col<5;++col) {
        const uint8_t bits=pgm_read_byte(&kAscii[i].columns[col]);
        for(uint8_t row=0;row<7;++row)if(bits&(1U<<row))pixel(x+col,y+row);
      }
      break;
    }
  }
}

void HandheldOled::text(int x,int y,const char *value,uint8_t size) {
  while(*value && x<128) {
    const uint8_t c=static_cast<uint8_t>(*value++);
    if(c<0x80){const char s[]={static_cast<char>(c),0};ascii(x,y+2,s);x+=6;continue;}
    if((c&0xf0)!=0xe0 || !value[0] || !value[1])break;
    const uint16_t code=((c&0x0f)<<12)|((static_cast<uint8_t>(value[0])&0x3f)<<6)|
                         (static_cast<uint8_t>(value[1])&0x3f);
    value+=2;
    const OledGlyph *font=size==16?kOledFont16:kOledFont12;
    const unsigned count=size==16?sizeof(kOledFont16)/sizeof(kOledFont16[0]):
                                  sizeof(kOledFont12)/sizeof(kOledFont12[0]);
    for(unsigned i=0;i<count;++i) {
      if(pgm_read_word(&font[i].code)!=code)continue;
      for(uint8_t row=0;row<size;++row) {
        const uint16_t bits=pgm_read_word(&font[i].rows[row]);
        for(uint8_t col=0;col<size;++col)if(bits&(1U<<(size-1-col)))pixel(x+col,y+row);
      }
      break;
    }
    x+=size;
  }
}

void HandheldOled::render() {
  memset(buffer_,0,sizeof(buffer_));
  if(status_.mode==3){renderJoystick();return;}
  text(0,0,status_.connected?"WiFi 已连接":"WiFi 未连接",12);
  if(status_.connected) {
    char signal[8];snprintf(signal,sizeof(signal),"%d",status_.rssi);
    ascii(82,3,signal);
    const uint8_t bars=status_.rssi>=-55?4:status_.rssi>=-67?3:status_.rssi>=-78?2:1;
    for(uint8_t b=0;b<bars;++b)for(uint8_t x=0;x<3;++x)
      for(uint8_t y=0;y<3+b*2;++y)pixel(111+b*4+x,10-y);
  }
  for(int x=0;x<128;++x){pixel(x,13);pixel(x,34);}
  const char *mode=status_.mode==1?"体感小车":status_.mode==2?"停车请求":"体感云台";
  text(32,16,mode,16);
  if(status_.mpuReady && status_.anglesValid && isfinite(status_.roll) && isfinite(status_.pitch)) {
    char angles[32];snprintf(angles,sizeof(angles),"R:%+6.1f P:%+6.1f",status_.roll,status_.pitch);
    ascii(13,39,angles);
  } else ascii(13,39,"R:  --   P:  --");
  const char *hint;
  if(!status_.connected)hint="等待无线连接";
  else if(status_.mode==2)hint="短按返回云台";
  else if(status_.buttonHeld)hint="松开切换模式";
  else if(!status_.mpuReady)hint=status_.mpuHintText?status_.mpuHintText:"MPU异常请检查";
  else if(!status_.anglesValid)hint="姿态数据异常";
  else if(!status_.controlReady)hint="等待姿态稳定";
  else hint="手持就绪";
  int width=0;
  for(const uint8_t *s=reinterpret_cast<const uint8_t*>(hint);*s;){if(*s<128){width+=6;++s;}else{width+=12;s+=3;}}
  text((128-width)/2,51,hint,12);
}

void HandheldOled::renderJoystick() {
  // Fixed 128x64 layout, same 200ms/16-byte idle-slot refresh as other modes.
  text(0,0,"摇杆小车",12);ascii(54,3,"V1.5");
  ascii(99,3,status_.connected?"WiFi":"--");
  for(int x=0;x<128;++x)pixel(x,13);
  const JoystickDiagnostic code=status_.joystickDiagnostic;
  const char *hint=joystickLabel(code);
  if(code==JoystickDiagnostic::Ready) {
    if(!status_.connected)hint="等待无线连接";
    else if(status_.buttonHeld)hint="松开切换模式";
    else if(!status_.controlReady)hint="请松手回中";
  }
  char line[96];
  snprintf(line,sizeof(line),"E%02u %s",static_cast<unsigned>(code),hint);
  text(0,16,line,12);
  if(status_.joystickHasRaw)
    snprintf(line,sizeof(line),"X:%6d Y:%6d",status_.joystickRawX,status_.joystickRawY);
  else snprintf(line,sizeof(line),"X:    -- Y:    --");
  ascii(0,31,line);
  snprintf(line,sizeof(line),"S:%u W:%u C:%u",status_.joystickErrorStage,
    status_.joystickWireResult,status_.joystickCalibration);
  ascii(0,43,line);
  text(102,40,!status_.joystickHasRaw?"无值":status_.joystickFresh?"新值":"无效",12);
  // L and S/W retain the last sampling fault even after recovery; N counts faults.
  snprintf(line,sizeof(line),"L%02u N:%lu",static_cast<unsigned>(status_.joystickLastError),
    static_cast<unsigned long>(status_.joystickErrors));
  ascii(0,55,line);
}
