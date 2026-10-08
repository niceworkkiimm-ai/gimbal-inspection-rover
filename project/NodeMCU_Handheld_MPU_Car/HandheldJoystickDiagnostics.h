#ifndef HANDHELD_JOYSTICK_DIAGNOSTICS_H
#define HANDHELD_JOYSTICK_DIAGNOSTICS_H
#include <stdint.h>
// V1.4: local diagnostics only; these values never enter the H1 control frame.
enum class JoystickDiagnostic : uint8_t {
  Ready=0, WaitingSample=1, NoAck=2, WriteFailed=3, ReadFailed=4,
  BusLow=5, TransferTimeout=6, ConfigMismatch=7, ConversionTimeout=8,
  PairTimeout=9, ValueRange=10, Stale=11, CenterRange=12,
  Unstable=13, Calibrating=14, NoService=15
};
#endif
