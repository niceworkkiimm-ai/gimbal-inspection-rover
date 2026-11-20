#ifndef WHEEL_ENCODER_H
#define WHEEL_ENCODER_H
#include "stm32f4xx.h"
#include "EncoderProtocol.h"
void WheelEncoder_Init(void);
/* Returns a recent signal-quality fault bit for each wheel (LF, RF, LR, RR). */
uint8_t WheelEncoder_Snapshot(EncoderFrame *frame, uint32_t sequence);
#endif
