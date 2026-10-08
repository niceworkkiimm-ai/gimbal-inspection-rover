#ifndef __GAS_SENSOR_H
#define __GAS_SENSOR_H

#include "stm32f4xx.h"

#define GAS_SENSOR_CHANNEL_COUNT 4U

typedef struct
{
    uint8_t address;
    uint8_t online; /* Fresh CRC-checked response, including sensor faults. */
    uint8_t valid;
    uint32_t concentration_raw; /* 18 bits from registers 0 and 1. */
    uint8_t unit_code;
    uint8_t decimals;
    uint8_t status;
    uint8_t gas_type;
} GasSensorReading_t;

void GasSensor_Init(void);
void GasSensor_Poll(void);
uint8_t GasSensor_GetReading(uint8_t channel, GasSensorReading_t *reading);

#endif
