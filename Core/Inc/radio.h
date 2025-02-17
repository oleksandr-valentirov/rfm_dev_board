#pragma once

#include <stdint.h>

#ifdef RFM69
#include "rfm69.h"
#include "rfm69_registers.h"
#endif

void RFM_Routine(void);
uint8_t RFM_Init(uint8_t network_id, uint8_t node_id);
