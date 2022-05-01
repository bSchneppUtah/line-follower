#include "stdint.h"

#ifndef IRQ_H_
#define IRQ_H_

void SetupEXTI(uint32_t IMR, uint32_t FallEdge, uint32_t RisingEdge);

#endif