#include "irq.h"
#include "stdint.h"

#include "stm32f072xb.h"
#include "stm32f0xx_hal.h"

void SetupEXTI(uint32_t IMR, uint32_t FallEdge, uint32_t RisingEdge)
{
	EXTI->IMR = IMR;
	EXTI->FTSR = FallEdge;
	EXTI->RTSR = RisingEdge;
}