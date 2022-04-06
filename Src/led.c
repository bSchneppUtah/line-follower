#include <stdint.h>
#include "stm32f0xx_hal.h"

void EnableLEDPin(uint32_t PinNo)
{
	GPIOC->BSRR = PinNo;
}

void DisableLEDPin(uint32_t PinNo)
{
	const uint32_t UpperHalf = 16;
	GPIOC->BSRR = PinNo << UpperHalf;
}


void ToggleLEDPin(uint32_t PinNo)
{
	if ((GPIOC->ODR & PinNo) != 0X00u)
	{
		DisableLEDPin(PinNo);
	}
	else
	{
		EnableLEDPin(PinNo);
	}
	
}