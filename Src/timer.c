#include "timer.h"
#include "stm32f072xb.h"
#include "stm32f0xx_hal.h"

void SetupTimer(TIM_TypeDef *Timer, IRQn_Type IRQ, uint8_t Priority)
{
	NVIC_EnableIRQ(IRQ);
	NVIC_SetPriority(IRQ, Priority);
}

void EnablePWMTimer(TIM_TypeDef *Timer, uint32_t Prescale, uint32_t Arr, uint32_t Compare)
{
	Timer->PSC = Prescale;
	Timer->ARR = Arr - 1;
	Timer->CCR1 |= Compare - 1;
	Timer->CCR2 |= Compare - 1;

	Timer->CCMR1 = 0x00;
	Timer->CCMR2 = 0x00;

	Timer->CCMR1 |= (0x00 << TIM_CCMR1_CC1S_Pos) | (0x00 << TIM_CCMR1_CC2S_Pos);
	Timer->CCMR1 |= (0x07 << TIM_CCMR1_OC1M_Pos) | (0x06 << TIM_CCMR1_OC2M_Pos);
	Timer->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E;
	Timer->CR1 |= TIM_CR1_CEN | TIM_CR1_ARPE;
}

void EnableTimer(TIM_TypeDef *Timer, uint32_t Prescale, uint32_t Arr, uint32_t Compare)
{	
	Timer->PSC = Prescale;
	Timer->ARR = Arr - 1;
	Timer->CCR1 = Compare - 1;
	Timer->CCMR1 = TIM_CCMR1_OC2M_0 | TIM_CCMR1_OC2M_1;
	Timer->DIER = TIM_DIER_CC1IE | TIM_DIER_UIE;
	Timer->CCER = TIM_CCER_CC1E;
	Timer->BDTR = TIM_BDTR_MOE;
	Timer->CR1 = TIM_CR1_CEN;
}

static volatile int Signal = 1;
int GetSignal()
{
	return Signal;
}

void IncSignal()
{
	Signal++;
	Signal %= 6;
}