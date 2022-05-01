#include "stm32f072xb.h"
#include "stm32f0xx_hal.h"

#ifndef TIMER_H_
#define TIMER_H_

void SetupTimer(TIM_TypeDef *Timer, IRQn_Type IRQ, uint8_t Priority);

void EnableTimer(TIM_TypeDef *Timer, uint32_t Prescale, uint32_t Arr, uint32_t Compare);
void EnablePWMTimer(TIM_TypeDef *Timer, uint32_t Prescale, uint32_t Arr, uint32_t Compare);

int GetSignal();
void IncSignal();

#endif