#include <stdint.h>

#ifndef LED_H_
#define LED_H_

void EnableLEDPin(uint32_t PinNo);
void DisableLEDPin(uint32_t PinNo);
void ToggleLEDPin(uint32_t PinNo);

#endif