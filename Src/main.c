
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include "motor.h"
#include "stm32f0xx.h"
#include "stm32f072xb.h"
#include "stm32f0xx_hal.h"

/* New stuff */
void InitGPIOCPWM(uint32_t PinIndex)
{
	const uint32_t Output = GPIO_MODE_AF_PP;
	const uint32_t Speed = GPIO_SPEED_FREQ_LOW;
	const uint32_t Pull = GPIO_NOPULL;

	/* Configure output type */
	uint32_t OutputMode = GPIOC->MODER;
	OutputMode &= ~(GPIO_MODER_MODER0 << (0x2 * PinIndex));
	OutputMode |= (Output & 0x03) << (0x2 * PinIndex);
	GPIOC->MODER = OutputMode;

	/* Configure i/o output type	*/
	uint32_t TypeMode = GPIOC->OTYPER;
	TypeMode &= ~(GPIO_OTYPER_OT_0 << (0x2 * PinIndex));
	TypeMode |= (((GPIO_MODE_OUTPUT_PP & 0x10) >> 4U) << (0x2 * PinIndex));
	GPIOC->OTYPER = TypeMode;

	/* Configure i/o output speed */
	uint32_t SpeedMode = GPIOC->OSPEEDR;
	SpeedMode &= ~(GPIO_OSPEEDER_OSPEEDR0 << (0x2 * PinIndex));
	SpeedMode |= (Speed << (0x2 * PinIndex));
	GPIOC->OSPEEDR = SpeedMode;

	/* Setup pull-up or pull-down for this pin */
	uint32_t PullUpDownMode = GPIOC->PUPDR;
	PullUpDownMode &= ~(GPIO_PUPDR_PUPDR0 << (0x2 * PinIndex));
	PullUpDownMode |= ((Pull) << (0x2 * PinIndex));
	GPIOC->PUPDR = PullUpDownMode;
}

/* -------------------------------------------------------------------------------------------------------------
 *  Global Variable Declarations
 *  -------------------------------------------------------------------------------------------------------------
 */
volatile uint32_t debouncer;

/* -------------------------------------------------------------------------------------------------------------
 *  Miscellaneous Core Functions
 *  -------------------------------------------------------------------------------------------------------------
 */

void LED_init(void) {
    // Initialize PC8 and PC9 for LED's
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;                                          // Enable peripheral clock to GPIOC
    GPIOC->MODER |= GPIO_MODER_MODER8_0 | GPIO_MODER_MODER9_0;                  // Set PC8 & PC9 to outputs
    GPIOC->OTYPER &= ~(GPIO_OTYPER_OT_8 | GPIO_OTYPER_OT_9);                    // Set to push-pull output type
    GPIOC->OSPEEDR &= ~((GPIO_OSPEEDR_OSPEEDR8_0 | GPIO_OSPEEDR_OSPEEDR8_1) |
                        (GPIO_OSPEEDR_OSPEEDR9_0 | GPIO_OSPEEDR_OSPEEDR9_1));   // Set to low speed
    GPIOC->PUPDR &= ~((GPIO_PUPDR_PUPDR8_0 | GPIO_PUPDR_PUPDR8_1) |
                      (GPIO_PUPDR_PUPDR9_0 | GPIO_PUPDR_PUPDR9_1));             // Set to no pull-up/down
    GPIOC->ODR &= ~(GPIO_ODR_8 | GPIO_ODR_9);                                   // Shut off LED's
}

void  button_init(void) {
    // Initialize PA0 for button input
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;                                          // Enable peripheral clock to GPIOA
    GPIOA->MODER &= ~(GPIO_MODER_MODER0_0 | GPIO_MODER_MODER0_1);               // Set PA0 to input
    GPIOC->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEEDR0_0 | GPIO_OSPEEDR_OSPEEDR0_1);     // Set to low speed
    GPIOC->PUPDR |= GPIO_PUPDR_PUPDR0_1;                                        // Set to pull-down
}

/* Called by SysTick Interrupt
 * Performs button debouncing, changes wave type on button rising edge
 * Updates frequency output from ADC value
 */
void HAL_SYSTICK_Callback(void) {
    // Remember that this function is called by the SysTick interrupt
    // You can't call any functions in here that use delay
    debouncer = (debouncer << 1);
    if(GPIOA->IDR & GPIO_PIN_0) {
        debouncer |= 0x1;
    }
}

/* -------------------------------------------------------------------------------------------------------------
 * Main Program Code
 *
 * Starts initialization of peripherals
 * Blinks green LED (PC9) in loop as heartbeat
 * -------------------------------------------------------------------------------------------------------------
 */
volatile uint32_t encoder_count = 0;

int main(int argc, char* argv[]) {

    debouncer = 0;                          // Initialize global variables
    HAL_Init();															// Initialize HAL
    LED_init();                             // Initialize LED's
    button_init();                          // Initialize button
    target_rpm = 130;
    motor_init();                           // Initialize motor code
    InitGPIOCPWM(9);
    while (1) 
    {
        HAL_Delay(128);                      // Delay 1/8 second

        GPIOC->ODR &= ~(GPIO_ODR_7);
        if (GPIOA->IDR & GPIO_PIN_0)
        {
            GPIOC->ODR |= GPIO_ODR_7;
        }
    }
}

// ----------------------------------------------------------------------------
