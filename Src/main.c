
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include "motor.h"
#include "stm32f0xx.h"
#include "stm32f072xb.h"
#include "stm32f0xx_hal.h"


/*******************/
/* Light sensor stuff (PC0) */
/*******************/
void InitSensorPin(uint32_t PinIndex)
{
    const uint32_t Output = GPIO_MODE_INPUT;
    const uint32_t Speed = GPIO_SPEED_FREQ_LOW;
    const uint32_t Pull = GPIO_NOPULL;

    uint32_t OutputMode = GPIOC->MODER;
    OutputMode &= ~(GPIO_MODER_MODER0 << (0x2 * PinIndex));
    OutputMode |= (Output & 0x03) << (0x2 * PinIndex);
    GPIOC->MODER = OutputMode;

    uint32_t SpeedMode = GPIOC->OSPEEDR;
    SpeedMode &= ~(GPIO_OSPEEDER_OSPEEDR0 << (0x2 * PinIndex));
    SpeedMode |= (Speed << (0x2 * PinIndex));
    GPIOC->OSPEEDR = SpeedMode;
}

void InitButtonPin(uint32_t PinIndex)
{
    const uint32_t Output = GPIO_MODE_INPUT;
    const uint32_t Speed = GPIO_SPEED_FREQ_LOW;

    GPIOA->MODER &= ~(0x3 << (0x2 * PinIndex));
    GPIOA->MODER |= (Output & 0x03) << (0x2 * PinIndex);

    GPIOA->OTYPER &= ~(1 << PinIndex);

    GPIOA->OSPEEDR &= ~(0x03 << (0x2 * PinIndex));
    GPIOA->OSPEEDR |= (Speed << (0x2 * PinIndex));

    GPIOA->PUPDR &= ~(0x03 << (0x02 * PinIndex));
    GPIOA->PUPDR |= (0x00 << (0x02 * PinIndex));

    if (PinIndex < 8)
    {
        GPIOA->AFR[0] &= ~(0xF << (4 * PinIndex));
    }
    else
    {
        GPIOA->AFR[1] &= ~(0xF << (4 * (PinIndex-8)));
    }
}

void InitSensor2Pin(uint32_t PinIndex)
{
    const uint32_t Output = GPIO_MODE_INPUT;
    const uint32_t Speed = GPIO_SPEED_FREQ_LOW;

    GPIOC->MODER &= ~(0x3 << (0x2 * PinIndex));
    GPIOC->MODER |= (Output & 0x03) << (0x2 * PinIndex);

    GPIOC->OTYPER &= ~(1 << PinIndex);

    GPIOC->OSPEEDR &= ~(0x03 << (0x2 * PinIndex));
    GPIOC->OSPEEDR |= (Speed << (0x2 * PinIndex));

    GPIOC->PUPDR &= ~(0x03 << (0x02 * PinIndex));
    GPIOC->PUPDR |= (0x00 << (0x02 * PinIndex));

    if (PinIndex < 8)
    {
        GPIOC->AFR[0] &= ~(0xF << (4 * PinIndex));
    }
    else
    {
        GPIOC->AFR[1] &= ~(0xF << (4 * (PinIndex-8)));
    }
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
    // Initialize everything but PC8 C8 and PC9 for LED's
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;                                          // Enable peripheral clock to GPIOC
    GPIOC->MODER |= GPIO_MODER_MODER6_0 | GPIO_MODER_MODER7_0 | GPIO_MODER_MODER8_0 | GPIO_MODER_MODER9_0;                  // Set PC8 & PC9 to outputs
    GPIOC->OTYPER &= ~(GPIO_OTYPER_OT_6 | GPIO_OTYPER_OT_7 | GPIO_OTYPER_OT_8 | GPIO_OTYPER_OT_9);                    // Set to push-pull output type
    GPIOC->OSPEEDR &= ~((GPIO_OSPEEDR_OSPEEDR6_0 | GPIO_OSPEEDR_OSPEEDR6_1) |
                        (GPIO_OSPEEDR_OSPEEDR7_0 | GPIO_OSPEEDR_OSPEEDR7_1) |
                        (GPIO_OSPEEDR_OSPEEDR8_0 | GPIO_OSPEEDR_OSPEEDR8_1) |
                        (GPIO_OSPEEDR_OSPEEDR9_0 | GPIO_OSPEEDR_OSPEEDR9_1));   // Set to low speed
    GPIOC->PUPDR &= ~((GPIO_PUPDR_PUPDR6_0 | GPIO_PUPDR_PUPDR6_1) |
                      (GPIO_PUPDR_PUPDR7_0 | GPIO_PUPDR_PUPDR7_1) |
                      (GPIO_PUPDR_PUPDR8_0 | GPIO_PUPDR_PUPDR8_1) |
                      (GPIO_PUPDR_PUPDR9_0 | GPIO_PUPDR_PUPDR9_1));             // Set to no pull-up/down
    GPIOC->ODR &= ~(GPIO_ODR_6 | GPIO_ODR_7 | GPIO_ODR_8 | GPIO_ODR_9);                                   // Shut off LED's
}

void  button_init(void) {
    // Initialize PA0 for button input
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN;                                          // Enable peripheral clock to GPIOA
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
    SystemClock_Config();

    LED_init();                             // Initialize LED's
    button_init();
    InitButtonPin(0);
    InitSensor2Pin(0);

    for (int Index = 0; Index <= 1; Index++)
    {
        InitSensorPin(Index);
    }
    target_rpm = 130;
    motor_init();                           // Initialize motor code
    while (1) 
    {
        HAL_Delay(128);                      // Delay 1/8 second

        GPIOC->ODR &= ~(GPIO_ODR_7 | GPIO_ODR_6);

        uint32_t AIDR = GPIOA->IDR;
        uint32_t CIDR = GPIOC->IDR;
        if (AIDR & GPIO_PIN_0)
        {
            GPIOC->ODR |= GPIO_ODR_7;
        }

        if (CIDR & GPIO_PIN_0)
        {
            GPIOC->ODR |= GPIO_ODR_6;
        }
    }
}

void _Error_Handler(void *a, void *b)
{

}

void SystemClock_Config(void)
{

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;

		/**Initializes the CPU, AHB and APB busses clocks
		*/
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = 16;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

		/**Initializes the CPU, AHB and APB busses clocks
		*/
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

		/**Configure the Systick interrupt time
		*/
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

		/**Configure the Systick
		*/
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

// ----------------------------------------------------------------------------
