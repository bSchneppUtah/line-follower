
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include "motor.h"
#include "stm32f0xx.h"
#include "stm32f072xb.h"
#include "stm32f0xx_hal.h"

void InitGPIOCPinAlternate(uint32_t PinIndex)
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

void WriteCharRaw(USART_TypeDef *Def, char Cur)
{	
	Def->TDR = Cur;
}

void WriteChar(USART_TypeDef *Def, char Cur)
{
	if (Cur == '\n')
	{
		WriteCharRaw(Def, '\r');
	}
	WriteCharRaw(Def, Cur);
	while ((Def->ISR & USART_ISR_TC) != USART_ISR_TC)
	{
	}
}

void FiniWrite(USART_TypeDef *Def)
{
	Def->ICR |= USART_ICR_TCCF;
}

void WriteString(USART_TypeDef *Def, const char *Str)
{
	for (uint16_t Index = 0;; Index++)
	{
		char Cur = Str[Index];
		if (Cur == 0x00)
		{
			break;
		}
		WriteChar(Def, Cur);
	}
	FiniWrite(Def);	
}

char RecvChar(USART_TypeDef *Def)
{
	for (;;)
	{
		if ((Def->ISR & USART_ISR_RXNE) == USART_ISR_RXNE)
		{
			return Def->RDR;
		}
	}
}

void uart_init()
{
	RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
	
	InitGPIOCPinAlternate(4);
	InitGPIOCPinAlternate(5);

	GPIOC->AFR[0] |= (1 << 16) | (1 << 20);	
	
	/* Get the right baud rate... */
	uint32_t DestBaud = 115200;
	uint32_t SrcClock = HAL_RCC_GetHCLKFreq();
	uint32_t BaudBRR = SrcClock / DestBaud;

	USART3->BRR = BaudBRR;
	USART3->CR3 = USART_CR3_CTSE | USART_CR3_RTSE;
	NVIC_EnableIRQ(USART3_4_IRQn);
	USART3->CR1 = USART_CR1_RXNEIE | USART_CR1_RE | USART_CR1_UE | USART_CR1_TE;
}


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

void itoa16(int Num, char *Out)
{
	int Index = 0;
	do
	{
		int NewNum = Num % 16;
		if (Num < 0)
		{
			/* Fix negatives... C is a little weird with negative numbers here. */
			NewNum = 16 + NewNum;
		}
		Out[Index++] = (NewNum < 10) ? '0' + NewNum : 'A' + (NewNum - 10);
	} while ((Num /= 16) > 0);

	/* Flip the number so we get what we wanted */
	for (int Subindex = 0; Subindex < Index / 2; ++Subindex)
	{
		char Tmp = Out[Subindex];
		Out[Subindex] = Out[Index - Subindex - 1];
		Out[Index - Subindex - 1] = Tmp;
	}

	/* Be 10000% sure we have a null character. */
	Out[Index] = '\0';
}

int main(int argc, char* argv[]) {

    debouncer = 0;                          // Initialize global variables
    HAL_Init();															// Initialize HAL
    SystemClock_Config();

    LED_init();                             // Initialize LED's
    button_init();
    InitButtonPin(0);
    InitSensorPin(0);
    InitSensor2Pin(0);

    uart_init();
    motor_init();                           // Initialize motor code
    target_rpm = 100;
    while (1) 
    {
        HAL_Delay(128);                      // Delay 1/8 second

        char *OK = "0";
        GPIOC->ODR &= ~(GPIO_ODR_7 | GPIO_ODR_6);
        uint32_t CIDR = GPIOC->IDR;
        if (CIDR & GPIO_PIN_0)
        {
            OK = "1";
            GPIOC->ODR |= GPIO_ODR_6;
        }
        SERIAL_LOG(OK);
        SERIAL_LOG("\n");

        if (OK == "0")
        {
            target_rpm = 20;
        }
        else if (OK == "1")
        {
            target_rpm = 100;
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
