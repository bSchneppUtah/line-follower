/* USER CODE BEGIN Header */
/**
	******************************************************************************
	* @file		stm32f0xx_it.c
	* @brief	 Interrupt Service Routines.
	******************************************************************************
	* @attention
	*
	* <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
	* All rights reserved.</center></h2>
	*
	* This software component is licensed by ST under BSD 3-Clause license,
	* the "License"; You may not use this file except in compliance with the
	* License. You may obtain a copy of the License at:
	*												opensource.org/licenses/BSD-3-Clause
	*
	******************************************************************************
	*/
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "led.h"
#include "timer.h"
#include "main.h"
#include "stm32f072xb.h"
#include "stm32f0xx_hal.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*					 Cortex-M0 Processor Interruption and Exception Handlers					*/ 
/******************************************************************************/
/**
	* @brief This function handles Non maskable interrupt.
	*/
void NMI_Handler(void)
{
	/* USER CODE BEGIN NonMaskableInt_IRQn 0 */

	/* USER CODE END NonMaskableInt_IRQn 0 */
	/* USER CODE BEGIN NonMaskableInt_IRQn 1 */

	/* USER CODE END NonMaskableInt_IRQn 1 */
}

void EXTI0_1_IRQHandler(void)
{
	EXTI->PR |= EXTI_PR_PIF0;	/* "ACK" the interrupt */
}

char RecvChar(USART_TypeDef *Def);
void WriteChar(USART_TypeDef *Def, char Cur);

static char Count[2];
static uint8_t Status = 0;

void WriteAsyncChar(USART_TypeDef *Def, char Cur)
{
	Def->TDR = Cur;
}

void USART3_4_IRQHandler(void)
{
	char Current = '\0';
	if ((USART3->ISR & USART_ISR_RXNE) == USART_ISR_RXNE)
	{
		Current = USART3->RDR;
	}
	USART3->ICR |= (USART_ISR_ORE | USART_ISR_RXNE);
}

void TIM2_IRQHandler(void)
{
	TIM2->SR = 0;
	/* ACK here */
}

/**
	* @brief This function handles Hard fault interrupt.
	*/
void HardFault_Handler(void)
{
	/* USER CODE BEGIN HardFault_IRQn 0 */

	/* USER CODE END HardFault_IRQn 0 */
	while (1)
	{
		/* USER CODE BEGIN W1_HardFault_IRQn 0 */
		/* USER CODE END W1_HardFault_IRQn 0 */
	}
}

/**
	* @brief This function handles System service call via SWI instruction.
	*/
void SVC_Handler(void)
{
	/* USER CODE BEGIN SVC_IRQn 0 */

	/* USER CODE END SVC_IRQn 0 */
	/* USER CODE BEGIN SVC_IRQn 1 */

	/* USER CODE END SVC_IRQn 1 */
}

/**
	* @brief This function handles Pendable request for system service.
	*/
void PendSV_Handler(void)
{
	/* USER CODE BEGIN PendSV_IRQn 0 */

	/* USER CODE END PendSV_IRQn 0 */
	/* USER CODE BEGIN PendSV_IRQn 1 */

	/* USER CODE END PendSV_IRQn 1 */
}

/**
	* @brief This function handles System tick timer.
	*/
void SysTick_Handler(void)
{
	/* USER CODE BEGIN SysTick_IRQn 0 */
	static uint8_t Val = 0;
	Val++;
	if (Val >= 200)
	{
		Val = 0;
	}

	/* USER CODE END SysTick_IRQn 0 */
	HAL_IncTick();
	/* USER CODE BEGIN SysTick_IRQn 1 */

	/* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F0xx Peripheral Interrupt Handlers																		*/
/* Add here the Interrupt Handlers for the used peripherals.									*/
/* For the available peripheral interrupt handler names,											*/
/* please refer to the startup file (startup_stm32f0xx.s).										*/
/******************************************************************************/

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
