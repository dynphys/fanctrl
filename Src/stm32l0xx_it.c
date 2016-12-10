/**
  ******************************************************************************
  * Copyright 2016 Lucas Galand
	*
  * Licensed under the Apache License, Version 2.0 (the "License");
  * you may not use this file except in compliance with the License.
  * You may obtain a copy of the License at
	*
  *     http://www.apache.org/licenses/LICENSE-2.0
	*
	*	Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an "AS IS" BASIS,
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  ******************************************************************************
*/
/* Includes ------------------------------------------------------------------*/
#include "stm32l0xx_hal.h"
#include "stm32l0xx.h"
#include "stm32l0xx_it.h"

/* USER CODE BEGIN 0 */
#include "firmware.h"
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern ADC_HandleTypeDef hadc;
extern TIM_HandleTypeDef htim21;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern UART_HandleTypeDef huart2;

/******************************************************************************/
/*            Cortex-M0+ Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

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

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32L0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32l0xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles EXTI line 4 to 15 interrupts.
*/
void EXTI4_15_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI4_15_IRQn 0 */

  /* USER CODE END EXTI4_15_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);
  /* USER CODE BEGIN EXTI4_15_IRQn 1 */

  /* USER CODE END EXTI4_15_IRQn 1 */
}

/**
* @brief This function handles DMA1 channel 4, channel 5, channel 6 and channel 7 interrupts.
*/
void DMA1_Channel4_5_6_7_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel4_5_6_7_IRQn 0 */

  /* USER CODE END DMA1_Channel4_5_6_7_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart2_rx);
  /* USER CODE BEGIN DMA1_Channel4_5_6_7_IRQn 1 */

  /* USER CODE END DMA1_Channel4_5_6_7_IRQn 1 */
}

/**
* @brief This function handles ADC1, COMP1 and COMP2 interrupts (COMP interrupts through EXTI lines 21 and 22).
*/
void ADC1_COMP_IRQHandler(void)
{
  /* USER CODE BEGIN ADC1_COMP_IRQn 0 */

  /* USER CODE END ADC1_COMP_IRQn 0 */
  HAL_ADC_IRQHandler(&hadc);
  /* USER CODE BEGIN ADC1_COMP_IRQn 1 */

  /* USER CODE END ADC1_COMP_IRQn 1 */
}

/**
* @brief This function handles TIM21 global interrupt.
*/
void TIM21_IRQHandler(void)
{
  /* USER CODE BEGIN TIM21_IRQn 0 */
	g_timer_1ms++;
  /* USER CODE END TIM21_IRQn 0 */
  HAL_TIM_IRQHandler(&htim21);
  /* USER CODE BEGIN TIM21_IRQn 1 */

  /* USER CODE END TIM21_IRQn 1 */
}

/**
* @brief This function handles USART2 global interrupt / USART2 wake-up interrupt through EXTI line 26.
*/
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */

  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */

  /* USER CODE END USART2_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
