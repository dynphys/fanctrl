#ifndef __MAIN_H__
#define __MAIN_H__

#include "stm32l0xx_hal.h"

#endif

extern ADC_HandleTypeDef hadc;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim21;
extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_rx;

extern uint32_t temp;
extern uint32_t i;
