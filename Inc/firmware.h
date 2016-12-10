#ifndef __FIRMWARE_H__
#define __FIRMWARE_H__

#include "stm32l0xx_hal.h"
#include <stdbool.h>

#define DEFAULT_TX_TIMEOUT_MS		500 
#define DEFAULT_T_REG 					250
#define DEFAULT_D_MIN 					0
#define DEFAULT_D_MAX 					100
#define DEFAULT_HYS 						10
#define DEFAULT_UART_LENGTH 		128
#define MAX_ARGS								16
#define START_DELIMITER					0x3A
#define SEPARATOR								0x2C

/* Flash map */
#define __FLASH_start_pwm				64
#define __FLASH_start_duration	68

typedef enum {
	OFF=0,
	ON,
}LedStates_t;

typedef enum {
	RED=0,
	GREEN,
}LedColors_t;

typedef enum {
	LOW=0,
	HIGH,
	UNKNOWN,
}RegStates_t;

typedef struct {
	GPIO_TypeDef * GPIO_Port;
	uint16_t GPIO_Pin;
	GPIO_PinState PinState;
} LED_t;

typedef struct {
	__IO uint32_t * pDutyCycleReg;
	__IO uint32_t * pPeriodReg;
	uint32_t 				DutyCycle;
} PWM_t;

typedef struct {
	char command[DEFAULT_UART_LENGTH];
	char arguments[MAX_ARGS][DEFAULT_UART_LENGTH];	
} UART_message_t;

typedef struct {
	uint32_t Profile_Id;
	PWM_t PWM_channel;
	RegStates_t RegState;
	uint32_t T_reg;
	uint32_t D_min;
	uint32_t D_max;
	uint32_t Hys;
	LED_t Red_Led;
	LED_t Green_Led;
} RegProfile_t;

uint32_t 						Firmware_Init(void);
uint32_t 						LM35_Convert(void);
uint32_t 						PWM_Set(RegProfile_t * channel, uint32_t duty_cycle);
uint32_t 						PWM_Update(RegProfile_t * channel, uint32_t temp);

uint32_t 						Param_Backup(RegProfile_t * profile, uint32_t channel);
uint32_t 						Param_Restore(void);
uint32_t 						UART_Send(char * pBuffer, uint32_t length);
uint32_t 						CheckValue(uint32_t val, uint32_t min, uint32_t max);
uint32_t 						CheckNArgs(UART_message_t * message, uint32_t nargs);
uint32_t 						LED(RegProfile_t * channel, LedColors_t color, LedStates_t state);
uint32_t 						DustOff(void);
uint32_t 						EEPROM_ReadWord(uint32_t address);
HAL_StatusTypeDef 	EEPROM_WriteWord(uint32_t address, uint32_t data);
HAL_StatusTypeDef 	EEPROM_WriteByte(uint32_t address, uint32_t data);
void 								HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);
uint32_t 						UART_Execute(void);

uint32_t cmd_Set(void);
uint32_t cmd_Trace(void);
uint32_t cmd_Start(void);
uint32_t cmd_Hyst(void);
uint32_t cmd_Config(void);

void HAL_UART_RxCpltCallback (UART_HandleTypeDef * huart);

#endif

extern uint32_t g_timer_1ms;
extern uint32_t g_lm35_cv_cmplt;

extern UART_message_t g_mes;
extern uint32_t g_UART_Buffer_Index;
extern uint8_t g_UART_Buffer;
extern uint8_t g_UART_Buffer_Array[DEFAULT_UART_LENGTH];
extern bool g_UART_Message_Ready;
extern bool g_trace_enable;

extern RegProfile_t g_RegCh1;
extern RegProfile_t g_RegCh2;
extern RegProfile_t g_RegCh3;
extern RegProfile_t g_RegCh4;

