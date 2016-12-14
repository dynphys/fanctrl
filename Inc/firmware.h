#ifndef __FIRMWARE_H__
#define __FIRMWARE_H__

#include "stm32l0xx_hal.h"
#include <stdbool.h>

#define DEFAULT_TX_TIMEOUT_MS   500 
#define DEFAULT_T_REG           250
#define DEFAULT_D_MIN           0
#define DEFAULT_D_MAX           100
#define DEFAULT_HYST            10
#define DEFAULT_START_DURATION  1000
#define DEFAULT_START_PWM       100
#define DEFAULT_UART_LENGTH     128
#define MAX_ARGS                16
#define START_DELIMITER         0x3A
#define SEPARATOR               0x2C
#define TEMP_OFFSET             17
#define TEMP_NUM                1000
#define TEMP_DEN                1365
#define MSG_UART_OK             "OK\r\n"
#define MSG_UART_NOK            "ERROR\r\n"
#define MSG_REG_ON              "Regulation enabled.\r\n"
#define MSG_REG_OFF             "Regulation disabled.\r\n"


#define MIN_START_DURATION      0
#define MAX_START_DURATION      60000
#define MIN_CHANNEL             1
#define MAX_CHANNEL             4
#define MIN_T_REG               0
#define MAX_T_REG               1000
#define MIN_PWM                 0
#define MAX_PWM                 100
#define MIN_HYST                5
#define MAX_HYST                50

/* Flash map */
#define __FLASH_start_pwm       64
#define __FLASH_start_duration  68
#define __FLASH_reg_enable      72

typedef enum {
	OFF=0,
	ON
}States_t;

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
	GPIO_TypeDef 	* GPIO_Port;
	GPIO_PinState   PinState;
	uint16_t        GPIO_Pin;
} LED_t;

typedef struct {
	__IO uint32_t * pDutyCycleReg;
	__IO uint32_t * pPeriodReg;
	uint32_t        DutyCycle;
} PWM_t;

typedef struct {
	char command[DEFAULT_UART_LENGTH];
	char arguments[MAX_ARGS][DEFAULT_UART_LENGTH];	
} UART_message_t;

typedef struct {
	PWM_t         PWM_channel;
	RegStates_t   RegState;
	LED_t         Red_Led;
	LED_t         Green_Led;
	uint32_t      Profile_Id;
	uint32_t      T_reg;
	uint32_t      D_min;
	uint32_t      D_max;
	uint32_t      Hys;
} RegProfile_t;


/***** Functions prototypes *****/
HAL_StatusTypeDef   EEPROM_WriteWord(uint32_t address, uint32_t data);
HAL_StatusTypeDef   EEPROM_WriteByte(uint32_t address, uint32_t data);
uint32_t            Firmware_Init(void);
uint32_t            Flash_Init(void);
uint32_t            LM35_Convert(void);
uint32_t            PWM_Set(RegProfile_t * channel, uint32_t duty_cycle);
uint32_t            Set_Regulation(States_t state);
uint32_t            PWM_Update(RegProfile_t * channel, uint32_t temp);
uint32_t            Param_Backup(RegProfile_t * profile, uint32_t channel);
uint32_t            Param_Check(void);
uint32_t            Param_Reset(void);
uint32_t            Param_Restore(void);
uint32_t            UART_Send(char * pBuffer, uint32_t length);
uint32_t            CheckValue(uint32_t val, uint32_t min, uint32_t max);
uint32_t            CheckNArgs(UART_message_t * message, uint32_t nargs);
uint32_t            LED(RegProfile_t * channel, LedColors_t color, States_t state);
uint32_t            LED_All(LedColors_t color, States_t state);
uint32_t            DustOff(void);
uint32_t            EEPROM_ReadWord(uint32_t address);
uint32_t            UART_Execute(void);
uint32_t            cmd_Set(void);
uint32_t            cmd_Trace(void);
uint32_t            cmd_Start(void);
uint32_t            cmd_Hyst(void);
uint32_t            cmd_Config(void);
void                HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);
void                HAL_UART_RxCpltCallback (UART_HandleTypeDef * huart);
void                HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

#endif

/***** Extern variables *****/
extern UART_message_t   g_mes;
extern RegProfile_t     g_RegCh1;
extern RegProfile_t     g_RegCh2;
extern RegProfile_t     g_RegCh3;
extern RegProfile_t     g_RegCh4;
extern uint32_t         g_timer_1ms;
extern uint32_t         g_lm35_cv_cmplt;
extern uint32_t         g_UART_Buffer_Index;
extern uint8_t          g_UART_Buffer;
extern uint8_t          g_UART_Buffer_Array[DEFAULT_UART_LENGTH];
extern bool             g_UART_Message_Ready;
extern bool             g_trace_enable;
extern bool             g_usrbtn;
extern bool             g_reg_enable;


