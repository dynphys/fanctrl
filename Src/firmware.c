#include <stdlib.h>
#include <string.h>
#include "firmware.h"
#include "main.h"

/***** Globlal variables *****/
UART_message_t 		g_mes;
uint32_t 					g_timer_1ms=0;
uint32_t 					g_lm35_cv_cmplt=0;
uint32_t 					g_UART_Buffer_Index=0;
uint8_t 					g_UART_Buffer=0;
uint8_t 					g_UART_Buffer_Array[DEFAULT_UART_LENGTH];
bool 							g_UART_Message_Ready = false;
bool 							g_trace_enable = false;
bool 							g_usrbtn = false;

/* Regulation profiles */
RegProfile_t 			g_RegCh1;
RegProfile_t 			g_RegCh2;
RegProfile_t 			g_RegCh3;
RegProfile_t 			g_RegCh4;

uint32_t Firmware_Init(void) {
	
	//Init timers
	HAL_TIM_Base_Start_IT(&htim21);
	HAL_TIM_Base_Start_IT(&htim2);
	
	//Start PWM channels
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
	
	//Init USART2
	HAL_UART_Receive_DMA(&huart2, &g_UART_Buffer, 1);
	
	//Init regulation profile
	g_RegCh1.Profile_Id = 0;
	g_RegCh1.PWM_channel.pDutyCycleReg=&TIM2->CCR3;
	g_RegCh1.PWM_channel.pPeriodReg = &TIM2->ARR;
	g_RegCh1.RegState=UNKNOWN;
	g_RegCh1.PWM_channel.DutyCycle=0;
	g_RegCh1.T_reg=DEFAULT_T_REG;
	g_RegCh1.D_min=DEFAULT_D_MIN;
	g_RegCh1.D_max=DEFAULT_D_MAX;
	g_RegCh1.Hys=DEFAULT_HYS;
	g_RegCh1.Red_Led.GPIO_Port=GPIOA;
	g_RegCh1.Red_Led.GPIO_Pin=GPIO_PIN_10;
	g_RegCh1.Green_Led.GPIO_Port=GPIOA;
	g_RegCh1.Green_Led.GPIO_Pin=GPIO_PIN_9;
	
	g_RegCh2.Profile_Id = 1;
	g_RegCh2.PWM_channel.pDutyCycleReg=&TIM2->CCR4;
	g_RegCh2.PWM_channel.pPeriodReg = &TIM2->ARR;
	g_RegCh2.RegState=UNKNOWN;
	g_RegCh2.PWM_channel.DutyCycle=0;
	g_RegCh2.T_reg=DEFAULT_T_REG;
	g_RegCh2.D_min=DEFAULT_D_MIN;
	g_RegCh2.D_max=DEFAULT_D_MAX;
	g_RegCh2.Hys=DEFAULT_HYS;
	g_RegCh2.Red_Led.GPIO_Port=GPIOB;
	g_RegCh2.Red_Led.GPIO_Pin=GPIO_PIN_7;
	g_RegCh2.Green_Led.GPIO_Port=GPIOA;
	g_RegCh2.Green_Led.GPIO_Pin=GPIO_PIN_12;
	
	g_RegCh3.Profile_Id = 2;
	g_RegCh3.PWM_channel.pDutyCycleReg=&TIM2->CCR2;
	g_RegCh3.PWM_channel.pPeriodReg = &TIM2->ARR;
	g_RegCh3.RegState=UNKNOWN;
	g_RegCh3.PWM_channel.DutyCycle=0;
	g_RegCh3.T_reg=DEFAULT_T_REG;
	g_RegCh3.D_min=DEFAULT_D_MIN;
	g_RegCh3.D_max=DEFAULT_D_MAX;
	g_RegCh3.Hys=DEFAULT_HYS;
	g_RegCh3.Red_Led.GPIO_Port=GPIOB;
	g_RegCh3.Red_Led.GPIO_Pin=GPIO_PIN_1;
	g_RegCh3.Green_Led.GPIO_Port=GPIOB;
	g_RegCh3.Green_Led.GPIO_Pin=GPIO_PIN_6;
	
	g_RegCh4.Profile_Id = 3;
	g_RegCh4.PWM_channel.pDutyCycleReg=&TIM2->CCR1;
	g_RegCh4.PWM_channel.pPeriodReg = &TIM2->ARR;
	g_RegCh4.RegState=UNKNOWN;
	g_RegCh4.PWM_channel.DutyCycle=0;
	g_RegCh4.T_reg=DEFAULT_T_REG;
	g_RegCh4.D_min=DEFAULT_D_MIN;
	g_RegCh4.D_max=DEFAULT_D_MAX;
	g_RegCh4.Hys=DEFAULT_HYS;
	g_RegCh4.Red_Led.GPIO_Port=GPIOA;
	g_RegCh4.Red_Led.GPIO_Pin=GPIO_PIN_11;
	g_RegCh4.Green_Led.GPIO_Port=GPIOA;
	g_RegCh4.Green_Led.GPIO_Pin=GPIO_PIN_8;
	
	return 0;
}

uint32_t PWM_Update(RegProfile_t * channel, uint32_t temp){
	int ret = 0;
	
	//LOW to HIGH
	if(temp>(channel->T_reg)+(channel->Hys) && channel->RegState==LOW){
		ret=PWM_Set(channel, channel->D_max);
		if(ret>0) return 1;
		
		ret=LED(channel,GREEN,OFF);
		if(ret>0) return 1;
		
		ret=LED(channel,RED,ON);
		if(ret>0) return 1;
		
		channel->RegState=HIGH;
	}
	
	//HIGH to LOW
	if(temp<(channel->T_reg)-(channel->Hys) && channel->RegState==HIGH){
		ret=PWM_Set(channel, channel->D_min);
		if(ret>0) return 1;
		
		ret=LED(channel,RED,OFF);
		if(ret>0) return 1;
		
		ret=LED(channel,GREEN,ON);
		if(ret>0) return 1;
		
		channel->RegState=LOW;
	}
	
	//UNKNOWN state
	if(channel->RegState==UNKNOWN){	
		ret=PWM_Set(channel, channel->D_max);
		if(ret>0) return 1;
		
		ret=LED(channel,GREEN,OFF);
		if(ret>0) return 1;
		
		ret=LED(channel,RED,ON);
		if(ret>0) return 1;
		
		channel->RegState=HIGH;
	}
	
	return ret;
}

uint32_t PWM_Set(RegProfile_t * channel, uint32_t duty_cycle){
	uint32_t ret, period = 0;
	
	ret = CheckValue(duty_cycle,0,100);
	if(ret>0) return 1;
	
	period = *(channel->PWM_channel.pPeriodReg);
	*(channel->PWM_channel.pDutyCycleReg)=(duty_cycle*period)/100;
	channel->PWM_channel.DutyCycle=duty_cycle;
	
	return 0;
}

//Return temperature (x0.1 °C)
uint32_t LM35_Convert(void) {
	uint32_t ret;
	uint32_t tickstart;
	
	g_lm35_cv_cmplt=0;
	HAL_ADC_Start_IT(&hadc);
	
	tickstart = HAL_GetTick();
	while(g_lm35_cv_cmplt==0 && HAL_GetTick()-tickstart<100);
	
	if(g_lm35_cv_cmplt==1)
	{
		ret = HAL_ADC_GetValue(&hadc);
	}
	else
	{
		ret=0;
	}
	
	ret = TEMP_NUM * ret;
	ret = ret / TEMP_DEN;
	ret = ret - TEMP_OFFSET;
	
	HAL_ADC_Stop_IT(&hadc);
	
	return ret;
}

uint32_t Param_Backup(RegProfile_t * profile, uint32_t channel) {
	uint32_t ret = 0;
	
	EEPROM_WriteWord(16*channel, profile->T_reg);
	EEPROM_WriteWord(16*channel+4, profile->D_min);
	EEPROM_WriteWord(16*channel+8, profile->D_max);
	EEPROM_WriteWord(16*channel+12, profile->Hys);
	
	return ret;
}

uint32_t Param_Restore(void) {
	uint32_t i=0;
	RegProfile_t * pArr[4] = {&g_RegCh1,&g_RegCh2,&g_RegCh3,&g_RegCh4};
	
	//Load parameters from EEPROM
	for(i=0;i<4;i++)
	{
		pArr[i]->T_reg = EEPROM_ReadWord(16*i);
		pArr[i]->D_min = EEPROM_ReadWord(16*i+4);
		pArr[i]->D_max = EEPROM_ReadWord(16*i+8);
		pArr[i]->Hys = EEPROM_ReadWord(16*i+12);
	}	
	
	return 0;
}

uint32_t UART_Send(char * pBuffer, uint32_t length) {
	uint32_t ret, time = 0;
	
	ret=HAL_UART_Transmit_IT(&huart2,(uint8_t *)pBuffer,length);
	
	time = HAL_GetTick();
	while(HAL_UART_GetState(&huart2)==HAL_UART_STATE_BUSY_TX_RX && HAL_GetTick()-time<DEFAULT_TX_TIMEOUT_MS);
	
	return ret;
}


uint32_t CheckValue(uint32_t val, uint32_t min, uint32_t max) {
	uint32_t ret = 0;
	
	if(val>=min && val<=max)
	{
		ret=0;
	}
	else
	{
		ret=1;
	}
	
	return ret;
}

uint32_t CheckNArgs(UART_message_t * message, uint32_t nargs){
	uint32_t ret,i = 0;
	
	ret=0;
	for(i=0;i<MAX_ARGS;i++){
		if(strlen(message->arguments[i])>0){
			ret++;
		}
	}
	
	if(ret!=nargs){
		return 1;
	}

	return 0;
}

uint32_t LED(RegProfile_t * channel, LedColors_t color, LedStates_t state) {
	uint32_t ret=0;	
	GPIO_PinState gps;
	
	if(state==OFF){
		gps=GPIO_PIN_RESET;
	}
	else if(state==ON){
		gps=GPIO_PIN_SET;
	}
	else{
		return 1;
	}
	
	if(color==RED){
		HAL_GPIO_WritePin(channel->Red_Led.GPIO_Port, channel->Red_Led.GPIO_Pin, gps);
	}
	else if(color==GREEN){
		HAL_GPIO_WritePin(channel->Green_Led.GPIO_Port, channel->Green_Led.GPIO_Pin, gps);
	}
	else{
		return 1;
	}

	return ret;
}

uint32_t DustOff(void) {
	uint32_t start_pwm, start_duration, ret,i;
	RegProfile_t * pArr[4] = {&g_RegCh1,&g_RegCh2,&g_RegCh3,&g_RegCh4};
	
	start_pwm = EEPROM_ReadWord(__FLASH_start_pwm);
	start_duration = EEPROM_ReadWord(__FLASH_start_duration);
	
	//Set channels
	ret=0;
	i=0;
	for(i=0;i<4;i++){
		ret += PWM_Set(pArr[i],start_pwm);
	}
	if(ret>0) return 1;

	//Wait and display led pattern =D
	start_duration=start_duration/200;
	for(i=0;i<start_duration;i++) {
		LED(&g_RegCh1,GREEN,ON);
		LED(&g_RegCh1,RED,OFF);
		LED(&g_RegCh2,GREEN,OFF);
		LED(&g_RegCh2,RED,OFF);
		LED(&g_RegCh3,GREEN,OFF);
		LED(&g_RegCh3,RED,OFF);
		LED(&g_RegCh4,GREEN,OFF);
		LED(&g_RegCh4,RED,ON);
		
		HAL_Delay(50);
		
		LED(&g_RegCh1,GREEN,OFF);
		LED(&g_RegCh1,RED,OFF);
		LED(&g_RegCh2,GREEN,ON);
		LED(&g_RegCh2,RED,OFF);
		LED(&g_RegCh3,GREEN,OFF);
		LED(&g_RegCh3,RED,ON);
		LED(&g_RegCh4,GREEN,OFF);
		LED(&g_RegCh4,RED,OFF);
		
		HAL_Delay(50);
		
		LED(&g_RegCh1,GREEN,OFF);
		LED(&g_RegCh1,RED,OFF);
		LED(&g_RegCh2,GREEN,OFF);
		LED(&g_RegCh2,RED,ON);
		LED(&g_RegCh3,GREEN,ON);
		LED(&g_RegCh3,RED,OFF);
		LED(&g_RegCh4,GREEN,OFF);
		LED(&g_RegCh4,RED,OFF);
		
		HAL_Delay(50);
		
		LED(&g_RegCh1,GREEN,OFF);
		LED(&g_RegCh1,RED,ON);
		LED(&g_RegCh2,GREEN,OFF);
		LED(&g_RegCh2,RED,OFF);
		LED(&g_RegCh3,GREEN,OFF);
		LED(&g_RegCh3,RED,OFF);
		LED(&g_RegCh4,GREEN,ON);
		LED(&g_RegCh4,RED,OFF);
		
		HAL_Delay(50);
	}
	
	return 0;
}

uint32_t EEPROM_ReadWord(uint32_t address) {
	address += 0x08080000;
	return *(uint32_t *)address;
}

HAL_StatusTypeDef EEPROM_WriteWord(uint32_t address, uint32_t data)
 {
    HAL_StatusTypeDef  status;
    address = address + 0x08080000;
    HAL_FLASHEx_DATAEEPROM_Unlock();
    status = HAL_FLASHEx_DATAEEPROM_Program(TYPEPROGRAMDATA_WORD, address, data);
    HAL_FLASHEx_DATAEEPROM_Lock();
    
	 return status;
}
 
HAL_StatusTypeDef EEPROM_WriteByte(uint32_t address, uint32_t data)
 {
    HAL_StatusTypeDef  status;
    address = address + 0x08080000;
    HAL_FLASHEx_DATAEEPROM_Unlock();
    status = HAL_FLASHEx_DATAEEPROM_Program(TYPEPROGRAMDATA_BYTE, address, data);
    HAL_FLASHEx_DATAEEPROM_Lock();
    
	 return status;
}
 
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
		g_lm35_cv_cmplt=1;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
		if(GPIO_Pin == GPIO_PIN_4) {
			g_usrbtn = true;
		}
}

uint32_t UART_Execute(void) {
	uint32_t ret = 1;
	
	//SET
	if(strcmp(g_mes.command,"SET")==0){
		ret = cmd_Set();
	}
	
	if(strcmp(g_mes.command,"TRACE")==0){
		ret = cmd_Trace();
	}
	
	if(strcmp(g_mes.command,"START")==0){
		ret = cmd_Start();
	}
	
	if(strcmp(g_mes.command,"HYST")==0){
		ret = cmd_Hyst();
	}
	
	if(strcmp(g_mes.command,"CONFIG")==0){
		ret = cmd_Config();
	}
	
	return ret;
}

//Firmware commands
uint32_t cmd_Set(void){
	uint32_t ret = 0;
	uint32_t channel, temp, pwm_min, pwm_max;
	RegProfile_t * pArr[4] = {&g_RegCh1,&g_RegCh2,&g_RegCh3,&g_RegCh4};
	
  channel = atoi(g_mes.arguments[0]);
	temp = atoi(g_mes.arguments[1]);
	pwm_min = atoi(g_mes.arguments[2]);
	pwm_max = atoi(g_mes.arguments[3]);
	
	ret=0;
	ret+=CheckNArgs(&g_mes,4);
	if(ret>0) return 1;
	
	ret=0;
	ret+=CheckValue(channel,1,4);
	ret+=CheckValue(temp,0,999);
	ret+=CheckValue(pwm_min,0,100);
	ret+=CheckValue(pwm_max,0,100);
	if(ret>0) return 1;
	
	//From 1 to 4 in uart but from 0 to 3 in flash
	channel--;
	
	pArr[channel]->T_reg = temp;
	pArr[channel]->D_min = pwm_min;
	pArr[channel]->D_max = pwm_max;
	pArr[channel]->RegState = UNKNOWN;
	
	ret+=Param_Backup(pArr[channel],channel);
	if(ret>0) return 1;
	
	return 0;
}

uint32_t cmd_Trace(void) {
	g_trace_enable = true;
	return 0;
}

uint32_t cmd_Start(void) {
	uint32_t start_pwm, start_duration, ret = 0;
	
	start_pwm=atoi(g_mes.arguments[0]);
	start_duration=atoi(g_mes.arguments[1]);
	
	ret=0;
	ret+=CheckValue(start_pwm,0,100);
	ret+=CheckValue(start_duration,0,60000);
	
	if(ret>0) return 1;
	
	ret=0;
	ret+=EEPROM_WriteWord(__FLASH_start_pwm, start_pwm);
	ret+=EEPROM_WriteWord(__FLASH_start_duration, start_duration);
	
	if(ret>0) return 1;
	
	return 0;
}

uint32_t cmd_Hyst(void) {
	uint32_t channel, hyst_value, ret = 0;
	RegProfile_t * pArr[4] = {&g_RegCh1,&g_RegCh2,&g_RegCh3,&g_RegCh4};
	
	channel=atoi(g_mes.arguments[0]);
	hyst_value=atoi(g_mes.arguments[1]);
	
	ret=0;
	ret+=CheckValue(channel,1,4);
	ret+=CheckValue(hyst_value,0,50);
	
	if(ret>0) return 1;
	
	//From 1 to 4 in uart but from 0 to 3 in flash
	channel--;
	
	ret=0;
	ret+=EEPROM_WriteWord(16*channel + 12, hyst_value);
	
	if(ret>0) return 1;
	
	pArr[channel]->Hys = hyst_value;
	
	return 0;
}

uint32_t cmd_Config(void) {
	uint32_t ret, i = 0;
	char txBuffer[DEFAULT_UART_LENGTH];
	RegProfile_t * pArr[4] = {&g_RegCh1,&g_RegCh2,&g_RegCh3,&g_RegCh4};
	
	for(i=0;i<4;i++) {
		memset(txBuffer,0,sizeof(txBuffer));
		sprintf(txBuffer,"CONFIG CH%d: %d, %d, %d, %d\r\n",i+1,pArr[i]->T_reg, pArr[i]->D_min, pArr[i]->D_max, pArr[i]->Hys);
		
		ret=0;
		ret+=UART_Send(txBuffer,strlen(txBuffer));
		if(ret>0) return 1;
		//HAL_Delay(10);
	}
	
	return 0;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef * huart) {	
	uint8_t i, arg_index=0;
	uint8_t start_pos, stop_pos=0;
	bool message_available, end_reached = false;
	
	//Sample characters
	if(g_UART_Buffer==START_DELIMITER){
		g_UART_Buffer_Index=0;
		memset(g_UART_Buffer_Array,0,sizeof(g_UART_Buffer_Array));
		g_UART_Buffer_Array[g_UART_Buffer_Index]=g_UART_Buffer;
		g_UART_Buffer_Index++;
	}
	else if(g_UART_Buffer_Array[0]==START_DELIMITER && g_UART_Buffer==0x0D){
		g_UART_Buffer_Array[g_UART_Buffer_Index]=g_UART_Buffer;
		g_UART_Buffer_Index=0;
		g_UART_Message_Ready = true;
		message_available = true;
	}
	else{
		g_UART_Buffer_Array[g_UART_Buffer_Index]=g_UART_Buffer;
		g_UART_Buffer_Index++;
	}
	
	//Escape sequence
	if(g_UART_Buffer==0x0D){
		g_trace_enable=false;
	}
	
	//Extract message
	if(message_available==true){
		//Extract command
		i=0;
		memset(g_mes.command,0,DEFAULT_UART_LENGTH);
		for(i=0;i<MAX_ARGS;i++){
			memset(g_mes.arguments[i],0,DEFAULT_UART_LENGTH);
		}
		
		i=0;
		while(g_UART_Buffer_Array[i]!=SEPARATOR && g_UART_Buffer_Array[i]!=0x0D && i<sizeof(g_UART_Buffer_Array)){
			i++;
		}
		memcpy(g_mes.command,(char *)(g_UART_Buffer_Array+1),i-1);
		
		//Extract arguments
		arg_index=0;
		start_pos=i+1;
		stop_pos=i+1;

		while(stop_pos<sizeof(g_UART_Buffer_Array)-1 && end_reached==false){
			memset(g_mes.arguments[arg_index],0,sizeof(g_mes.arguments[arg_index]));
			while(g_UART_Buffer_Array[stop_pos]!=SEPARATOR && g_UART_Buffer_Array[stop_pos]!=0x0D && stop_pos<sizeof(g_UART_Buffer_Array)){
				stop_pos++;
			}
			
			if(g_UART_Buffer_Array[stop_pos]==0x0D){
				end_reached=true;
			}
			
			memcpy(g_mes.arguments[arg_index],(char *)(g_UART_Buffer_Array+start_pos),stop_pos-start_pos);
			
			start_pos=stop_pos+1;
			stop_pos=start_pos;
			arg_index++;
		}
		
		memset(g_UART_Buffer_Array,0,sizeof(g_UART_Buffer_Array));
		message_available=false;
	}
}
