/*
 * uart_serial.c
 *
 *  Created on: Jan 5, 2024
 *      Author: catsa
 */
#include "main.h"
#include "stm32f1xx_it.h"
#include "apps.h"
#include "uart_proc.h"
#include "timer_ctrl.h"

DEVICE Device;

uint8_t bSW[MAX_SW_PORT] = {1,};
uint8_t bEX_INPUT[MAX_SW_PORT] = {1,};

void check_gpio_input(void)
{
	unsigned int status = 0x00;
	int flag = 0, i;
	GPIO_TypeDef *port[]={SW1_GPIO_Port, SW2_GPIO_Port, SW3_GPIO_Port, SW4_GPIO_Port};
	uint16_t pin[]={SW1_Pin, SW2_Pin, SW3_Pin, SW4_Pin};

	// ?�려 ?�트??체크?�기
	for(i = 0; i < MAX_SW_PORT; i++){
		status = HAL_GPIO_ReadPin(port[i], pin[i]);
		if (bSW[i] != status){
			bSW[i] = status;
		}
		if(bSW[i] == GPIO_PIN_RESET){
			if(i==2)
				flag |= 0x02;
			if(i==3)
				flag |= 0x01;
		}
	}

	if (Device.nDeviceNo == 0){
		flag++;
		Device.nDeviceNo = flag;
	}
}

void check_ext_input(void)
{
	unsigned int status = 0x00;
	uint8_t i;
	GPIO_TypeDef *port[]={EX_INPUT1_GPIO_Port, EX_INPUT2_GPIO_Port, EX_INPUT3_GPIO_Port, EX_INPUT4_GPIO_Port};
	uint16_t pin[]={EX_INPUT1_Pin, EX_INPUT2_Pin, EX_INPUT3_Pin, EX_INPUT4_Pin};

	for(i = 0; i < MAX_SW_PORT; i++){
		status = HAL_GPIO_ReadPin(port[i], pin[i]);
		if (bEX_INPUT[i] != status){
			bEX_INPUT[i] = status;
			if (bEX_INPUT[i] == GPIO_PIN_RESET){
				PWM_OUT(i, Device.nPWM[i]);
			}else{
				PWM_OUT(i, 0);
			}
		}
	}
}

#if 0
#define TOOGLE_MS	500 // 500ms
void OP_LED(void)
{
	static uint32_t toogle_time = 0, start_time = 0;

	if(toogle_time < (HAL_GetTick()-start_time)){
		start_time = HAL_GetTick();
		toogle_time = TOOGLE_MS+HAL_GetTickFreq();
		HAL_GPIO_TogglePin(OPLED_GPIO_Port, OPLED_Pin);
	}
}

void OP_LED(void)   <-- stm32f1xx_hal.c
{
	static uint32_t toogle_time = 0, start_time = 0;
	uint32_t odr;

	if(toogle_time < (uwTick-start_time)){
		start_time = uwTick;
		toogle_time = TOOGLE_MS+uwTickFreq;
		//HAL_GPIO_TogglePin(OPLED_GPIO_Port, OPLED_Pin);
		  odr = GPIOC->ODR;
		  GPIOC->BSRR = ((odr & GPIO_PIN_13) << 16u) | (~odr & GPIO_PIN_13);
	}
}
#else
extern void OP_LED(void);
#endif
void apps_proc(void)
{
//	uint8_t old_status = 0;

	Serial_ResetRequest();
	check_ext_input();

	while(1){
		OP_LED();
#if 0
		if(Device.nDeviceCurrentStatus != old_status){
			Printf("Status[%d]", Device.nDeviceCurrentStatus);
			old_status = Device.nDeviceCurrentStatus;
		}
#endif

		if(Device.nDeviceCurrentStatus == LCD_STATUS_MODE_EXT){
			check_ext_input();
		}

		SerialRecvData();
		SerialRecvData2();

		HAL_IWDG_Refresh(&hiwdg);
	}
}

void apps_dev_init(void)
{
	Device.nDeviceNo = 0x00;
	Device.nDeviceCurrentStatus = 0;
	Device.nDeviceCurrentStatus_old = -1;
	setCurDeviceStatus();

	for (int i = 0; i < 4; i++){
		Device.nPWM[i]=0;
		Device.nDELAY[i]=0;
		Device.nPULSE[i]=100;
		Device.nEDGE[i]=1;
		Device.nSerialPWM[i]=0xff;
	}

	check_gpio_input();

	for(int i = 0; i < MAX_PWM_CH; i++){
		PWM_OUT(i, 0);
	}
}
