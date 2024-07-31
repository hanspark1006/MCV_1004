/*
 * timer_ctrl.c
 *
 *  Created on: Jan 5, 2024
 *      Author: catsa
 */
#include "main.h"
#include "apps.h"
#include "uart_proc.h"
#include "timer_ctrl.h"

//TIM_HandleTypeDef timer_handle[MAX_PWM_CH];

//PWM_Timer nPWMTimer[MAX_PWM_CH] = {0,};
__IO int nPWMTimer[MAX_PWM_CH] = {0,};
GPIO_TypeDef *gpio_port[MAX_PWM_CH] = {CH1_OUT_GPIO_Port, CH2_OUT_GPIO_Port, CH3_OUT_GPIO_Port, CH4_OUT_GPIO_Port};
uint16_t gpio_pin[MAX_PWM_CH] = {CH1_OUT_Pin, CH2_OUT_Pin, CH3_OUT_Pin, CH4_OUT_Pin};
TIM_OC_InitTypeDef sConfigOC_;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	return ;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	return;
}

void PWM_OUT(int channel, int data)
{
	uint32_t tim_ch[MAX_PWM_CH]={TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4};

	sConfigOC_.Pulse = (uint32_t)data;
#if DAEGON_MODEL
#else
	if(HAL_TIM_PWM_Stop(&htim2, tim_ch[channel])!=HAL_OK){
		Printf("PWM Stop Error!!\r\n");
	}
#endif
	if(HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC_, tim_ch[channel]) != HAL_OK){
		Printf("PWM Set config Error!!\r\n");
	}
#if DAEGON_MODEL
#else
	if(HAL_TIM_PWM_Start(&htim2, tim_ch[channel]) != HAL_OK){
		Printf("PWM Start Error!!\r\n");
	}

	if (channel == 0){
		if ((Device.nDeviceCurrentStatus >= LCD_STATUS_MODE_LOCAL) && (Device.nDeviceCurrentStatus <= LCD_STATUS_MODE_EXT)){
			if (data == 0) DataSend2(SENSING_LED, 10);
			else DataSend2(SENSING_LED, 11);
		}
	}
#endif
}

void set_gpio_out_port(void)
{
	for(int i = 0; i < MAX_PWM_CH; i++){
		HAL_GPIO_WritePin(gpio_port[i], gpio_pin[i], GPIO_PIN_RESET);
	}

	GPIO_InitTypeDef GPIO_InitStruct = {0};

	GPIO_InitStruct.Pin = CH1_OUT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(CH1_OUT_GPIO_Port, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = CH2_OUT_Pin | CH3_OUT_Pin | CH4_OUT_Pin | GPIO_PIN_12;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(CH2_OUT_GPIO_Port, &GPIO_InitStruct);
}

void pwm_timer_disable(void)
{
	uint32_t tim_ch[MAX_PWM_CH]={TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4};

	Printf("PWM Disable!!\r\n");

	for(int i = 0; i < MAX_PWM_CH; i++){
		HAL_TIM_PWM_Stop(&htim2, tim_ch[i]);
	}
	HAL_TIM_PWM_DeInit(&htim2);
}

void strobe_timer_init(void)
{
	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};

	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 72-1; // 5us
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 10-1;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
	{
		Printf("Timer Init error tim2!!\r\n");
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
	{
		Printf("Timer Config Clock error!!tim2\r\n");
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
	{
		Printf("Timer Config Sync error!!tim2\r\n");
		Error_Handler();
	}

	set_gpio_out_port();

	HAL_TIM_Base_Start_IT(&htim1);
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_TIM_Base_Start_IT(&htim4);
}

void timer_ctrl_init(void)
{
	sConfigOC_.OCMode = TIM_OCMODE_PWM1;
	sConfigOC_.Pulse = 500;
	sConfigOC_.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC_.OCFastMode = TIM_OCFAST_ENABLE;

#if DAEGON_MODEL
	HAL_TIM_Base_Start_IT(&htim1);
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_TIM_Base_Start_IT(&htim4);
#endif
}
