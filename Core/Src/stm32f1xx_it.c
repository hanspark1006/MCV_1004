/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "timer_ctrl.h"
#include "uart_proc.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */
enum{
	DIMMING_MODE,
	STROBE_MODE,
	STROBE_TEST
};
/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#if 0
#define TIMER_UNIT		1  // 10us
#define INIT_DELAY_MS	10000001
#endif
#define PULSE_OFFSET	0
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
static int nStrobe = 0;
static int nStrobeTestCount = 25001;
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
/* USER CODE BEGIN EV */
extern TIM_OC_InitTypeDef sConfigOC_;
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
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
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
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
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line0 interrupt.
  */
void EXTI0_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_IRQn 0 */
	if(Device.nDeviceCurrentStatus == LCD_STATUS_MODE_STROBE_NORMAL){
		if(Device.nDELAY[0] == 0){
#if DAEGON_MODEL
			//PWM_OUT(0, Device.nPWM[0]);
			//sConfigOC_.Pulse = Device.nPWM[0];
			//HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC_, TIM_CHANNEL_1);
			TIM2->CCR1 = Device.nPWM[0];
#else
			GPIOA->BSRR = GPIO_PIN_15;
#endif
			nPWMTimer[0] = 10000001;
		}else{
			nPWMTimer[0] = Device.nDELAY[0]+10000000;
		}
	}
#if 0
  /* USER CODE END EXTI0_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(EX_INPUT1_Pin);
  /* USER CODE BEGIN EXTI0_IRQn 1 */
#endif
	if(__HAL_GPIO_EXTI_GET_IT(EX_INPUT1_Pin) != 0x00u){
		__HAL_GPIO_EXTI_CLEAR_IT(EX_INPUT1_Pin);
	}
  /* USER CODE END EXTI0_IRQn 1 */
}

/**
  * @brief This function handles EXTI line1 interrupt.
  */
void EXTI1_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI1_IRQn 0 */
	if(Device.nDeviceCurrentStatus == LCD_STATUS_MODE_STROBE_NORMAL){
		if(Device.nDELAY[1] == 0){
			GPIOB->BSRR = GPIO_PIN_3;
#if DAEGON_MODEL
			//PWM_OUT(1, Device.nPWM[1]);
			//sConfigOC_.Pulse = Device.nPWM[1];
			//HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC_, TIM_CHANNEL_2);
			TIM2->CCR2 = Device.nPWM[1];
#else
			GPIOB->BSRR = GPIO_PIN_3;
#endif
			nPWMTimer[1] = 10000001;
		}else{
			nPWMTimer[1] = Device.nDELAY[1]+10000000;
		}
	}
#if 0
  /* USER CODE END EXTI1_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(EX_INPUT2_Pin);
  /* USER CODE BEGIN EXTI1_IRQn 1 */
#endif
	if(__HAL_GPIO_EXTI_GET_IT(EX_INPUT2_Pin) != 0x00u){
		__HAL_GPIO_EXTI_CLEAR_IT(EX_INPUT2_Pin);
	}
  /* USER CODE END EXTI1_IRQn 1 */
}

/**
  * @brief This function handles EXTI line4 interrupt.
  */
void EXTI4_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI4_IRQn 0 */
	if(Device.nDeviceCurrentStatus == LCD_STATUS_MODE_STROBE_NORMAL){
		if(Device.nDELAY[3] == 0){
#if DAEGON_MODEL
			TIM2->CCR3 = Device.nPWM[3];
#else
			GPIOB->BSRR = GPIO_PIN_11;
#endif
			nPWMTimer[3] = 10000001;
		}else{
			nPWMTimer[3] = Device.nDELAY[3]+10000000;
		}
	}
#if 0
  /* USER CODE END EXTI4_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(EX_INPUT4_Pin);
  /* USER CODE BEGIN EXTI4_IRQn 1 */
#endif
	if(__HAL_GPIO_EXTI_GET_IT(EX_INPUT4_Pin) != 0x00u){
		__HAL_GPIO_EXTI_CLEAR_IT(EX_INPUT4_Pin);
	}
  /* USER CODE END EXTI4_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[9:5] interrupts.
  */
void EXTI9_5_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI9_5_IRQn 0 */
	if(Device.nDeviceCurrentStatus == LCD_STATUS_MODE_STROBE_NORMAL){
		if(Device.nDELAY[2] == 0){
#if DAEGON_MODEL
			TIM2->CCR3 = Device.nPWM[2];
#else
			GPIOB->BSRR = GPIO_PIN_10;
#endif
			nPWMTimer[2] = 10000001;
		}else{
			nPWMTimer[2] = Device.nDELAY[2]+10000000;
		}
	}
#if 0
  /* USER CODE END EXTI9_5_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(EX_INPUT3_Pin);
  /* USER CODE BEGIN EXTI9_5_IRQn 1 */
#endif
	if(__HAL_GPIO_EXTI_GET_IT(EX_INPUT3_Pin) != 0x00u){
		__HAL_GPIO_EXTI_CLEAR_IT(EX_INPUT3_Pin);
	}

  /* USER CODE END EXTI9_5_IRQn 1 */
}

/**
  * @brief This function handles TIM1 update interrupt.
  */
void TIM1_UP_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_IRQn 0 */
	if(nStrobe == STROBE_MODE){
		if(nPWMTimer[0] > 0){
			nPWMTimer[0]--;
			if(nPWMTimer[0] == INIT_DELAY_MS){
#if DAEGON_MODEL
				//PWM_OUT(0, Device.nPWM[0]);
				//sConfigOC_.Pulse = Device.nPWM[0];
				//HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC_, TIM_CHANNEL_1);
				TIM2->CCR1 = Device.nPWM[0];
#else
				GPIOA->BSRR = GPIO_PIN_15;
#endif
				nPWMTimer[0] = Device.nPULSE[0];
			}else if(nPWMTimer[0] <= 0){
#if DAEGON_MODEL
				//PWM_OUT(0, 0);
				//sConfigOC_.Pulse = 0;
				//HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC_, TIM_CHANNEL_1);
				TIM2->CCR1 = 0;
#else
				GPIOA->BSRR = 0x80000000;//(uint32_t)GPIO_PIN_15 << 16u;
#endif
			}
//			else{
//				nPWMTimer[0]-=TIMER_UNIT;
//			}
		}
	}else if (nStrobe == STROBE_TEST){
		nStrobeTestCount--;
		if (nStrobeTestCount == 100000){
#if DAEGON_MODEL
			//PWM_OUT(0,0);
			//PWM_OUT(1,0);
			//sConfigOC_.Pulse = 0;
			//HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC_, TIM_CHANNEL_1);
			//HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC_, TIM_CHANNEL_2);
			TIM2->CCR1 = 0;
			TIM2->CCR2 = 0;
#if DAEGON_4CH
			TIM2->CCR3 = 0;
			TIM2->CCR4 = 0;
#endif
#else
			GPIOA->BSRR = 0x80000000;
			GPIOB->BSRR = 0x00080000;//(uint32_t)GPIO_PIN_3 << 16u;
			GPIOB->BSRR = 0x04000000;//(uint32_t)GPIO_PIN_10 << 16u;
			GPIOB->BSRR = 0x08000000;//(uint32_t)GPIO_PIN_11 << 16u;
#endif
		}else if(nStrobeTestCount == 100){
#if DAEGON_MODEL
			//PWM_OUT(0,255);
			//PWM_OUT(1,255);
			//sConfigOC_.Pulse = 255;
			//HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC_, TIM_CHANNEL_1);
			//HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC_, TIM_CHANNEL_2);
			TIM2->CCR1 = 255;
			TIM2->CCR2 = 255;
#if DAEGON_4CH
			TIM2->CCR3 = 255;
			TIM2->CCR4 = 255;
#endif
#else
			GPIOA->BSRR = GPIO_PIN_15;
			GPIOB->BSRR = GPIO_PIN_3;
			GPIOB->BSRR = GPIO_PIN_10;
			GPIOB->BSRR = GPIO_PIN_11;
#endif
		}else if(nStrobeTestCount <= 0){
			nStrobeTestCount =  100001;
		}
	}

#if 0
  /* USER CODE END TIM1_UP_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_UP_IRQn 1 */
#endif
	if (__HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_UPDATE) != RESET){
		if (__HAL_TIM_GET_IT_SOURCE(&htim1, TIM_IT_UPDATE) != RESET)
		{
			__HAL_TIM_CLEAR_IT(&htim1, TIM_IT_UPDATE);
		}
	}
  /* USER CODE END TIM1_UP_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */
#if DAEGON_MODEL
#else
	if (nStrobe == STROBE_MODE){
		if(nPWMTimer[1] > 0){
			nPWMTimer[1]-=TIMER_UNIT;
			if(nPWMTimer[1] == INIT_DELAY_MS){
				GPIOB->BSRR = GPIO_PIN_3;
				nPWMTimer[1] = Device.nPULSE[1];
			}else if(nPWMTimer[1] <= 0){
				GPIOB->BSRR = 0x00080000;//(uint32_t)GPIO_PIN_3 << 16u;
			}
//			else{
//				nPWMTimer[1]-=TIMER_UNIT;
//			}
		}
	}
#if 0
  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */
#endif
#endif
	if (__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_UPDATE) != RESET){
		if (__HAL_TIM_GET_IT_SOURCE(&htim2, TIM_IT_UPDATE) != RESET){
			__HAL_TIM_CLEAR_IT(&htim2, TIM_IT_UPDATE);
		}
	}
  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */
	uint8_t idx = 2;
#if	DAEGON_MODEL
	idx = 1;
#endif
	if (nStrobe == STROBE_MODE){
		if(nPWMTimer[idx] > 0){
			nPWMTimer[idx]-=TIMER_UNIT;
			if(nPWMTimer[idx] == INIT_DELAY_MS){
#if	DAEGON_MODEL
				//PWM_OUT(idx, Device.nPWM[idx]);
				//sConfigOC_.Pulse = Device.nPWM[idx];
				//HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC_, TIM_CHANNEL_2);
				TIM2->CCR2 = Device.nPWM[idx];
#else
				GPIOB->BSRR = GPIO_PIN_10;
#endif
				nPWMTimer[idx] = Device.nPULSE[idx];
			}else if(nPWMTimer[idx] <= 0){
#if DAEGON_MODEL
				//PWM_OUT(idx,0);
				//sConfigOC_.Pulse = 0;
				//HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC_, TIM_CHANNEL_2);
				TIM2->CCR2 = 0;
#else
				GPIOB->BSRR = 0x04000000;//(uint32_t)GPIO_PIN_10 << 16u;
#endif
			}
		}
	}
#if 0
  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */
#endif
	if (__HAL_TIM_GET_FLAG(&htim3, TIM_FLAG_UPDATE) != RESET){
		if (__HAL_TIM_GET_IT_SOURCE(&htim3, TIM_IT_UPDATE) != RESET){
			__HAL_TIM_CLEAR_IT(&htim3, TIM_IT_UPDATE);
		}
	}
  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles TIM4 global interrupt.
  */
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */
	if (nStrobe == STROBE_MODE){
#if	DAEGON_MODEL
		for(int i = 2; i < 4; i++){
			if(nPWMTimer[i] > 0){
				nPWMTimer[i]--;
				if(nPWMTimer[i] == INIT_DELAY_MS){
					if(i == 2){
						TIM2->CCR3 = Device.nPWM[i];
					}else{
						TIM2->CCR4 = Device.nPWM[i];
					}
					nPWMTimer[i] = Device.nPULSE[i];
				}
			}else if(nPWMTimer[i] <= 0){
				if(i == 2){
					TIM2->CCR3 = 0;
				}else{
					TIM2->CCR4 = 0;
				}
			}
		}
#else
		if(nPWMTimer[3] > 0){
			nPWMTimer[3]--;
			if(nPWMTimer[3] == INIT_DELAY_MS){
				GPIOB->BSRR = GPIO_PIN_11;
				nPWMTimer[3] = Device.nPULSE[3];
			}else if(nPWMTimer[3] <= 0){
				GPIOB->BSRR = 0x08000000;//(uint32_t)GPIO_PIN_11 << 16u;
			}
		}
#endif
	}
#if 0
  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */
#endif
	if (__HAL_TIM_GET_FLAG(&htim4, TIM_FLAG_UPDATE) != RESET){
		if (__HAL_TIM_GET_IT_SOURCE(&htim4, TIM_IT_UPDATE) != RESET){
			__HAL_TIM_CLEAR_IT(&htim4, TIM_IT_UPDATE);
		}
	}
  /* USER CODE END TIM4_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */

  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */

  /* USER CODE END USART2_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[15:10] interrupts.
  */
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */

  /* USER CODE END EXTI15_10_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_12);
  /* USER CODE BEGIN EXTI15_10_IRQn 1 */
  //for(int i = 0; i < 200000000; i++) HAL_Delay(10);
  /* USER CODE END EXTI15_10_IRQn 1 */
}

/* USER CODE BEGIN 1 */
void setCurDeviceStatus(void)
{
	if ((Device.nDeviceCurrentStatus >= LCD_STATUS_MODE_STROBE_NORMAL) && (Device.nDeviceCurrentStatus <= LCD_STATUS_MODE_STROBE_REMOTE)){
		nStrobe = STROBE_MODE;
	}else if(Device.nDeviceCurrentStatus == LCD_STATUS_MODE_STROBE_TEST){
		nStrobe = STROBE_TEST;
	}else{
		nStrobe = DIMMING_MODE;
	}
}
/* USER CODE END 1 */
