/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

extern IWDG_HandleTypeDef hiwdg;
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void JumpToBootloader(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define OPLED_Pin GPIO_PIN_13
#define OPLED_GPIO_Port GPIOC
#define StatusLED_Pin GPIO_PIN_14
#define StatusLED_GPIO_Port GPIOC
#define EX_INPUT1_Pin GPIO_PIN_0
#define EX_INPUT1_GPIO_Port GPIOB
#define EX_INPUT1_EXTI_IRQn EXTI0_IRQn
#define EX_INPUT2_Pin GPIO_PIN_1
#define EX_INPUT2_GPIO_Port GPIOB
#define EX_INPUT2_EXTI_IRQn EXTI1_IRQn
#define BOOT1_Pin GPIO_PIN_2
#define BOOT1_GPIO_Port GPIOB
#define SW2_Pin GPIO_PIN_13
#define SW2_GPIO_Port GPIOB
#define SW3_Pin GPIO_PIN_14
#define SW3_GPIO_Port GPIOB
#define SW4_Pin GPIO_PIN_15
#define SW4_GPIO_Port GPIOB
#define EX_INPUT4_Pin GPIO_PIN_4
#define EX_INPUT4_GPIO_Port GPIOB
#define EX_INPUT4_EXTI_IRQn EXTI4_IRQn
#define EX_INPUT3_Pin GPIO_PIN_5
#define EX_INPUT3_GPIO_Port GPIOB
#define EX_INPUT3_EXTI_IRQn EXTI9_5_IRQn

/* USER CODE BEGIN Private defines */
#define CH1_OUT_Pin GPIO_PIN_15
#define CH1_OUT_GPIO_Port GPIOA
#define CH2_OUT_Pin GPIO_PIN_3
#define CH2_OUT_GPIO_Port GPIOB
#define CH3_OUT_Pin GPIO_PIN_10
#define CH3_OUT_GPIO_Port GPIOB
#define CH4_OUT_Pin GPIO_PIN_11
#define CH4_OUT_GPIO_Port GPIOB

#define SW1_Pin GPIO_PIN_12
#define SW1_GPIO_Port GPIOB

//#define TEST_10KHZ		1

#define DAEGON_MODEL	0
#define DAEGON_4CH		0
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
