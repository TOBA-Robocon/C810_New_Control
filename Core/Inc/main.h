/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32f3xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define W_Current_Pin GPIO_PIN_0
#define W_Current_GPIO_Port GPIOA
#define V_Current_Pin GPIO_PIN_1
#define V_Current_GPIO_Port GPIOA
#define U_Current_Pin GPIO_PIN_2
#define U_Current_GPIO_Port GPIOA
#define W_SD_Pin GPIO_PIN_3
#define W_SD_GPIO_Port GPIOA
#define Encoder_1_Pin GPIO_PIN_4
#define Encoder_1_GPIO_Port GPIOA
#define Encoder_2_Pin GPIO_PIN_5
#define Encoder_2_GPIO_Port GPIOA
#define W_PWM_Pin GPIO_PIN_6
#define W_PWM_GPIO_Port GPIOA
#define V_PWM_Pin GPIO_PIN_7
#define V_PWM_GPIO_Port GPIOA
#define U_PWM_Pin GPIO_PIN_0
#define U_PWM_GPIO_Port GPIOB
#define V_SD_Pin GPIO_PIN_1
#define V_SD_GPIO_Port GPIOB
#define U_SD_Pin GPIO_PIN_8
#define U_SD_GPIO_Port GPIOA
#define LED_G_Pin GPIO_PIN_4
#define LED_G_GPIO_Port GPIOB
#define LED_R_Pin GPIO_PIN_5
#define LED_R_GPIO_Port GPIOB
#define LED_B_Pin GPIO_PIN_6
#define LED_B_GPIO_Port GPIOB
#define test_GPIO_Pin GPIO_PIN_7
#define test_GPIO_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
