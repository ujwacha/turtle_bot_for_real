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
#include "stm32f4xx_hal.h"

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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define M3D_Pin GPIO_PIN_3
#define M3D_GPIO_Port GPIOE
#define M3P_Pin GPIO_PIN_5
#define M3P_GPIO_Port GPIOE
#define M4P_Pin GPIO_PIN_6
#define M4P_GPIO_Port GPIOE
#define M4D_Pin GPIO_PIN_14
#define M4D_GPIO_Port GPIOC
#define START_BTN_Pin GPIO_PIN_0
#define START_BTN_GPIO_Port GPIOA
#define M2P_Pin GPIO_PIN_3
#define M2P_GPIO_Port GPIOA
#define M1P_Pin GPIO_PIN_5
#define M1P_GPIO_Port GPIOA
#define M1D_Pin GPIO_PIN_7
#define M1D_GPIO_Port GPIOE
#define ENC1_C1_Pin GPIO_PIN_9
#define ENC1_C1_GPIO_Port GPIOE
#define ENC1_C2_Pin GPIO_PIN_11
#define ENC1_C2_GPIO_Port GPIOE
#define M2D_Pin GPIO_PIN_14
#define M2D_GPIO_Port GPIOE
#define GREEN_LED_Pin GPIO_PIN_12
#define GREEN_LED_GPIO_Port GPIOD
#define ORANGE_LED_Pin GPIO_PIN_13
#define ORANGE_LED_GPIO_Port GPIOD
#define RED_LED_Pin GPIO_PIN_14
#define RED_LED_GPIO_Port GPIOD
#define BLUE_LED_Pin GPIO_PIN_15
#define BLUE_LED_GPIO_Port GPIOD
#define ENC5_C1_Pin GPIO_PIN_6
#define ENC5_C1_GPIO_Port GPIOC
#define ENC5_C2_Pin GPIO_PIN_7
#define ENC5_C2_GPIO_Port GPIOC
#define LED4_Pin GPIO_PIN_8
#define LED4_GPIO_Port GPIOC
#define LED6_Pin GPIO_PIN_8
#define LED6_GPIO_Port GPIOA
#define LED2_Pin GPIO_PIN_12
#define LED2_GPIO_Port GPIOC
#define LED5_Pin GPIO_PIN_1
#define LED5_GPIO_Port GPIOD
#define LED1_Pin GPIO_PIN_3
#define LED1_GPIO_Port GPIOD
#define ENC2_C1_Pin GPIO_PIN_4
#define ENC2_C1_GPIO_Port GPIOB
#define ENC2_C2_Pin GPIO_PIN_5
#define ENC2_C2_GPIO_Port GPIOB
#define ENC3_C1_Pin GPIO_PIN_6
#define ENC3_C1_GPIO_Port GPIOB
#define ENC3_C2_Pin GPIO_PIN_7
#define ENC3_C2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
