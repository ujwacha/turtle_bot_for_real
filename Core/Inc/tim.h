/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tim.h
  * @brief   This file contains all the function prototypes for
  *          the tim.c file
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
  ************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TIM_H__
#define __TIM_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern TIM_HandleTypeDef htim1;

extern TIM_HandleTypeDef htim2;

extern TIM_HandleTypeDef htim3;

extern TIM_HandleTypeDef htim4;

extern TIM_HandleTypeDef htim8;

extern TIM_HandleTypeDef htim9;

extern TIM_HandleTypeDef htim10;

extern TIM_HandleTypeDef htim14;

/* USER CODE BEGIN Private defines */
 #define M1P_Timer htim2 
 #define M2P_Timer htim2 
 #define M3P_Timer htim9 
 #define M4P_Timer htim9 

 #define M1P_Tim_Channel TIM_CHANNEL_1 
 #define M2P_Tim_Channel TIM_CHANNEL_4 
 #define M3P_Tim_Channel TIM_CHANNEL_1 
 #define M4P_Tim_Channel TIM_CHANNEL_2 

 #define ENC1_Timer htim1 
 #define ENC2_Timer htim3 
 #define ENC3_Timer htim4 
 #define ENC5_Timer htim8 
/* USER CODE END Private defines */

void MX_TIM1_Init(void);
void MX_TIM2_Init(void);
void MX_TIM3_Init(void);
void MX_TIM4_Init(void);
void MX_TIM8_Init(void);
void MX_TIM9_Init(void);
void MX_TIM10_Init(void);
void MX_TIM14_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __TIM_H__ */

