/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#define EN_B_Pin GPIO_PIN_1
#define EN_B_GPIO_Port GPIOC
#define PWM_1_Pin GPIO_PIN_0
#define PWM_1_GPIO_Port GPIOA
#define PWM_2_Pin GPIO_PIN_1
#define PWM_2_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */

#define MAX_VOLTAGE 12
#define MAX_COUNT 1200
#define STOP_COUNT 600
#define TOTAL_CURVES 12
#define ENCODER_TOTAL_POS 4200
#define ENCODER_THRESHOLE 1000
#define ENCODER_LAP 24
#define REDUCTER 463

/* CONTROLLER */
double Kp;
#define LAPS 5
#define FINISH_TIME 5000
uint32_t count;
uint32_t reference;
void setReference(uint32_t vueltas);

#define START_POSITION  0x7FFFFFFF



uint32_t encoder, last_encoder;
uint32_t pos;
int32_t diff,extra;

typedef enum
{
	POSITIVE  = 0,
	NEGATIVE   = 1
}Sentido;

Sentido direction;
void startSystem (void);
void setVoltage (float voltage);
void isNewLap (void);
void readEncoder (void);
void controller(void);
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
