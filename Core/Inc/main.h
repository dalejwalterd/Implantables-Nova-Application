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
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "data.h"
#include "can.h"

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
#define I2C2_INT_Pin GPIO_PIN_12
#define I2C2_INT_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

#define TIMEOUT_ms(n)	((n))
#define TIMEOUT_sec(n)	(TIMEOUT_ms((n) * 1000L) - 1)
#define START_DELAY_MS  4UL	//to be multiplied by nodeID on startup

extern uint8_t RxData[8];
extern uint8_t TxData[8];
extern CAN_TxHeaderTypeDef TxHeader;
extern uint32_t TxMailbox;
extern CAN_RxHeaderTypeDef RxHeader;

extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;
extern CAN_HandleTypeDef hcan1;
extern I2C_HandleTypeDef hi2c2;
extern TIM_HandleTypeDef htim2;

extern uint8_t ADCres[4];

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
