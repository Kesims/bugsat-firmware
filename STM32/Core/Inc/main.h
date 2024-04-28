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
#define BTN_USER_Pin GPIO_PIN_3
#define BTN_USER_GPIO_Port GPIOC
#define SPI3_CS_Pin GPIO_PIN_4
#define SPI3_CS_GPIO_Port GPIOA
#define GPS_TX_Pin GPIO_PIN_4
#define GPS_TX_GPIO_Port GPIOC
#define GPS_RX_Pin GPIO_PIN_5
#define GPS_RX_GPIO_Port GPIOC
#define LORA_NSS_Pin GPIO_PIN_0
#define LORA_NSS_GPIO_Port GPIOB
#define BATREF_Pin GPIO_PIN_1
#define BATREF_GPIO_Port GPIOB
#define LORA_RST_Pin GPIO_PIN_12
#define LORA_RST_GPIO_Port GPIOB
#define LORA_DIO0_Pin GPIO_PIN_13
#define LORA_DIO0_GPIO_Port GPIOB
#define LORA_DIO1_Pin GPIO_PIN_14
#define LORA_DIO1_GPIO_Port GPIOB
#define LORA_DIO2_Pin GPIO_PIN_15
#define LORA_DIO2_GPIO_Port GPIOB
#define BMP_INT_Pin GPIO_PIN_7
#define BMP_INT_GPIO_Port GPIOC
#define LIS3_INT1_Pin GPIO_PIN_8
#define LIS3_INT1_GPIO_Port GPIOC
#define LIS3_INT2_Pin GPIO_PIN_9
#define LIS3_INT2_GPIO_Port GPIOC
#define TRIGGER_Pin GPIO_PIN_8
#define TRIGGER_GPIO_Port GPIOA
#define H3LIS_INT1_Pin GPIO_PIN_11
#define H3LIS_INT1_GPIO_Port GPIOA
#define H3LIS_INT2_Pin GPIO_PIN_12
#define H3LIS_INT2_GPIO_Port GPIOA
#define GPS_TIMEPULSE_Pin GPIO_PIN_2
#define GPS_TIMEPULSE_GPIO_Port GPIOD
#define LED_RED_Pin GPIO_PIN_8
#define LED_RED_GPIO_Port GPIOB
#define LED_GREEN_Pin GPIO_PIN_9
#define LED_GREEN_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

// ----- GENERAL SETTINGS -----

#define ENABLE_DEBUG_MESSAGES 1


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
