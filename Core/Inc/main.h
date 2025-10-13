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
#define E1A_Pin GPIO_PIN_0
#define E1A_GPIO_Port GPIOA
#define E1B_Pin GPIO_PIN_1
#define E1B_GPIO_Port GPIOA
#define ECHO_Pin GPIO_PIN_0
#define ECHO_GPIO_Port GPIOB
#define ECHO_EXTI_IRQn EXTI0_IRQn
#define TRIG_Pin GPIO_PIN_1
#define TRIG_GPIO_Port GPIOB
#define stop_car_Pin GPIO_PIN_2
#define stop_car_GPIO_Port GPIOB
#define E3B_Pin GPIO_PIN_6
#define E3B_GPIO_Port GPIOC
#define E3A_Pin GPIO_PIN_7
#define E3A_GPIO_Port GPIOC
#define EN2_Pin GPIO_PIN_8
#define EN2_GPIO_Port GPIOC
#define EN1_Pin GPIO_PIN_9
#define EN1_GPIO_Port GPIOC
#define PH2_Pin GPIO_PIN_11
#define PH2_GPIO_Port GPIOA
#define PH1_Pin GPIO_PIN_12
#define PH1_GPIO_Port GPIOA
#define SPI_CS_Pin GPIO_PIN_15
#define SPI_CS_GPIO_Port GPIOA
#define unused_Pin GPIO_PIN_12
#define unused_GPIO_Port GPIOC
#define DR_IRQ_Pin GPIO_PIN_5
#define DR_IRQ_GPIO_Port GPIOB
#define DR_IRQ_EXTI_IRQn EXTI9_5_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
