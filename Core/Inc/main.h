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
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;
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
#define gray_left4_Pin GPIO_PIN_0
#define gray_left4_GPIO_Port GPIOC
#define gray_left5_Pin GPIO_PIN_1
#define gray_left5_GPIO_Port GPIOC
#define gray_left6_Pin GPIO_PIN_2
#define gray_left6_GPIO_Port GPIOC
#define gray_left7_Pin GPIO_PIN_3
#define gray_left7_GPIO_Port GPIOC
#define gray_left3_Pin GPIO_PIN_4
#define gray_left3_GPIO_Port GPIOA
#define gray_left2_Pin GPIO_PIN_4
#define gray_left2_GPIO_Port GPIOC
#define gray_left1_Pin GPIO_PIN_5
#define gray_left1_GPIO_Port GPIOC
#define gray_left0_Pin GPIO_PIN_0
#define gray_left0_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_1
#define LED2_GPIO_Port GPIOB
#define LED1_Pin GPIO_PIN_2
#define LED1_GPIO_Port GPIOB
#define LED3_Pin GPIO_PIN_6
#define LED3_GPIO_Port GPIOC
#define LED4_Pin GPIO_PIN_7
#define LED4_GPIO_Port GPIOC
#define EN2_Pin GPIO_PIN_8
#define EN2_GPIO_Port GPIOC
#define EN1_Pin GPIO_PIN_9
#define EN1_GPIO_Port GPIOC
#define SR04_TRIG_Pin GPIO_PIN_8
#define SR04_TRIG_GPIO_Port GPIOA
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
