/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#define LED_BOARD_Pin GPIO_PIN_13
#define LED_BOARD_GPIO_Port GPIOC
#define UART_TX_2_MTF01_RX_Pin GPIO_PIN_2
#define UART_TX_2_MTF01_RX_GPIO_Port GPIOA
#define UART_RX_2_MTF01_TX_Pin GPIO_PIN_3
#define UART_RX_2_MTF01_TX_GPIO_Port GPIOA
#define FLASH_CS_Pin GPIO_PIN_4
#define FLASH_CS_GPIO_Port GPIOA
#define BNO085_SCL_Pin GPIO_PIN_10
#define BNO085_SCL_GPIO_Port GPIOB
#define UART_TX_2_RPI_RX_Pin GPIO_PIN_9
#define UART_TX_2_RPI_RX_GPIO_Port GPIOA
#define UART_RX_2_RPI_TX_Pin GPIO_PIN_10
#define UART_RX_2_RPI_TX_GPIO_Port GPIOA
#define BNO085_SDA_Pin GPIO_PIN_3
#define BNO085_SDA_GPIO_Port GPIOB
#define BNO085_INT_Pin GPIO_PIN_4
#define BNO085_INT_GPIO_Port GPIOB
#define BNO085_INT_EXTI_IRQn EXTI4_IRQn
#define BNO085_RST_Pin GPIO_PIN_5
#define BNO085_RST_GPIO_Port GPIOB
#define MOTOR1_DSHOT_Pin GPIO_PIN_6
#define MOTOR1_DSHOT_GPIO_Port GPIOB
#define MOTOR2_DSHOT_Pin GPIO_PIN_7
#define MOTOR2_DSHOT_GPIO_Port GPIOB
#define MOTOR3_DSHOT_Pin GPIO_PIN_8
#define MOTOR3_DSHOT_GPIO_Port GPIOB
#define MOTOR4_DSHOT_Pin GPIO_PIN_9
#define MOTOR4_DSHOT_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
