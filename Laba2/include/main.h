/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
 
#define Latch_Pin GPIO_PIN_5
#define Latch_Port GPIOB
#define Clock_Pin GPIO_PIN_8
#define Clock_Port GPIOA
#define Data_Pin GPIO_PIN_9
#define Data_Port GPIOA

#define D1_Port GPIOA
#define D2_Port GPIOA
#define D3_Port GPIOA
#define D4_Port GPIOB

#define D1_Pin GPIO_PIN_5
#define D2_Pin GPIO_PIN_6
#define D3_Pin GPIO_PIN_7
#define D4_Pin GPIO_PIN_6

#define SB1_Port GPIOA
#define SB2_Port GPIOA
#define SB3_Port GPIOB

#define SB1_Pin GPIO_PIN_1
#define SB2_Pin GPIO_PIN_4
#define SB3_Pin GPIO_PIN_0

#define WAIT_GAME 1
#define SHOW_LEVEL 2
#define INPUT_LEVEL 3
#define RESULTS_LEVEL 4

#define NONE 10
#define E_BIG 11
#define R_LITTLE 12

#define BLINK_DELAY 500

#define MAX(X,Y) (X) > (Y) ? (X) : (Y)

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

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI15_10_IRQn
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
