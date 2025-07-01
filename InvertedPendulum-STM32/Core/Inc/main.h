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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MCO_Pin GPIO_PIN_0
#define MCO_GPIO_Port GPIOF
#define Mux_ADC_Pin GPIO_PIN_0
#define Mux_ADC_GPIO_Port GPIOA
#define MD_In4_Pin GPIO_PIN_1
#define MD_In4_GPIO_Port GPIOA
#define VCP_TX_Pin GPIO_PIN_2
#define VCP_TX_GPIO_Port GPIOA
#define VCP_RX_Pin GPIO_PIN_3
#define VCP_RX_GPIO_Port GPIOA
#define QEI_L_B_Pin GPIO_PIN_4
#define QEI_L_B_GPIO_Port GPIOA
#define Current_ADC_R_Pin GPIO_PIN_5
#define Current_ADC_R_GPIO_Port GPIOA
#define QEI_R_A_Pin GPIO_PIN_6
#define QEI_R_A_GPIO_Port GPIOA
#define Current_ADC_L_Pin GPIO_PIN_7
#define Current_ADC_L_GPIO_Port GPIOA
#define P_ADC1_Pin GPIO_PIN_0
#define P_ADC1_GPIO_Port GPIOB
#define P_ADC2_Pin GPIO_PIN_1
#define P_ADC2_GPIO_Port GPIOB
#define QEI_L_A_Pin GPIO_PIN_8
#define QEI_L_A_GPIO_Port GPIOA
#define QEI_L_BA9_Pin GPIO_PIN_9
#define QEI_L_BA9_GPIO_Port GPIOA
#define MD_In1_Pin GPIO_PIN_10
#define MD_In1_GPIO_Port GPIOA
#define MD_In2_Pin GPIO_PIN_11
#define MD_In2_GPIO_Port GPIOA
#define Mux_C_Pin GPIO_PIN_12
#define Mux_C_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define Mux_B_Pin GPIO_PIN_3
#define Mux_B_GPIO_Port GPIOB
#define Mux_A_Pin GPIO_PIN_4
#define Mux_A_GPIO_Port GPIOB
#define SW_Pin GPIO_PIN_5
#define SW_GPIO_Port GPIOB
#define LED_Pin GPIO_PIN_6
#define LED_GPIO_Port GPIOB
#define MD_In3_Pin GPIO_PIN_7
#define MD_In3_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
