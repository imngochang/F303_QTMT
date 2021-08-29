/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#define SIM_RESET_Pin GPIO_PIN_2
#define SIM_RESET_GPIO_Port GPIOC
#define SIM_PWR_Pin GPIO_PIN_3
#define SIM_PWR_GPIO_Port GPIOC
#define SIM_DTR_Pin GPIO_PIN_0
#define SIM_DTR_GPIO_Port GPIOA
#define SIM_RI_Pin GPIO_PIN_1
#define SIM_RI_GPIO_Port GPIOA
#define SIM_RI_EXTI_IRQn EXTI1_IRQn
#define SIM_RX_Pin GPIO_PIN_2
#define SIM_RX_GPIO_Port GPIOA
#define SIM_TX_Pin GPIO_PIN_3
#define SIM_TX_GPIO_Port GPIOA
#define RS485_RX_Pin GPIO_PIN_10
#define RS485_RX_GPIO_Port GPIOB
#define RS485_TX_Pin GPIO_PIN_11
#define RS485_TX_GPIO_Port GPIOB
#define RS485_EN_Pin GPIO_PIN_12
#define RS485_EN_GPIO_Port GPIOB
#define ADC_SOLAR_Pin GPIO_PIN_14
#define ADC_SOLAR_GPIO_Port GPIOB
#define ADC_VIN_Pin GPIO_PIN_15
#define ADC_VIN_GPIO_Port GPIOB
#define CTR_PWR_SS_Pin GPIO_PIN_8
#define CTR_PWR_SS_GPIO_Port GPIOA
#define IN_1_Pin GPIO_PIN_11
#define IN_1_GPIO_Port GPIOC
#define IN_2_Pin GPIO_PIN_12
#define IN_2_GPIO_Port GPIOC
#define IN_3_Pin GPIO_PIN_2
#define IN_3_GPIO_Port GPIOD
#define IN_4_Pin GPIO_PIN_3
#define IN_4_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
