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
#include "stm32g4xx_hal.h"

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
#define __1V65_Pin GPIO_PIN_1
#define __1V65_GPIO_Port GPIOA
#define TEMP_PGA_Pin GPIO_PIN_2
#define TEMP_PGA_GPIO_Port GPIOA
#define TEMP_SENSE_Pin GPIO_PIN_3
#define TEMP_SENSE_GPIO_Port GPIOA
#define PHOTO_DIFF_P_Pin GPIO_PIN_5
#define PHOTO_DIFF_P_GPIO_Port GPIOA
#define PGA_OUT_P_Pin GPIO_PIN_6
#define PGA_OUT_P_GPIO_Port GPIOA
#define __1V65A7_Pin GPIO_PIN_7
#define __1V65A7_GPIO_Port GPIOA
#define V__SENSE_Pin GPIO_PIN_4
#define V__SENSE_GPIO_Port GPIOC
#define __1V65B0_Pin GPIO_PIN_0
#define __1V65B0_GPIO_Port GPIOB
#define PGA_OUT_N_Pin GPIO_PIN_1
#define PGA_OUT_N_GPIO_Port GPIOB
#define PHOTO_DIFF_N_Pin GPIO_PIN_2
#define PHOTO_DIFF_N_GPIO_Port GPIOB
#define ADC_PHOTO_N_Pin GPIO_PIN_11
#define ADC_PHOTO_N_GPIO_Port GPIOB
#define VFPI_SENSE_Pin GPIO_PIN_12
#define VFPI_SENSE_GPIO_Port GPIOB
#define I_LAMP_Pin GPIO_PIN_14
#define I_LAMP_GPIO_Port GPIOB
#define ADC_PHOTO_P_Pin GPIO_PIN_15
#define ADC_PHOTO_P_GPIO_Port GPIOB
#define nI_LAMP_ALERT_Pin GPIO_PIN_6
#define nI_LAMP_ALERT_GPIO_Port GPIOC
#define nI_LAMP_ALERT_EXTI_IRQn EXTI9_5_IRQn
#define EN_PWM_Pin GPIO_PIN_8
#define EN_PWM_GPIO_Port GPIOA
#define BOOST_PWM_Pin GPIO_PIN_9
#define BOOST_PWM_GPIO_Port GPIOA
#define LAMP_PWM_Pin GPIO_PIN_10
#define LAMP_PWM_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
