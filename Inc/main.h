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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define PE2_232_POWER_Pin GPIO_PIN_2
#define PE2_232_POWER_GPIO_Port GPIOE
#define Status_Pin GPIO_PIN_4
#define Status_GPIO_Port GPIOE
#define PC3_RS485_DE_Pin GPIO_PIN_3
#define PC3_RS485_DE_GPIO_Port GPIOC
#define PB0_Relay_Pin GPIO_PIN_0
#define PB0_Relay_GPIO_Port GPIOB
#define PB1_485_POWER_Pin GPIO_PIN_1
#define PB1_485_POWER_GPIO_Port GPIOB
#define LORA_Power_Pin GPIO_PIN_2
#define LORA_Power_GPIO_Port GPIOB
#define PE7_12VC_Pin GPIO_PIN_7
#define PE7_12VC_GPIO_Port GPIOE
#define PE8_5VC_Pin GPIO_PIN_8
#define PE8_5VC_GPIO_Port GPIOE
#define PE9_3V3_Pin GPIO_PIN_9
#define PE9_3V3_GPIO_Port GPIOE
#define PE10_5V_ADC_Pin GPIO_PIN_10
#define PE10_5V_ADC_GPIO_Port GPIOE
#define PE12_GPS_POWER_Pin GPIO_PIN_12
#define PE12_GPS_POWER_GPIO_Port GPIOE
#define Lora_RST_Pin GPIO_PIN_13
#define Lora_RST_GPIO_Port GPIOE
#define PE14_IIC_Power_Pin GPIO_PIN_14
#define PE14_IIC_Power_GPIO_Port GPIOE
#define PE15_TF_Power_Pin GPIO_PIN_15
#define PE15_TF_Power_GPIO_Port GPIOE
#define NB_EN_Pin GPIO_PIN_12
#define NB_EN_GPIO_Port GPIOB
#define Lora_IRQ5_Pin GPIO_PIN_13
#define Lora_IRQ5_GPIO_Port GPIOB
#define LORA_IRQ4_Pin GPIO_PIN_14
#define LORA_IRQ4_GPIO_Port GPIOB
#define LORA_IRO0_Pin GPIO_PIN_15
#define LORA_IRO0_GPIO_Port GPIOB
#define LORA_IRQ1_Pin GPIO_PIN_8
#define LORA_IRQ1_GPIO_Port GPIOD
#define LORA_IRQ2_Pin GPIO_PIN_9
#define LORA_IRQ2_GPIO_Port GPIOD
#define LORA_IRQ3_Pin GPIO_PIN_10
#define LORA_IRQ3_GPIO_Port GPIOD
#define LSM6DSL_INT1_Pin GPIO_PIN_11
#define LSM6DSL_INT1_GPIO_Port GPIOD
#define TF_Detect_Pin GPIO_PIN_15
#define TF_Detect_GPIO_Port GPIOD
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
