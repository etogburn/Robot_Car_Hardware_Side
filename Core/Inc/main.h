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
#define PWM_AL_RMC_Pin GPIO_PIN_13
#define PWM_AL_RMC_GPIO_Port GPIOC
#define RMC_HALL2_Pin GPIO_PIN_7
#define RMC_HALL2_GPIO_Port GPIOF
#define RMC_HALL3_Pin GPIO_PIN_8
#define RMC_HALL3_GPIO_Port GPIOF
#define PWM_AH_RMC_Pin GPIO_PIN_0
#define PWM_AH_RMC_GPIO_Port GPIOC
#define PWM_BH_RMC_Pin GPIO_PIN_1
#define PWM_BH_RMC_GPIO_Port GPIOC
#define PWM_CH_RMC_Pin GPIO_PIN_2
#define PWM_CH_RMC_GPIO_Port GPIOC
#define BOARD_TEMP_Pin GPIO_PIN_3
#define BOARD_TEMP_GPIO_Port GPIOA
#define nAUX2_EN_Pin GPIO_PIN_5
#define nAUX2_EN_GPIO_Port GPIOA
#define WHEEL_MOTOR_CURRENT_LIMIT_Pin GPIO_PIN_6
#define WHEEL_MOTOR_CURRENT_LIMIT_GPIO_Port GPIOA
#define PWM_BL_RMC_Pin GPIO_PIN_0
#define PWM_BL_RMC_GPIO_Port GPIOB
#define nDEBUG_LED2_Pin GPIO_PIN_11
#define nDEBUG_LED2_GPIO_Port GPIOF
#define nDEBUG_LED1_Pin GPIO_PIN_14
#define nDEBUG_LED1_GPIO_Port GPIOF
#define LMC_CURRSENSE_Pin GPIO_PIN_7
#define LMC_CURRSENSE_GPIO_Port GPIOE
#define PWM_CL_RMC_Pin GPIO_PIN_12
#define PWM_CL_RMC_GPIO_Port GPIOE
#define nRMC_CURRFAULT_Pin GPIO_PIN_10
#define nRMC_CURRFAULT_GPIO_Port GPIOB
#define RMC_CURRSENSE_Pin GPIO_PIN_14
#define RMC_CURRSENSE_GPIO_Port GPIOB
#define V_3V3_DIAG_Pin GPIO_PIN_8
#define V_3V3_DIAG_GPIO_Port GPIOD
#define V_5V_DIAG_Pin GPIO_PIN_9
#define V_5V_DIAG_GPIO_Port GPIOD
#define VBAT_SW_DIAG_Pin GPIO_PIN_10
#define VBAT_SW_DIAG_GPIO_Port GPIOD
#define AUX2_OUTPUT_Pin GPIO_PIN_15
#define AUX2_OUTPUT_GPIO_Port GPIOD
#define PWM_BH_LMC_Pin GPIO_PIN_7
#define PWM_BH_LMC_GPIO_Port GPIOC
#define ACC_INT_Pin GPIO_PIN_0
#define ACC_INT_GPIO_Port GPIOG
#define ACC_INT_EXTI_IRQn EXTI0_IRQn
#define nCS_MCM_GYRO_SPI_Pin GPIO_PIN_1
#define nCS_MCM_GYRO_SPI_GPIO_Port GPIOG
#define LED_A_INT_Pin GPIO_PIN_12
#define LED_A_INT_GPIO_Port GPIOA
#define RMC_HALL1_Pin GPIO_PIN_6
#define RMC_HALL1_GPIO_Port GPIOF
#define PWM_AL_LMC_Pin GPIO_PIN_10
#define PWM_AL_LMC_GPIO_Port GPIOC
#define PWM_BL_LMC_Pin GPIO_PIN_11
#define PWM_BL_LMC_GPIO_Port GPIOC
#define PWM_CL_LMC_Pin GPIO_PIN_12
#define PWM_CL_LMC_GPIO_Port GPIOC
#define MCM_KILL_Pin GPIO_PIN_7
#define MCM_KILL_GPIO_Port GPIOG
#define CAN_EN_5V_Pin GPIO_PIN_8
#define CAN_EN_5V_GPIO_Port GPIOG
#define nCAN_STBY_Pin GPIO_PIN_9
#define nCAN_STBY_GPIO_Port GPIOG
#define nLMC_CURRFAULT_Pin GPIO_PIN_2
#define nLMC_CURRFAULT_GPIO_Port GPIOD
#define LMC_HALL1_Pin GPIO_PIN_3
#define LMC_HALL1_GPIO_Port GPIOD
#define LMC_HALL2_Pin GPIO_PIN_4
#define LMC_HALL2_GPIO_Port GPIOD
#define AUX2_TX_Pin GPIO_PIN_5
#define AUX2_TX_GPIO_Port GPIOD
#define AUX2_RX_Pin GPIO_PIN_6
#define AUX2_RX_GPIO_Port GPIOD
#define LMC_HALL3_Pin GPIO_PIN_7
#define LMC_HALL3_GPIO_Port GPIOD
#define PWM_AH_LMC_Pin GPIO_PIN_6
#define PWM_AH_LMC_GPIO_Port GPIOB
#define PWM_CH_LMC_Pin GPIO_PIN_9
#define PWM_CH_LMC_GPIO_Port GPIOB
#define nWHEEL_MOTOR_SHDN_Pin GPIO_PIN_1
#define nWHEEL_MOTOR_SHDN_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
