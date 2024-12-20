/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include <motor_control.h>
#include "main.h"
#include "adc.h"
#include "dac.h"
#include "dma.h"
#include "fdcan.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Coms_Handler.h"
#include "IMU.h"
#include "command_handler.h"
#include "robot_system.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
IMU_HandleTypeDef imu = {
	    .hspi = &hspi1,      // SPI handle
	    .cs_port = nCS_MCM_GYRO_SPI_GPIO_Port,        // GPIO port for CS pin
	    .cs_pin = nCS_MCM_GYRO_SPI_Pin,              // GPIO pin for CS
	    .int_port = ACC_INT_GPIO_Port,       // GPIO port for interrupt pin
	    .int_pin = ACC_INT_Pin
};

RobotSystem robot = {
		.Enable_Port = nWHEEL_MOTOR_SHDN_GPIO_Port,
		.Enable_Pin = nWHEEL_MOTOR_SHDN_Pin,
		.currentLimitDAC = &hdac2,
		.currentLimitDACChannel = DAC_CHANNEL_1
};

Motor rightWheel = {
		.Hall1_Channel = TIM_CHANNEL_1,
		.Hall2_Channel = TIM_CHANNEL_2,
		.Hall3_Channel = TIM_CHANNEL_3,
		.Hall1_Port = LMC_HALL1_GPIO_Port,  // GPIO Port for Hall Sensor 1
		.Hall1_Pin = LMC_HALL1_Pin,       // GPIO Pin for Hall Sensor 1
		.Hall2_Port = LMC_HALL2_GPIO_Port,  // GPIO Port for Hall Sensor 2
		.Hall2_Pin = LMC_HALL2_Pin,      // GPIO Pin for Hall Sensor 2
		.Hall3_Port = LMC_HALL3_GPIO_Port,  // GPIO Port for Hall Sensor 3
		.Hall3_Pin = LMC_HALL3_Pin,        // GPIO Pin for Hall Sensor 3
		.HallTimer = &htim2,
		.Timer = &htim8,  // Timer for PWM control
		.phaseChannel = {TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3},
		.commutationOrder = {6,2,3,1,5,4}, //011, 010, 110, 100, 101, 001
		.hallState = 0,
		.isDirInverted = true,
		.direction = true,
		.acceleration = MAX_MOTOR_RPM/4, //rpm change per pid loop
		.pid.Kp = 150,
		.pid.Ki = 500,
		.pid.Kd = 10
};

Motor leftWheel = {
		.Hall1_Channel = TIM_CHANNEL_1,
		.Hall2_Channel = TIM_CHANNEL_2,
		.Hall3_Channel = TIM_CHANNEL_3,
		.Hall1_Port = RMC_HALL1_GPIO_Port,  // GPIO Port for Hall Sensor 1
		.Hall1_Pin = RMC_HALL1_Pin,       // GPIO Pin for Hall Sensor 1
		.Hall2_Port = RMC_HALL2_GPIO_Port,  // GPIO Port for Hall Sensor 2
		.Hall2_Pin = RMC_HALL2_Pin,      // GPIO Pin for Hall Sensor 2
		.Hall3_Port = RMC_HALL3_GPIO_Port,  // GPIO Port for Hall Sensor 3
		.Hall3_Pin = RMC_HALL3_Pin,        // GPIO Pin for Hall Sensor 3
		.HallTimer = &htim5,
		.Timer = &htim1,  // Timer for PWM control
		.phaseChannel = {TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3},
		.commutationOrder = {6,2,3,1,5,4}, //011, 010, 110, 100, 101, 001
		.hallState = 0,
		.isDirInverted = false,
		.direction = true,
		.acceleration = MAX_MOTOR_RPM/4, //rpm change per pid loop
		.pid.Kp = 150,
		.pid.Ki = 500,
		.pid.Kd = 10
};

ComsInterface_t serial;
DecodedPacket_t input;
DecodedPacket_t response;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs);
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC3_Init();
  MX_DAC2_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM8_Init();
  MX_TIM2_Init();
  MX_TIM5_Init();
  MX_FDCAN1_Init();
  /* USER CODE BEGIN 2 */

  IMU_Init(&imu);
  RobotSystem_Init(&robot, leftWheel, rightWheel);
  Comm_Init(&serial, COMM_UART, &huart2);
  HAL_Delay(250);
  DecodedPacket_t readyPacket = {
		  .command = COMMAND_READY,
		  .length = 0,
		  .invalid = false
  };
  Comm_Send(&serial, &readyPacket);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  while (1)
  {

	  Comm_Process(&serial);
	  RobotSystem_Calculate(&robot);

	  CommandHandler_ProcessCommand(&serial, &robot);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
	RobotSystem_InterruptHandler(&robot, htim);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	IMU_InterruptHandler(&imu, GPIO_Pin);

}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size) {
	Comm_Receive(&serial, 0, size);
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
//  if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
//  {
//    /* Retreive Rx messages from RX FIFO0 */
//    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
//    {
//    /* Reception Error */
//    Error_Handler();
//    }
//    if (HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
//    {
//      /* Notification Error */
//      Error_Handler();
//    }
//  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
