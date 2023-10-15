/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "main.h"
#include "adc.h"
#include "can.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lkmoto.h"
#include "can.h"
#include "usart.h"
#include "gpio.h"
#include "stdio.h"
#include "ids830can.h"
#include "Sensors_reading.h"
#include "fourier_series_traj_exciting.h"
#include "esp8266.h"

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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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
  MX_CAN_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART3_UART_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
	
	//WIFI模块初始化
	HAL_UART_Receive_IT(&huart3, &temp_rx, 1);
	HAL_TIM_Base_Start_IT(&htim4);
	
	if(esp8266_client_config())
	{
		printf("WIFI模块客户端配置失败！\r\n");
	}
	
//	CAN_Filter_Init();
//	traj_exciting_init();
//	
////运行电机motor1~motor5
// 	for(int i=2;i<=6;i++)
// 	{
// 		motor_run(i);
// 	}

#ifdef SET_ZERO_POSITION
//写入当前位置到ROM作为零点(多次写入会影响芯片寿命，不建议频繁使用)
 	HAL_Delay(1000);
 	for(int i=1;i<=6;i++)
 	{
 		write_current_position_to_rom(i);
 		HAL_Delay(500);
 	}
	printf("zero point set successfully!");
#endif

////外骨骼初始位置	
//  ske_base_position();

////先读取一次各关节数据	
//	for(int i=1; i<=6; i++)
//	{
//		if(i == 1)
//		{
//			LinearActuator_read_position(i);
//			LinearActuator_read_CurrentandSpeed(i);
//			pressure_SensorReading();
//		}
//		else
//			read_status2(i);
//			read_angle(i);
//	}
//	
////开启定时器中断：每隔0.05s发送一次控制命令
//	HAL_TIM_Base_Start_IT(&htim2);
//  printf("start!!!\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
  while (1)
  {
//		esp8266_Connect_IOTServer();
//		for(int i=1; i<=6; i++)
//		{
//			if(i == 1)
//			{
//				LinearActuator_read_position(i);
//				LinearActuator_read_CurrentandSpeed(i);
//				pressure_SensorReading();
//			}
//			else
//				read_status2(i);
//			  read_angle(i);
//		}
//		if(motor_control_k > 400)
//		{
//			break;
//		}
		
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  }
//	HAL_Delay(500);

//	for(int i=1; i<=6; i++)
//	{
//		if(i == 1)
//			LinearActuator_startRun_maxspeed_position(i, 0, 30);
//		
//		else if(i == 4)
//			angle_close_loop_with_speed(i, 0, 30);
//		else if(i == 5)
//			angle_close_loop_with_speed(i, 0, 30);
//		else if(i == 6)
//			angle_close_loop_with_speed(i, 0, 30);
//		
//		else
//			angle_close_loop_with_speed(i, 0, 30);
//	}
//	for(int count=1; count<40; count++)
//	{
//		for(int i=1; i<=6; i++)
//		{
//			if(i == 1)
//			{
//				LinearActuator_read_position(i);
//				LinearActuator_read_CurrentandSpeed(i);
//				pressure_SensorReading();
//			}
//			else
//				read_status2(i);
//			  read_angle(i);
//		}
//		HAL_Delay(100);
//	}

//	printf("exciting traj experiment end!!!\n");
	
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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
