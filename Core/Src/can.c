/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
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
#include "can.h"

/* USER CODE BEGIN 0 */

#include "usart.h"
#include "stdio.h"
#include "stdlib.h"
#include "lkmoto.h"
#include "ids830can.h"
CAN_TxHeaderTypeDef hCAN1_TxHeader; //CAN1发送消息
CAN_RxHeaderTypeDef hCAN1_RxHeader; //CAN1接收消息
	
CAN_FilterTypeDef sFilterConfig;//CANl滤波器
/* USER CODE END 0 */

CAN_HandleTypeDef hcan;

/* CAN init function */
void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 4;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_4TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = ENABLE;
  hcan.Init.AutoWakeUp = ENABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */


  /* USER CODE END CAN_Init 2 */

}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**CAN GPIO Configuration
    PA11     ------> CAN_RX
    PA12     ------> CAN_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();

    /**CAN GPIO Configuration
    PA11     ------> CAN_RX
    PA12     ------> CAN_TX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

//过滤器配置	及启动can，开启接受终端
void CAN_Filter_Init(void)
{
	sFilterConfig.FilterActivation = ENABLE;//开启过滤器
	sFilterConfig.FilterBank = 0;//过滤器0 这里可设0-13
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;//采用掩码模式
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;//采用32位掩码模式
	sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;//采用FIFO0

	sFilterConfig.FilterIdHigh = 0x0000; //设置过滤器ID高16位
	sFilterConfig.FilterIdLow = 0x0000;//设置过滤器ID低16位
	sFilterConfig.FilterMaskIdHigh = 0x0000;//设置过滤器掩码高16位
	sFilterConfig.FilterMaskIdLow = 0x0000;//设置过滤器掩码低16位
	if(HAL_CAN_ConfigFilter(&hcan,&sFilterConfig) != HAL_OK)//初始化过滤器
	{
		Error_Handler();
	}
	if(HAL_CAN_Start(&hcan) != HAL_OK)//启动can
	{
		Error_Handler();
	}
	if(HAL_CAN_ActivateNotification(&hcan,CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)//开启接受邮箱0挂起中断
	{
		Error_Handler();
	}
}

/*******************************************************************************
* Function Name  : HAL_CAN_RxFifo0MsgPendingCallback
* Description    : 消息接收回调函数
* Input          : hcan
* Output         : None
* Return         : None
****************************************************************************** */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  uint8_t aRxData[8]={0};
  if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &hCAN1_RxHeader, aRxData) == HAL_OK)
  {
		if(aRxData[1] == 0x2b && aRxData[2] == 0xe8 && aRxData[5] == 0xe9)
		{
			for(int i=0; i<8; i++)
			{
				ids830_position[i] = aRxData[i];
			}
//			HAL_UART_Transmit(&huart1, ids830_position, 8, 10);
		}
		else if(aRxData[1] == 0x2b && aRxData[2] == 0xe2 && aRxData[5] == 0xe4)
		{
			for(int i=0; i<8; i++)
			{
				ids830_currentAndspeed[i] = aRxData[i];
			}
//			HAL_UART_Transmit(&huart1, ids830_currentAndspeed, 8, 10);
		}
		else
		{
			for(int i=0;i<8;i++)
			{
				CAN_motor_data[i] = aRxData[i];
			}
//			HAL_UART_Transmit(&huart1, CAN_motor_data, 8, 10);
		}
  }
}


/* USER CODE END 1 */
