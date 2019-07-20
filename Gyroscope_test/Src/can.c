/**
  ******************************************************************************
  * File Name          : CAN.c
  * Description        : This file provides code for the configuration
  *                      of the CAN instances.
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "can.h"
#include "ahrs.h"
#include "gpio.h"

/* USER CODE BEGIN 0 */
void CAN1_Send_Msg(uint32_t id, int16_t data1, int16_t data2, int16_t data3, int16_t data4, uint8_t len);
/* USER CODE END 0 */

CAN_HandleTypeDef hcan;
CAN_FilterConfTypeDef  hcan1_filter;

CanTxMsgTypeDef     TxMessage;
CanRxMsgTypeDef     RxMessage;
/* CAN init function */
void MX_CAN_Init(void)
{
   hcan.pTxMsg = &TxMessage;
   hcan.pRxMsg = &RxMessage;
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 4;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SJW = CAN_SJW_1TQ;
  hcan.Init.BS1 = CAN_BS1_5TQ;
  hcan.Init.BS2 = CAN_BS2_3TQ;
  hcan.Init.TTCM = DISABLE;
  hcan.Init.ABOM = DISABLE;
  hcan.Init.AWUM = DISABLE;
  hcan.Init.NART = DISABLE;
  hcan.Init.RFLM = DISABLE;
  hcan.Init.TXFP = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  hcan1_filter.FilterIdHigh=0X0000;
  hcan1_filter.FilterIdLow=0X0000;
  hcan1_filter.FilterMaskIdHigh=0X0000;
  hcan1_filter.FilterMaskIdLow=0X0000;
  hcan1_filter.FilterFIFOAssignment=CAN_FILTER_FIFO0;
  hcan1_filter.FilterNumber=0;
  hcan1_filter.FilterMode=CAN_FILTERMODE_IDMASK;
  hcan1_filter.FilterScale=CAN_FILTERSCALE_32BIT;
  hcan1_filter.FilterActivation=ENABLE;
  hcan1_filter.BankNumber=14;

  if (HAL_CAN_ConfigFilter(&hcan,&hcan1_filter) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_CAN_Receive_IT(&hcan,CAN_FIFO0 );
}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();
  
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
    HAL_NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 4, 0);
    HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
//    HAL_NVIC_SetPriority(CAN1_RX1_IRQn, 5, 0);
//    HAL_NVIC_EnableIRQ(CAN1_RX1_IRQn);
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
    HAL_NVIC_DisableIRQ(CAN1_RX1_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */

void CAN1_Send_Msg(uint32_t id, int16_t data1, int16_t data2, int16_t data3, int16_t data4, uint8_t len)
{
		hcan.pTxMsg->StdId = id;
    hcan.pTxMsg->IDE = CAN_ID_STD;
    hcan.pTxMsg->RTR = CAN_RTR_DATA;
    hcan.pTxMsg->DLC = 2*len;
    hcan.pTxMsg->Data[0] = (uint8_t)(data1 >> 8);
    hcan.pTxMsg->Data[1] = (uint8_t)data1;
    hcan.pTxMsg->Data[2] = (uint8_t)(data2 >> 8);
    hcan.pTxMsg->Data[3] = (uint8_t)data2;
    hcan.pTxMsg->Data[4] = (uint8_t)(data3 >> 8);
    hcan.pTxMsg->Data[5] = (uint8_t)data3;
    hcan.pTxMsg->Data[6] = (uint8_t)(data4 >> 8);
    hcan.pTxMsg->Data[7] = (uint8_t)data4;

    HAL_CAN_Transmit(&hcan,10);
}

static float gyro_yaw_angle, last_gyro_yaw_angle = 0;
void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* hcan)
{
	
		if (hcan->pRxMsg->StdId == 0x401) {
//    case Can_ID:
				last_gyro_yaw_angle = gyro_yaw_angle;
        gyro_yaw_angle = (float)(0.008571428571f)*((int32_t)(hcan->pRxMsg->Data[0]<<24)|(int32_t)(hcan->pRxMsg->Data[1]<<16) | (int32_t)(hcan->pRxMsg->Data[2]<<8) | (int32_t)(hcan->pRxMsg->Data[3])); 
				gyro_yaw_v = (gyro_yaw_angle - last_gyro_yaw_angle);
//        break;
//    default:
//        break;
    }
		
    if (hcan->pRxMsg->StdId == Can_ID) {
//    case Can_ID:
        mode = hcan->pRxMsg->Data[0];
        if(mode == 0x30){
            calibration_time = (hcan->pRxMsg->Data[1]<<8) | hcan->pRxMsg->Data[2];
        }
//        break;
//    default:
//        break;
    }
    __HAL_CAN_ENABLE_IT(hcan, CAN_IT_FMP0);

}
/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
