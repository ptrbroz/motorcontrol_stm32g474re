/**
  ******************************************************************************
  * @file    fdcan.c
  * @brief   This file provides code for the configuration
  *          of the FDCAN instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "fdcan.h"

/* USER CODE BEGIN 0 */
#include "hw_config.h"
#include "user_config.h"
#include "math_ops.h"
#include <stdio.h>


/* USER CODE END 0 */

FDCAN_HandleTypeDef hfdcan2;

/* FDCAN2 init function */
void MX_FDCAN2_Init(void)
{

  /* USER CODE BEGIN FDCAN2_Init 0 */

  /* USER CODE END FDCAN2_Init 0 */

  /* USER CODE BEGIN FDCAN2_Init 1 */

  /* USER CODE END FDCAN2_Init 1 */
  hfdcan2.Instance = FDCAN2;
  hfdcan2.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan2.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan2.Init.AutoRetransmission = DISABLE;
  hfdcan2.Init.TransmitPause = DISABLE;
  hfdcan2.Init.ProtocolException = DISABLE;
  hfdcan2.Init.NominalPrescaler = 5;
  hfdcan2.Init.NominalSyncJumpWidth = 1;
  hfdcan2.Init.NominalTimeSeg1 = 12;
  hfdcan2.Init.NominalTimeSeg2 = 4;
  hfdcan2.Init.DataPrescaler = 1;
  hfdcan2.Init.DataSyncJumpWidth = 1;
  hfdcan2.Init.DataTimeSeg1 = 12;
  hfdcan2.Init.DataTimeSeg2 = 4;
  hfdcan2.Init.StdFiltersNbr = 1;
  hfdcan2.Init.ExtFiltersNbr = 1;
  hfdcan2.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN2_Init 2 */

  /* USER CODE END FDCAN2_Init 2 */

}

void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef* fdcanHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(fdcanHandle->Instance==FDCAN2)
  {
  /* USER CODE BEGIN FDCAN2_MspInit 0 */

  /* USER CODE END FDCAN2_MspInit 0 */
    /* FDCAN2 clock enable */
    __HAL_RCC_FDCAN_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**FDCAN2 GPIO Configuration
    PB5     ------> FDCAN2_RX
    PB6     ------> FDCAN2_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* FDCAN2 interrupt Init */
    HAL_NVIC_SetPriority(FDCAN2_IT0_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(FDCAN2_IT0_IRQn);
  /* USER CODE BEGIN FDCAN2_MspInit 1 */

  /* USER CODE END FDCAN2_MspInit 1 */
  }
}

void HAL_FDCAN_MspDeInit(FDCAN_HandleTypeDef* fdcanHandle)
{

  if(fdcanHandle->Instance==FDCAN2)
  {
  /* USER CODE BEGIN FDCAN2_MspDeInit 0 */

  /* USER CODE END FDCAN2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_FDCAN_CLK_DISABLE();

    /**FDCAN2 GPIO Configuration
    PB5     ------> FDCAN2_RX
    PB6     ------> FDCAN2_TX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_5|GPIO_PIN_6);

    /* FDCAN2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(FDCAN2_IT0_IRQn);
  /* USER CODE BEGIN FDCAN2_MspDeInit 1 */

  /* USER CODE END FDCAN2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
void can_rx_init(CANRxMessage *msg){
	msg->filter.FilterID1 = CAN_ID;
	msg->filter.FilterID2 = CAN_ID;
	msg->filter.FilterIndex = 0;
	msg->filter.FilterType = FDCAN_FILTER_DUAL;//FDCAN_FILTER_RANGE; //
	msg->filter.IdType = FDCAN_STANDARD_ID;
	msg->filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	//msg->filter.FilterConfig = FDCAN_FILTER_DISABLE;


	HAL_FDCAN_ConfigFilter(&CAN_H, &msg->filter);

	//HAL_FDCAN_ConfigGlobalFilter(&CAN_H, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
	HAL_FDCAN_ConfigGlobalFilter(&CAN_H, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_REJECT_REMOTE);


	//original settings from Ben's fw:
	/*
	msg->filter.FilterFIFOAssignment=CAN_FILTER_FIFO0; 	// set fifo assignment
	msg->filter.FilterIdHigh=CAN_ID<<5; 				// CAN ID
	msg->filter.FilterIdLow=0x0;
	msg->filter.FilterMaskIdHigh=0xFFF;
	msg->filter.FilterMaskIdLow=0;
	msg->filter.FilterMode = CAN_FILTERMODE_IDMASK;
	msg->filter.FilterScale=CAN_FILTERSCALE_32BIT;
	msg->filter.FilterActivation=ENABLE;
	HAL_CAN_ConfigFilter(&CAN_H, &msg->filter);
	*/
}

void can_tx_init(CANTxMessage *msg){
	msg->tx_header.BitRateSwitch = FDCAN_BRS_OFF;
	msg->tx_header.DataLength = FDCAN_DLC_BYTES_6;
	msg->tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE; //???
	msg->tx_header.FDFormat = FDCAN_CLASSIC_CAN;
	msg->tx_header.IdType = FDCAN_STANDARD_ID;
	msg->tx_header.Identifier = CAN_MASTER; // ?
	msg->tx_header.MessageMarker = 0; //???
	msg->tx_header.TxEventFifoControl = FDCAN_STORE_TX_EVENTS; //???
	msg->tx_header.TxFrameType = FDCAN_DATA_FRAME;

	//Original settings from Ben:
	/*
	msg->tx_header.DLC = 6; 			// message size of 8 byte - DataLength
	msg->tx_header.IDE=CAN_ID_STD; 		// set identifier to standard - TxFrameType
	msg->tx_header.RTR=CAN_RTR_DATA; 	// set data type to remote transmission request? - ???
	msg->tx_header.StdId = CAN_MASTER;  // recipient CAN ID - ??? Probably Identifier
	*/
}

//TODO: Check below for differences in packet structure?

/// CAN Reply Packet Structure ///
/// 16 bit position, between -4*pi and 4*pi
/// 12 bit velocity, between -30 and + 30 rad/s
/// 12 bit current, between -40 and 40;
/// CAN Packet is 5 8-bit words
/// Formatted as follows.  For each quantity, bit 0 is LSB
/// 0: [position[15-8]]
/// 1: [position[7-0]]
/// 2: [velocity[11-4]]
/// 3: [velocity[3-0], current[11-8]]
/// 4: [current[7-0]]
void pack_reply(CANTxMessage *msg, uint8_t id, float p, float v, float t){
    int p_int = float_to_uint(p, P_MIN, P_MAX, 16);
    int v_int = float_to_uint(v, V_MIN, V_MAX, 12);
    int t_int = float_to_uint(t, -I_MAX*KT*GR, I_MAX*KT*GR, 12);
    msg->data[0] = id;
    msg->data[1] = p_int>>8;
    msg->data[2] = p_int&0xFF;
    msg->data[3] = v_int>>4;
    msg->data[4] = ((v_int&0xF)<<4) + (t_int>>8);
    msg->data[5] = t_int&0xFF;
    }

/// CAN Command Packet Structure ///
/// 16 bit position command, between -4*pi and 4*pi
/// 12 bit velocity command, between -30 and + 30 rad/s
/// 12 bit kp, between 0 and 500 N-m/rad
/// 12 bit kd, between 0 and 100 N-m*s/rad
/// 12 bit feed forward torque, between -18 and 18 N-m
/// CAN Packet is 8 8-bit words
/// Formatted as follows.  For each quantity, bit 0 is LSB
/// 0: [position[15-8]]
/// 1: [position[7-0]]
/// 2: [velocity[11-4]]
/// 3: [velocity[3-0], kp[11-8]]
/// 4: [kp[7-0]]
/// 5: [kd[11-4]]
/// 6: [kd[3-0], torque[11-8]]
/// 7: [torque[7-0]]
void unpack_cmd(CANRxMessage msg, float *commands){// ControllerStruct * controller){
        int p_int = (msg.data[0]<<8)|msg.data[1];
        int v_int = (msg.data[2]<<4)|(msg.data[3]>>4);
        int kp_int = ((msg.data[3]&0xF)<<8)|msg.data[4];
        int kd_int = (msg.data[5]<<4)|(msg.data[6]>>4);
        int t_int = ((msg.data[6]&0xF)<<8)|msg.data[7];

        commands[0] = uint_to_float(p_int, P_MIN, P_MAX, 16);
        commands[1] = uint_to_float(v_int, V_MIN, V_MAX, 12);
        commands[2] = uint_to_float(kp_int, KP_MIN, KP_MAX, 12);
        commands[3] = uint_to_float(kd_int, KD_MIN, KD_MAX, 12);
        commands[4] = uint_to_float(t_int, -I_MAX*KT*GR, I_MAX*KT*GR, 12);

        printf("C %.2f", commands[1]);

    }
/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
