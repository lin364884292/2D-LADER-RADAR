/**
******************************************************************************
* @file    usart_user.h
* @author  linjiawei
* @date    2017/1/3
* @brief   串口驱动
* @attention Copyright (C) 2016 Inmotion Corporation
******************************************************************************
*/

#ifndef __USART_USER_H__
#define __USART_USER_H__

#include "stm32_lib.h"
#include "usart.h"

#define RX_ALMOST_FULL 	    16
#define TX_LEN DATA_AMOUNT
#define RX_LEN 2048

HAL_StatusTypeDef USER_UART_Transmit_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, u32 tx_tc_flag);

#endif

