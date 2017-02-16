/**
******************************************************************************
* @file    usart_user.c
* @author  linjiawei
* @date    2017/1/3
* @brief   串口驱动
* @attention Copyright (C) 2016 Inmotion Corporation
******************************************************************************
*/

#include "usart_user.h"
#include "sysconfig.h"
#include "usart.h"
#include "stdio.h"
#include "stm32f3xx_hal_uart.h"
#include "rec.h"

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;

static void SendWholePixel(void)
{
	u32 out_len = 0;

	PackageDataStruct package;
    CCD_DataBuffer[2] = DEBUG_GET_WHOLE_PIXEL_VALUE;
    
	package.DataID = PACK_DEBUG_MODE;
	package.DataInBuff = (u8*)&CCD_DataBuffer[2];
	package.DataInLen = (PIXEL_1_FPS - 2)*2;
	package.DataOutBuff = ComBuffer.TxBuffer;
	package.DataOutLen = &out_len;
	
    Package(package);
    
    HAL_UART_Transmit(&huart1,ComBuffer.TxBuffer,out_len,10);    
}


/**
  * @brief  处理串口发过来的数据包
	* @note   发过来的数据有两种：
			1，配置命令: 固定16bit数据
			2，更新：数据长度不固定
	* @param  None
  * @retval None
  */
void HandleCmd(void)
{
    if (RecPackage.DataID != PACK_NULL)
    {
        s16 config_data = *(s16 *)RecPackage.DataOutBuff;
        
		switch (RecPackage.DataID)
        {
        case PACK_SET_PIXOFFSET:
            PixOffset = config_data;
            break;

        case PACK_SET_SPEED:
            SystemConfig.GivenSpeed = config_data;
            break;

        case PACK_GET_GYROSCOPE_DATA:
            
            break;

        case PACK_FLASH_CONFIG:
			
            break;
		
        case PACK_FIRMWARE_UPDATE:
			
            break;
		
		case PACK_ANGLE_OFFSET:
			SystemConfig.AngleOffset = config_data;
			break;
		
		case PACK_CONTROL_LASER:
			
			break;
			
		case PACK_SELF_TEST:
			
			break;
		
		case PACK_DEBUG_MODE:
           while(1)
           {               
             SendWholePixel();
              while(HAL_UART_GetState(&huart1) == HAL_UART_STATE_BUSY);
           }
		    break;

		case PACK_START_ROTATE:
			
		    break;		
		
		default:
            break;
        }
		
    }
			RecPackage.DataID = PACK_NULL;	
}


void UART_SetDMA(void)
{
	if(HAL_UART_Receive_DMA(&huart1,ComBuffer.RxBuffer,DATA_AMOUNT) == HAL_OK)
	{
		//UserPrintf("Info:Com1 OK\n");
	}
	else
	{
		//UserPrintf("Error:Com1 NG\n");
	}	
}

void UART_RestartDMA(void)
{
	__HAL_DMA_DISABLE(&hdma_usart1_rx);
	hdma_usart1_rx.Instance->CNDTR = DATA_AMOUNT;
	__HAL_DMA_ENABLE(&hdma_usart1_rx);
}

/**
	* @brief  串口DMA接收完成中断的回调函敿
	* @note   每次解包完成后会重置DMA，所以只有当串口有错误发生才会触发此回调
	* @param  None
	* @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	printf("Error:Com1 overflow,please check serial port\n");
	UART_RestartDMA();
}


