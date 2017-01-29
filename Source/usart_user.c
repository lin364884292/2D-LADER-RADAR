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
#include "usart.h"
#include "stdio.h"
#include "stm32f3xx_hal_uart.h"
#include "rec.h"

static void UART_DMATransmitCplt(DMA_HandleTypeDef *hdma);
extern DMA_HandleTypeDef hdma_usart1_tx;

//加入以下代码,支持printf函数,而不需要选择use MicroLIB
#pragma import(__use_no_semihosting)

//标准库需要的支持函数
struct __FILE
{
    int handle;
};

FILE __stdout;

////定义_sys_exit()以避免使用半主机模式
//_sys_exit(int x)
//{
//    x = x;
//}

//重定向fputc函数,调试用
int fputc(int ch, FILE *f)
{
    while ((USART1->ISR & 0X40) == 0);
    USART1->RDR = (u8)ch;
    return ch;
}

/**
* @brief  串口 DMA 发送接口
* @param  huart      : 串口句柄
* @param  pData      : 数据指针
* @param  Size       : 数据长度
* @param  tx_tc_flag : 发送完成 flag 偏移
* @retval None
*/
HAL_StatusTypeDef USER_UART_Transmit_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, u32 tx_tc_flag)
{
    if(__HAL_DMA_GET_FLAG(huart->hdmatx, tx_tc_flag))
    {
        if(huart->gState == HAL_UART_STATE_BUSY_TX_RX)
        {
            huart->gState = HAL_UART_STATE_BUSY_TX;
        }
        else
        {
            huart->gState = HAL_UART_STATE_READY;
        }
        
        __HAL_UNLOCK(huart->hdmatx);
        __HAL_DMA_CLEAR_FLAG(huart->hdmatx, tx_tc_flag);
    }
    return HAL_UART_Transmit_DMA(huart, pData, Size);
}

/**
  * @brief  串口1中断服务程序
  * @param  None
  * @retval None
  */

void USART1_IRQHANDLER_USER (UART_HandleTypeDef *huart , DMA_HandleTypeDef *hdma)
{
    u32 temp;
    if((__HAL_UART_GET_IT( huart , UART_IT_IDLE)) != RESET) //空闲线路中断
    {
    u16 data_counter = 0;
    //清除IDLE标志位的软件序列：读SR，接着读DR
    temp = USART1->ISR;
    temp = USART1->RDR;
    
    temp = (__HAL_DMA_GET_COUNTER(hdma));
    
    data_counter = RX_LEN - temp;
    		//0~7
//    RecPackage.DataID = PACK_NULL;
//	RecPackage.DataInBuff = ComBuffer.RxBuffer; 
//	RecPackage.DataInLen = data_counter;
        
//  if (Unpacking(&RecPackage) == PACK_OK)
//		{
//			DMA_Cmd(DMA2_Stream5, DISABLE);
//			DMA_ClearFlag(DMA2_Stream5, DMA_FLAG_TCIF5);
//			DMA_SetCurrDataCounter(DMA2_Stream5, RX_LEN);
//			DMA_Cmd(DMA2_Stream5, ENABLE);
//		}  
    }
    
    if(__HAL_UART_GET_IT( huart , UART_IT_IDLE) != RESET)
    {
        temp = USART1->RDR; 
        USART1->RDR = temp;
    }
}

/** 
  * @brief  串口1 DMA发送中断服务程序     TX
  * @note   单次传输模式，每次传送完成后，需要软件清除TCIF标志位
  * @param  None
  * @retval None
  * 此段函数似乎并没有任何作用，cube生成的时候似乎已经有进行清除标志位
  */
void DMA2_Stream7_IRQHandler( DMA_HandleTypeDef *hdma , UART_HandleTypeDef *huart )
{
//    if (DMA_GetFlagStatus(DMA2_Stream7, DMA_FLAG_TCIF7) == SET)
//    {
//        DMA_ClearFlag(DMA2_Stream7, DMA_FLAG_TCIF7);
//    }
    if(__HAL_DMA_GET_FLAG(hdma, __HAL_DMA_GET_TE_FLAG_INDEX(hdma)) != RESET)    /* Set the UART DMA transfer complete callback */
    {
        __HAL_DMA_CLEAR_FLAG(huart->hdmatx , DMA_FLAG_TC4);
    }
    
}

/**
  * @brief  串口1 DMA接收中断服务程序   RX  
  * @note   在Idle中有将DMA的计数器清零，所以这个中断发生，说明UART发过来的数据有误
  * @param  None
  * @retval None
  */
void DMA2_Stream5_IRQHandler(DMA_HandleTypeDef *hdma, UART_HandleTypeDef *huart )
{
     if(__HAL_DMA_GET_FLAG(hdma, __HAL_DMA_GET_TE_FLAG_INDEX(hdma)) != RESET)
    {
        __HAL_DMA_CLEAR_FLAG(huart->hdmarx , DMA_FLAG_TC5);
        
        HAL_DMA_Abort( hdma );
        
//        __HAL_DMA_DISABLE ( hdma );
//        hdma->Instance->CNDTR = 0;
//        __HAL_DMA_ENABLE ( hdma );
                
    }
}


