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
#include "Median_Filter.h"
#include "AVG_filter.h"
#include "kalman_filter.h"
#include "algorithm.h"
#include "delay_user.h"

//static PixCurveCoor MaxPixInfo;
//static u8 CmdFlag[16];

static float PixIndex = 0;
static float Distance = 0;

static float distance_avg = 0;
static float pix_index_avg =0;

static uint16_t Buffer_2[PIXEL_1_FPS];
//static u8 QuitFlag = 0;

//初始化卡尔曼滤波器
extern kalman1_state kalman1;

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;

static u8 temp[PIXEL_1_FPS+1];
static uint16_t Buffer_temp[PIXEL_1_FPS];

static float pix;
void FFT_filter(u16 *data)
{
    int i;
    int sum=0;
    int n=0;
    
    int ref_line = 2000;
    int VALID_PIX_START_1 = 5;
    int VALID_PIX_END_1 = 1027;
    for (i=0;i<VALID_PIX_START_1;i++)
    {
        data[i]=0;
    }
    for (i = VALID_PIX_START_1; i < VALID_PIX_END_1; i++) 
    {
        if(data[i]>ref_line)
        {
            data[i]=1;
            sum += i;
            n++;
        }
        else
            data[i]=0;       
    }
    pix = (float)sum/n;
}



/**
  * @brief  发送debug数据
  * @param  None
  * @retval None
  */
static void SendData_2(DebugTypeDef id , u8 *data_buffer , u32 len)
{
    u32 i = 0;
    PackageDataStruct package;
    u32 out_len;
      
	temp[0] = (u8)id;	
    
    memcpy(temp+1,data_buffer,len);
    
    package.DataID = PACK_DEBUG_MODE;
    package.DataInBuff = temp;
    package.DataInLen = len+1;
    package.DataOutBuff = ComBuffer.TxBuffer;
    package.DataOutLen = &out_len;
    
    Package(package);
    
    HAL_UART_Transmit(&huart1,ComBuffer.TxBuffer,out_len,10);    

}


//uint16_t CCD_DataBuffer_2[PIXEL_1_FPS];  
static void SendWholePixel(void)
{
    int i;
	u32 out_len = 0;
    u32 len = sizeof(CCD_DataBuffer);
	PackageDataStruct package;
//    CCD_DataBuffer[0] = DEBUG_GET_WHOLE_PIXEL_VALUE;
    
    memcpy(Buffer_temp,CCD_DataBuffer,sizeof(CCD_DataBuffer));   
    
    
//    Median_Filter_s32(Buffer_temp);
//    AVG_LINE_Filter_s32(Buffer_temp);
//    kalman1_line_filter(&kalman1,Buffer_temp);
//    FFT_filter(Buffer_temp);
    
    temp[0] = DEBUG_GET_WHOLE_PIXEL_VALUE;
    memcpy(temp+1,Buffer_temp,len);
//     memcpy(temp+1,CCD_DataBuffer,len);
	package.DataID = PACK_DEBUG_MODE;
	package.DataInBuff = temp;
	package.DataInLen = len+1;
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
               memcpy(Buffer_2,CCD_DataBuffer,sizeof(CCD_DataBuffer));    
               //灰度质心法求得像素质心位置
               
               GetCentroid(Buffer_2, &PixIndex);
                Get_Average_Centroid(Buffer_2, &pix_index_avg);
               
                //调整像素偏移量(校准得来)
                pix_index_avg += 22;
                PixIndex += PixOffset;
               
                GetDistance(PixIndex, &Distance);
               
                GetDistance(pix_index_avg, &distance_avg);
//               
//               SendWholePixel();  
//              SendData_2(DEBUG_GET_WHOLE_PIXEL_VALUE,(u8*)CCD_DataBuffer,sizeof(CCD_DataBuffer));
              
            u16 pix_index = PixIndex *10;
			SendData_2(DEBUG_GET_WHOLE_PIXEL_VALUE,(u8 *)&pix_index,sizeof(pix_index));
               
               
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
