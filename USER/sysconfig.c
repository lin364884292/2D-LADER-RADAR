/**
  * @file        sysconfig.c
  * @author      陈维
  * @version     V01
  * @date        2016.09.21
  * @brief       
  * @note        系统配置相关
  *
  * @attention   COYPRIGHT INMOTION ROBOT
  **/
#include "sysconfig.h"
#include "pro.h"

//用来在flash中生成一个key
const uint32_t __attribute__((at(CONFIG_BASE_ADDRESS + 12))) KeyStore = 0x5A2F5D81;
uint16_t CCD_DataBuffer[PIXEL_1_FPS];  
ConfigTypeDef SystemConfig = {0,144,1800,0x5A2F5D81,0};
uint32_t RecCounter = 0;
//像素偏移量
float PixOffset = -51;

//接收的package
PackageDataStruct RecPackage = {PACK_NULL,NULL,0,NULL,NULL};

//待发送距离数据缓存
ComBufferTypeDef ComBuffer;
LidarDataTypeDef LidarData;

void InitSystemConfig(void)
{
    uint16_t i = 0;
    
    
    RecPackage.DataID = PACK_NULL;
    RecPackage.DataInBuff = ComBuffer.RxBuffer;
    RecPackage.DataOutLen = &RecCounter;

    for(i=0;i<360;i++)
    {
        LidarData.PointData[i].Distance = i*10;
    }
}

///**
//  * @brief  将PixOffset等配置数据写入Flash
//  * @note   存储方式为按32bit存储
//  * @param  None
//  * @retval None
//  */
//void WriteConfig(void)
//{
//	uint32_t *ptr = (uint32_t *)&SystemConfig;
//	uint32_t i;
//    FLASH_Status flash_status;
//	
//    //将float型转为无符号16bit整型，不能直接转换
//    SystemConfig.PixOffset = (uint32_t)(s16)(PixOffset*10);	
//	SystemConfig.KeyStore = 0x5A2F5D81;
//	
//	//记录当前温度adc_value
//	SystemConfig.TempAdcValue = Temperature;
//    
//	CloseUsedInterrupt();
//	FLASH_Unlock();
//    FLASH_DataCacheCmd(DISABLE);
//    FLASH_ClearFlag(FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
//    flash_status = FLASH_EraseSector(FLASH_Sector_2,VoltageRange_3);
//    FLASH_ClearFlag(FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
//	
//	for( i = 0 ; i < sizeof(SystemConfig); i+=4)
//	{
//		FLASH_ProgramWord(CONFIG_BASE_ADDRESS + i,*ptr);
//		ptr ++;
//	}
//	
//    FLASH_DataCacheCmd(ENABLE);
//    FLASH_Lock();
//    flash_status = flash_status;
//	
//	RecoverUsedInterrupt();
//}

///**
//  * @brief  读取Config
//  * @param  None
//  * @retval None
//  */
//void ReadConfig(void)
//{
//	SystemConfig = *((ConfigTypeDef *)CONFIG_BASE_ADDRESS);	

//	if(SystemConfig.PixOffset == 0xffffffff)
//	{
//		SystemConfig.PixOffset = 0;
//	}
//	
//    PixOffset = (s16)SystemConfig.PixOffset;
//    PixOffset /= 10;
//	
//	if(SystemConfig.AngleOffset == 0xffffffff) //默认值
//		SystemConfig.AngleOffset = 144;
//	
//	if(SystemConfig.GivenSpeed == 0xffffffff)
//		SystemConfig.GivenSpeed = 2520;
//	
//	if(SystemConfig.TempAdcValue == 0xffffffff)
//		SystemConfig.TempAdcValue = 4096 / 2 + 300;
//}

///**
//  * @brief  屏蔽目前在用的所有中断
//  * @param  None
//  * @retval None
//  */
//void CloseUsedInterrupt(void)
//{
//    DMA_ITConfig(DMA2_Stream1, DMA_IT_TC, DISABLE);	
//    TIM_ITConfig(TIM14, TIM_IT_Update, DISABLE);
//    USART_ITConfig(USART1, USART_IT_IDLE, DISABLE);
//    DMA_ITConfig(DMA2_Stream7, DMA_IT_TC, DISABLE);
//    DMA_ITConfig(DMA2_Stream5, DMA_IT_TC, DISABLE);
//    TIM_ITConfig(TIM3, TIM_IT_CC4, DISABLE);	
//}

///**
//  * @brief  恢复目前在用的所有中断
//  * @param  None
//  * @retval None
//  */
//void RecoverUsedInterrupt(void)
//{
//    DMA_ITConfig(DMA2_Stream1, DMA_IT_TC, ENABLE);	
//    TIM_ITConfig(TIM14, TIM_IT_Update, ENABLE);
//    USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);
//    DMA_ITConfig(DMA2_Stream7, DMA_IT_TC, ENABLE);
//    DMA_ITConfig(DMA2_Stream5, DMA_IT_TC, ENABLE);
//    TIM_ITConfig(TIM3, TIM_IT_CC4, ENABLE);	
//}



/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
