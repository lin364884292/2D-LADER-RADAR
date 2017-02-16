/**
  * @file        sysconfig.c
  * @author      林佳伟
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



/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
