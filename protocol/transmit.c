/**
******************************************************************************
* @file    transmit.c
* @author  linjiawei
* @date    2017/1/6
* @brief   打包
* @attention Copyright (C) 2016 Inmotion Corporation
******************************************************************************
*/

#include "transmit.h"  
#include "pro.h"
//#include "sysconfig.h"
//#include "usart.h"
//#include "angle_and_speed.h"
//#include "algorithm.h"


//extern ComBufferTypeDef ComBuffer;
//extern LidarDataTypeDef LidarData;
/* USER CODE BEGIN 0 */


/* USER CODE BEGIN 0 */
/**
  * @brief  将Lidar 2D数据按鿚讯协议打包
  * @param  None
  * @retval None
  */
//static void PackageAndSendTxData(void)
//{   
//	u32 out_len = 0;

//	PackageDataStruct package;

//    //目标速度
//    LidarData.GivenSpeed = SystemConfig.GivenSpeed;
//    //当前速度
//    LidarData.CurrSpeed = GetAngularSpeed();
//    
//	package.DataID = PACK_LIDAR_DATA;
//	package.DataInBuff = (u8*)&LidarData;
//	package.DataInLen = sizeof(LidarDataTypeDef);
//	package.DataOutBuff = ComBuffer.TxBuffer;
//	package.DataOutLen = &out_len;
//	
//    Package(package);
//    
//    HAL_UART_Transmit_DMA(&huart1,ComBuffer.TxBuffer,out_len);

//}

/**
  * @brief  数据打包函数
  * @param  package :待打包数据信息
  * @note   需要确保输入的数据信息正确，特别是out buffer最好要是待打包数据长度的两倍
  * @retval None
  */
ResultTypeDef Package(PackageDataStruct package)
{
	u32 j = 0;
	u32 i = 0;
	SdkProtocolHeaderTypeDef sdk_header;
	u8 *psdk = (u8 *)&sdk_header;
	u8 checksum = 0;
	
	if((package.DataInBuff == NULL) || (package.DataOutBuff == NULL) || (package.DataOutLen == NULL))
		return PACK_FAIL;
	
	sdk_header.DeviceAddr = LIDAR_ADDRESS;
	sdk_header.FunctionCode = package.DataID;
	sdk_header.StartAddr = 0;
	sdk_header.Len = package.DataInLen;

	*(package.DataOutBuff+i ++) = P_HEADER;
	*(package.DataOutBuff+i ++) = P_HEADER;

	for(j = 0 ; j<sizeof(SdkProtocolHeaderTypeDef);j++)
	{
		if((*(psdk+j) == P_CTRL) || (*(psdk+j) == P_HEADER) || (*(psdk+j) == P_TAIL))
		{
			*(package.DataOutBuff+i ++) = P_CTRL;
		}
		*(package.DataOutBuff+i ++) = *(psdk+j);
		checksum += *(psdk+j);
	}
	
	for(j = 0 ; j<package.DataInLen; j++)
	{
		if((*(package.DataInBuff+j) == P_CTRL) || (*(package.DataInBuff+j) == P_HEADER) || (*(package.DataInBuff+j) == P_TAIL))
		{
			*(package.DataOutBuff+i++) = P_CTRL;
		}
		checksum += *(package.DataInBuff+j);
		*(package.DataOutBuff+i++) = *(package.DataInBuff+j);
	}
	
	if((checksum == P_CTRL) || (checksum == P_HEADER) || (checksum == P_TAIL))
	{
		*(package.DataOutBuff+i++) = P_CTRL;
	}
	*(package.DataOutBuff+i++) = checksum;
	
	*(package.DataOutBuff+i++) = P_TAIL;
	*(package.DataOutBuff+i++) = P_TAIL;
	
	*package.DataOutLen = i;
	
	return PACK_OK;
}

  
/************************ (C) COPYRIGHT INMOTION ROBOT *****END OF FILE****/
