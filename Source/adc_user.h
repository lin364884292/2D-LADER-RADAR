/**
******************************************************************************
* @file    adc_user.h
* @author  linjiawei
* @date    2016/12/29
* @brief   ADC 驱动
* @attention Copyright (C) 2016 Inmotion Corporation
******************************************************************************
*/


#ifndef __ADC_USER_H__
#define __ADC_USER_H__

#include "stm32_lib.h"
#include "adc.h"
/**
  *@name ADC1的DMA相关宏定义
  *@{
*/
   
#define RANK_VIDEO         0    /** 实测差分信号线传感器所使用的转换通道1 */ 
#define RANK_ANA_TEST      1    /** 测试差分信号参考电压的所使用的转换通道 */   
#define RANK_ADC1_NUMBER   2    /** 采集参考电压的所使用的转换通道 */   
#define RANK_MAX           8    /** 转换通道数最大值 */
/**
  *@}
*/

/**
  *@name ADC2的DMA相关宏定义
  *@{
*/
   
#define RANK_P3V3_ADC      0    /** 外部3.3V传感器所使用的转换通道1 */ 
#define RANK_LASER         1    /** 激光雷达电流电压的所使用的转换通道4 */   
#define RANK_ADC2_NUMBER   2    /** ADC2所采用的adc通道总数  */
#define RANK_MAX           8    /** 转换通道数最大值 */
/**
  *@}
*/

#define ADC_FULL_VALUE  0x0FFF   /** ADC采集的溢出数值 */
#define ADC_FULL_V      3300     /** ADC采集的溢出电压 3.3V */

#define VOL_SIZE RANK_MAX
extern s32 ADC1_Buffer[RANK_ADC1_NUMBER];
extern s32 ADC2_Buffer[RANK_ADC2_NUMBER];

s32 ADC_LASER(void);
s32 ADC_GetValue(u8 rank);
void ADC_Init(ADC_HandleTypeDef* hadc);

#endif
