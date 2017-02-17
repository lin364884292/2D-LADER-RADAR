/**
******************************************************************************
* @file    adc_user.c
* @author  linjiawei
* @date    2016/12/29
* @brief   ADC 驱动
* @attention Copyright (C) 2016 Inmotion Corporation
******************************************************************************
*/

#include "adc_user.h"

s32 ADC1_Buffer[RANK_ADC1_NUMBER];
s32 ADC2_Buffer[RANK_ADC2_NUMBER];
/**
* @brief  初始化ADC，DMA通道
* @param  hadc：ADC口
* @retval None
*/
void ADC_Init(ADC_HandleTypeDef* hadc)
{
    if(hadc == &hadc1)
    {  
        HAL_ADCEx_Calibration_Start(hadc,ADC_DIFFERENTIAL_ENDED);
        HAL_ADC_Start_DMA(hadc, (u32*)ADC1_Buffer, RANK_ADC1_NUMBER);
    }
//    if(hadc == &hadc2)
//    {
//        HAL_ADCEx_Calibration_Start(hadc,ADC_SINGLE_ENDED);
//        HAL_ADC_Start_DMA(hadc, (u32*)ADC2_Buffer, RANK_ADC2_NUMBER);
//    }    
}

/**
* @brief  ADC2_single采集电压值
* @param  rank：DMA通道口
* @retval None
* @return 该通道口的电压值
*/    
s32 ADC_GetValue(u8 rank)
{
    if(rank < RANK_MAX)
    {
      return ((s32)ADC2_Buffer[rank]);  //板子到时需要验证其是否拥有浮点运算器
    }
    return 0;
}

/**
* @brief  LASER_ADC采集电压值
* @param  idx :DMA通道口
* @retval None
* @note　因为地侧的ADC只用到四个，单独开来
*/
s32 ADC_LASER(void)
{
    return  ADC_GetValue(RANK_LASER);
}
