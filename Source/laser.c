/**
******************************************************************************
* @file    laser.c
* @author  linjiawei
* @date    2016/12/29
* @brief   laser 驱动
* @attention Copyright (C) 2016 Inmotion Corporation
******************************************************************************
*/

//暂时还有定时器7的作用不明需要询问

#include "laser.h"
#include "stm32f3xx_hal_tim.h"
#include "adc_user.h"

#define CKTIM ((u32)180000000)
#define LASER_TIM 100000

extern TIM_HandleTypeDef htim3;

/**
  * @brief  激光电流检测
  * @param  None
  * @retval None
  */
float GetLaserCurrent(void)
{
  float laser_current = 0;
  laser_current = ADC_LASER();
  laser_current = laser_current / 4096 * 3300 / (float)24.9;
  return laser_current;
}

/**
  * @brief  设置gpio为正常输入功能 
  * @param  None
  * @retval 没什么卵用
//*/
//void LaserPC2ConfigToIn(void)
//{
//  GPIO_InitTypeDef GPIO_InitStruct;
//  __HAL_RCC_GPIOA_CLK_ENABLE();
//  GPIO_InitStruct.Pin = PA_LASER_Pin;
//  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//  HAL_GPIO_Init(PA_LASER_GPIO_Port, &GPIO_InitStruct); 
//}

///**
//  * @brief  拉高PC2是为了彻底关断激光，否则会有漏电流. 
//  * @param  None
//  * @retval 设置gpio为正常输出功能，取消其原本PWM输出模式
//*/
//void LaserConfigToOutHigh(void)
//{
//  GPIO_InitTypeDef GPIO_InitStruct;   
//  __HAL_RCC_GPIOA_CLK_ENABLE();
//  GPIO_InitStruct.Pin = PA_LASER_Pin;
//  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//  HAL_GPIO_Init(PA_LASER_GPIO_Port, &GPIO_InitStruct);
//  LASER_ON;
//}


///**
//  * @brief  关闭激光
//  * @param  None
//  * @retval None
//*/
//void LaserOff(void)
//{
//  HAL_TIM_OC_Stop(&htim3,TIM_CHANNEL_1);    
//  LASER_OFF;  
//}

/**
  * @brief  激光PWM调制
  * @param  dutycycle: 激光占空比
  * @retval None
  */
void Laser_Init(uint16_t dutycycle)
{
    TIM_OC_InitTypeDef sConfigOC;
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = dutycycle;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim3,&sConfigOC,TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);  
}

/**
  * @brief  开启激光
  * @param  dutycycle: 激光占空比
  * @retval None
  */
void LaserOn(u8 dutycycle)
{ 
    u32 compareValue;
    
    if(dutycycle == 0)
    {
//      LASER_OFF;        
    }
    else
//       LASER_ON;    
    
    compareValue = ((CKTIM / 2) / LASER_TIM) - 1;
    compareValue = (compareValue + 1) * dutycycle / 100;
    //HAL_TIM_OC_Start(&htim3,TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
}

