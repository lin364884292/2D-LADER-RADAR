/**
******************************************************************************
* @file    delay.c
* @author  linjiawei
* @date    2016/12/29
* @brief   延时函数
* @attention Copyright (C) 2016 Inmotion Corporation
******************************************************************************
*/
#include "delay_user.h"
#include "stm32_lib.h"
#include "sys.h"

static u8  fac_us = 0;//us延时倍乘数			   
static u16 fac_ms = 0;//ms延时倍乘数,在ucos下,代表每个节拍的ms数


/**
  * @brief  延时函数初始化
  * @param  SYSCLK: 系统时钟
  * @retval None
  */
void Delay_Init(u8 SYSCLK)
{
    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK_DIV8);
    fac_us = SYSCLK / 8;
    fac_ms = (u16)fac_us * 1000;
}


/**
  * @brief  微秒延时
  * @param  nus: 延时n us
  * @retval None
  */
void Delay_us(u32 nus)
{
    u32 temp;
    SysTick->LOAD = nus*fac_us; //时间加载	  		 
    SysTick->VAL = 0x00;        //清空计数器
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;          //开始倒数 
    do
    {
        temp = SysTick->CTRL;
    } while ((temp & 0x01) && !(temp&(1 << 16)));//等待时间到达   
    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;       //关闭计数器
    SysTick->VAL = 0X00;       //清空计数器	 
}

/**
  * @brief  Delay_ms函数调用
  * @param  nms: 延时n ms
  * @retval None
  */
static void Delay_xms(u16 nms)
{
    u32 temp;
    SysTick->LOAD = (u32)nms*fac_ms;//时间加载(SysTick->LOAD为24bit)
    SysTick->VAL = 0x00;           //清空计数器
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;          //开始倒数  
    do
    {
        temp = SysTick->CTRL;
    } while ((temp & 0x01) && !(temp&(1 << 16)));//等待时间到达   
    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;       //关闭计数器
    SysTick->VAL = 0X00;       //清空计数器	  	    
}

/**
  * @brief  毫秒延时
  * @param  nms: 延时n ms
  * @retval None
  */
void Delay_ms(u16 nms)
{
    u8 repeat = nms / 540;	//这里用540,是考虑到某些客户可能超频使用,
    //比如超频到248M的时候,delay_xms最大只能延时541ms左右了
    u16 remain = nms % 540;
    while (repeat)
    {
        Delay_xms(540);
        repeat--;
    }
    if (remain)Delay_xms(remain);

}
