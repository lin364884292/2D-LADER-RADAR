/**
******************************************************************************
* @file    gpio_user.c
* @author  linjiawei
* @date    2016/12/29
* @brief   GPIO 相关驱动
* @attention Copyright (C) 2016 Inmotion Corporation
******************************************************************************
*/

#include "gpio_user.h"
#include "main.h"

/**
* @brief  Led流水灯
* @param  None
* @retval None
* @note  每执行一次翻转一次led引口的电平
*/
void LedToggle(void)
{
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
}




