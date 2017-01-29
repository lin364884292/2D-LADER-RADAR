/**
******************************************************************************
* @file    delay.h
* @author  linjiawei
* @date    2016/12/29
* @brief   延时函数
* @attention Copyright (C) 2016 Inmotion Corporation
******************************************************************************
*/

#ifndef __DELAY_H
#define __DELAY_H 		

#include "stm32_lib.h"

void Delay_Init(u8 SYSCLK);
void Delay_ms(u16 nms);
void Delay_us(u32 nus);

#endif
