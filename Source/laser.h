/**
******************************************************************************
* @file    laser.h
* @author  linjiawei
* @date    2016/12/29
* @brief   laser 驱动
* @attention Copyright (C) 2016 Inmotion Corporation
******************************************************************************
*/

#ifndef __LASER_USER_H__
#define __LASER_USER_H__

#include "stm32_lib.h"
#include "main.h"
#include "gpio.h"

#define LASER_OFF HAL_GPIO_WritePin(PA_LASER_GPIO_Port,PA_LASER_Pin,GPIO_PIN_RESET);
#define LASER_ON  HAL_GPIO_WritePin(PA_LASER_GPIO_Port,PA_LASER_Pin,GPIO_PIN_SET);

#endif
