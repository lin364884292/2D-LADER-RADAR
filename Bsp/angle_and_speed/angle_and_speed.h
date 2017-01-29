/**
  * @file        angle_and_speed.h
  * @author      陈维
  * @version     V01
  * @date        2016.05.16
  * @brief       测电机转速及计算当前角度
  * @note
  *
  * @attention   COYPRIGHT INMOTION ROBOT
  */

#ifndef __ANGLE_AND_SPEED_H
#define __ANGLE_AND_SPEED_H

#include "stm32f3xx_hal.h"
#include "sysconfig.h"

typedef enum
{
    ZERO_POINT,
    NON_ZERO_POINT
}ZeroFlagType;


#define RISING_EDGE 0x40
#define FALLING_EDGE 0x80
#define WAIT_START 0x00

#define GRATING_AMOUNT 30 // 光栅数量
#define ANGLE_PER_GRATING 360/GRATING_AMOUNT

#define MAX_SPEED 5400  //电机最快能转15hz
void AngleCalculate_Init(void);
uint16_t GetLidarAngle(void);
BOOL IsZeroPoint(void);
uint32_t GetAngularSpeed(void);
void SetPulseWidthThreshold(void);
BOOL IsAllPulseWidthDataReady(void);
BOOL IsLidarStall(void);
BOOL IsTimeToSendSpeed(void);
void SendSpeed(uint16_t speed);
void ResetAngularSpeed(void);
uint8_t GetCurrentGratingIndex(void);
void InitSpeedCapture(void);
#endif
/************************ (C) COPYRIGHT INMOTION ROBOT *****END OF FILE****/
