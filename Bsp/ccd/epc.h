/**
  * @file        epc.h
  * @author      陈维
  * @version     V01
  * @date        2017.01.11
  * @brief       epc芯片驱动
  * @note
  *
  * @attention   COYPRIGHT INMOTION ROBOT
  */

#ifndef _EPC_H_
#define _EPC_H_

#include "stm32f3xx_hal.h"

typedef struct
{
    uint16_t Left;
    uint16_t Right;
}TempSensorTypeDef;


void StartCCDCapture(void);
void StartRead(void);
void ClearData(void);
void ClearPix(void);


extern uint8_t DataReady;
#endif



