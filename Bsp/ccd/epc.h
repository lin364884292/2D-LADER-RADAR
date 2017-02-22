
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
