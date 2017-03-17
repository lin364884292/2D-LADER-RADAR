/**
******************************************************************************
* @file    Median_Filter.h
* @author  
* @date    
* @brief   均值滤波器
******************************************************************************
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MEDIAN_FILTER_H__
#define __MEDIAN_FILTER_H__

/* Includes ------------------------------------------------------------------*/
#include "stm32_lib.h"

/* Exported types ------------------------------------------------------------*/
typedef struct MedianFilterInt32DefStruct
{
	s32 *delay;     ///< 缓存区指针
	u16 tap;        ///< 缓存区尺寸
    u16 index;
    s64 sum;
}MedianFilterInt32Def;

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
//s32 Median_Filter_s32(MedianFilterInt32Def *filter, s32 invar) ;

//void Median_Filter_Clear_s32(MedianFilterInt32Def *filter);
void Median_Filter_s32(uint16_t data[]);
/* Exported variables ------------------------------------------------------- */

#endif
