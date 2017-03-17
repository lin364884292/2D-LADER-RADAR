/**
******************************************************************************
* @file    AVG_Filter.h
* @author  
* @date    
* @brief   均值滤波器
******************************************************************************
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __AVG_FILTER_H__
#define __AVG_FILTER_H__

/* Includes ------------------------------------------------------------------*/
#include "stm32_lib.h"

/* Exported types ------------------------------------------------------------*/
typedef struct AvgFilterInt32DefStruct
{
	s32 *delay;     ///< 缓存区指针
	u16 tap;        ///< 缓存区尺寸
    u16 index;
    s64 sum;
}AvgFilterInt32Def;

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
s32 AVG_Filter_s32(AvgFilterInt32Def *filter, s32 invar) ;
void AVG_Filter_Clear_s32(AvgFilterInt32Def *filter);
void AVG_LINE_Filter_s32(uint16_t data[]);
void Sliding_average_filter_s32(AvgFilterInt32Def *filter, s32 invar);

/* Exported variables ------------------------------------------------------- */

#endif
