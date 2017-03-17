/**
******************************************************************************
* @file    Median_Filter.c
* @author  
* @date    
* @brief   中值滤波器
******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "Median_Filter.h"
#include "sysconfig.h"
static int Temp;
/**
* @brief  清空滤波器存储值
* @param  filter: 滤波器
* @retval None
*/
void Median_Filter_Clear_s32(MedianFilterInt32Def *filter)
{
	u16 i;
	for(i = 0 ; i < filter->tap ; i ++)
		filter->delay[i] = 0;	
}

void Median_Filter_s32(uint16_t data[])
{
    Temp=5;
    uint16_t a[5];//暂存固定长度冒泡排序的数据
    int  ii,jj,kk,bTemp;
    int i,j,Temporary;//冒泡排序用的循环变量
    if ((Temp & 1) > 0)
    {
        // 数组有奇数个元素，返回中间一个元素
        bTemp = (Temp-1)/2;
    }
    
    for(kk =bTemp;kk<(PIXEL_1_FPS-bTemp);kk++)
    {
        for(jj=0;jj<Temp;jj++)
        {
            a[jj]=data[kk-bTemp+jj];
        }
        
        for (j = 0; j < Temp - 1; j ++)
        {
            for (i = 0; i < Temp - j - 1; i ++)
            {
            if (a[i] > a[i + 1])
                {
                    // 互换
                    Temporary = a[i];
                    a[i] = a[i + 1];
                    a[i + 1] = Temporary;
                }
            }
        }
        data[kk]=a[bTemp];
    }
}
