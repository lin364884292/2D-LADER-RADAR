/**
******************************************************************************
* @file    AVG_Filter.c
* @author  
* @date    
* @brief   均值滤波器
******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "AVG_filter.h"
#include "sysconfig.h"
/**
* @brief  清空滤波器存储值
* @param  filter: 滤波器
* @retval None
*/
void AVG_Filter_Clear_s32(AvgFilterInt32Def *filter)
{
	u16 i;
	for(i = 0 ; i < filter->tap ; i ++)
		filter->delay[i] = 0;	
}

/**
* @brief  滑动平均滤波器(int32)
* @param  filter: 滤波器
* @param  invar : 新值
* @retval 滤波结果值
*/
s32 AVG_Filter_s32(AvgFilterInt32Def *filter, s32 invar)
{
    filter->sum += invar - filter->delay[filter->index];
    filter->delay[filter->index++] = invar;
    
    if(filter->index >= filter->tap)
    {
        filter->index = 0;
    }

	return (filter->sum/filter->tap);
}

///**
//* @brief  曲线滑动平均滤波器(int32)
//* @param  filter: 滤波器
//* @param  invar : 新值
//* @retval 滤波结果值
//*/
//void Sliding_average_filter_s32(uint16_t data[])
//{
//    int FILTER_N=5;
//    int filter_buf[FILTER_N + 1];
//    int filter_sum = 0;
//    for(kk =Temp;kk<(PIXEL_1_FPS-Temp);kk++)
//    {    
//        filter_buf[FILTER_N] = 	data[kk]//AD转换的值赋给数组最后一个值
//        for(i = 0; i < Temp; i++) 
//        {
//            filter_buf[i] = filter_buf[i + 1]; // 所有数据左移，低位仍掉
//            filter_sum += filter_buf[i];
//        }
//    }
// 
//    filter->sum += invar - filter->delay[filter->index];
//    filter->delay[filter->index++] = invar;
//    
//    if(filter->index >= filter->tap)
//    {
//        filter->index = 0;
//    }

//}

/**
* @brief  曲线均值滤波器(int32)
* @param  
* @param  
* @retval 滤波结果值
*/
void AVG_LINE_Filter_s32(uint16_t data[])
{
    int Temp=21;
    int  ii,jj,kk,bTemp;
    static int sum;
     uint16_t a[3];//暂存固定长度数据
    
    if ((Temp & 1) > 0)
    {
        // 数组有奇数个元素，返回中间一个元素
        bTemp = (Temp-1)/2;
    }
    for(ii=0;ii<5;ii++)
    {
        data[ii]=0;
    }
    for(jj=0;jj<Temp;jj++)
    {
        sum +=data[jj];
    }
//    for(kk =bTemp;kk<(PIXEL_1_FPS-bTemp);kk=kk+3)
    for(kk =bTemp;kk<(PIXEL_1_FPS-bTemp);kk++)
    {
        if(kk-bTemp-1>=0)
        {
            sum = sum + data[kk+bTemp]- data[kk-bTemp-1];
        }

        data[kk]=sum/Temp;
    }
}

//void AVG_LINE_Filter_s32(uint16_t data[])
//{
//    int Temp=5;
//    int  ii,jj,kk,bTemp;
//    int sum=0;
//     uint16_t a[5];//暂存固定长度数据
//    if ((Temp & 1) > 0)
//    {
//        // 数组有奇数个元素，返回中间一个元素
//        bTemp = (Temp-1)/2;
//    }
//    
//    for(kk =bTemp;kk<(PIXEL_1_FPS-bTemp);kk++)
//    {
//         for(jj=0;jj<Temp;jj++)
//        {
//            sum +=data[kk-bTemp+jj];
//        }
//        data[kk]=sum/Temp;
//        sum=0;
//    }
//}
