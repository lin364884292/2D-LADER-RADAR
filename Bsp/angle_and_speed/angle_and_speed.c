/**
  * @file        angle_and_speed.c
  * @author      林佳伟
  * @version     V01
  * @date        2017.1.30
  * @brief       测电机转速及计算当前角度
  * @note
  */
#include "angle_and_speed.h"
#include "transmit.h"
#include "tim.h"
#include "sysconfig.h"

static BOOL LidarStall = TRUE;  //Lidar停转标志
static u8 CaptureState = WAIT_START;	//上一次输入捕获触发沿		    				
static u32 PulseWidth;	//输入捕获的值(TIM3是16位)
static u16 Period = 0;//pwm周期，上升沿开始计算
static u8 GratingIndex = 0x0;//光栅的编号
static u32 AngularSpeed = 0;//角速度
static u32 TimePerCircle = 0;//转一圈的总时间
static ZeroFlagType ZeroFlag = NON_ZERO_POINT;
static u16 GratingPulseWidth[GRATING_AMOUNT]; // 记录每个光栅的脉宽
static BOOL PulseWidthDataReady = FALSE;
static u16 LevelThreshold = 0;  //定义光栅电平持续时间阈值。低于阈值则为最短的光栅，单位 us

//extern s16 RobotAngluarSpeed;

//记录陀螺仪角速度
s16 RobotAngluarSpeedRecord[GRATING_AMOUNT];

//记录Lidar旋转时，经过一个光栅下降沿的时间
u32 ElapsedTimeRecord[GRATING_AMOUNT];


void InitSpeedCapture(void)
{
    HAL_TIM_IC_Start_IT(&htim15,TIM_CHANNEL_2);
    HAL_TIM_Base_Start(&htim6);   
}


/**
  * @brief  定时器3中断服务程序
  * @param  None
  * @retval None
  */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) 
{
    //发生捕获事件
    if(htim == &htim15)
    {
        if (CaptureState == RISING_EDGE) //上一次捕获的状态是上升沿,这一次进入中断为下降沿.捕获到一个下降沿，一次捕获结束 		
        {
            CaptureState = FALLING_EDGE;  //标记捕获到了下降沿
            PulseWidth = htim->Instance->CNT; //获取捕获的脉宽
            //printf("%d,%d\n",PulseWidth,LevelThreshold);
            if (PulseWidth < LevelThreshold) //判断是否找到原点
            {
                //计算角速度.
                //单位为(度/S)
                TimePerCircle = htim6.Instance->CNT;
                
                if (TimePerCircle != 0)
                    AngularSpeed = 360 * 10000 / TimePerCircle;
                else
                    AngularSpeed = 0;
                htim6.Instance->CNT = 0;
                
                
                
                //转速突然变快，会导致PulseWidth一直小于LevelThreshold，从而产生一个非常大的AngularSpeed
                //在这里重置LevelThreshold，并且不发出这一帧数据
                if (AngularSpeed > MAX_SPEED)
                {
                    LevelThreshold = 0;
                }
				else
				{
					if(GratingIndex == GRATING_AMOUNT-1)
					{
						ZeroFlag = ZERO_POINT;
						LidarStall = FALSE;
					}
				}
				
				GratingIndex = 0;
            }
            else //光栅数加1
            {
                GratingIndex++;
            }

			//防止非正常旋转造成的数组下标越界 
			if (GratingIndex  < GRATING_AMOUNT)
			{
				if (PulseWidthDataReady == FALSE)
				{
					//将脉宽存入数组
					GratingPulseWidth[GratingIndex] = PulseWidth;
				}			
				//记录数据
				//RobotAngluarSpeedRecord[GratingIndex] = RobotAngluarSpeed;
				ElapsedTimeRecord[GratingIndex] = htim6.Instance->CNT;	
			}
            if (GratingIndex >= (GRATING_AMOUNT - 1))
            {
                 PulseWidthDataReady = TRUE;
            }
			
            //TIM_OC4PolarityConfig(TIM3, TIM_ICPolarity_Rising); //CC1P=0 设置为上升沿捕获
            __HAL_TIM_SET_CAPTUREPOLARITY(htim,TIM_CHANNEL_2,TIM_INPUTCHANNELPOLARITY_RISING);
            
        }
        else  	//捕获上升沿
        {
            if (CaptureState == WAIT_START)
                Period = 0;
            else
                Period = htim->Instance->CNT;

            CaptureState = RISING_EDGE; //标记捕获到了上升沿

            //TIM_OC4PolarityConfig(TIM3, TIM_ICPolarity_Falling);  //CC1P=1 设置为下降沿捕获
            __HAL_TIM_SET_CAPTUREPOLARITY(htim,TIM_CHANNEL_2,TIM_INPUTCHANNELPOLARITY_FALLING);
            
            htim->Instance->CNT = 0;
        }

		//TIM_SetCounter(TIM4, 0);
    }
}

/**
  * @brief  判断脉宽数据是否全部获取完成
  * @param  None
  * @retval 返回FALSE或者TRUE
  */
BOOL IsAllPulseWidthDataReady(void)
{
    if (PulseWidthDataReady == TRUE)
    {
        PulseWidthDataReady = FALSE;
        return TRUE;
    }

    return FALSE;
}

/**
  * @brief  计算并设置光栅的脉宽阈值，用来区分最短光栅
  * @note   在不同转速情况下，光栅的脉宽都会变化。所以lidar每旋转一周
  进行一次计算，确定最佳的一个阈值，适用不同的转速。
  * @param  None
  * @retval None
  */
void SetPulseWidthThreshold(void)
{
    u8 i = 0;
    u16 min_pulse_width = 0xffff;
    u32 ave_pulse_width = 0;
    for (i = 0; i < GRATING_AMOUNT; i++)
    {
        if (GratingPulseWidth[i] < min_pulse_width)
            min_pulse_width = GratingPulseWidth[i];
        ave_pulse_width += GratingPulseWidth[i];
    }
    ave_pulse_width /= GRATING_AMOUNT;
    LevelThreshold = (ave_pulse_width + min_pulse_width) / 2;
    
          
}

/**
  * @brief  获取角速度
  * @param  None
  * @retval 角速度值(度/S)
  */
u32 GetAngularSpeed(void)
{
    return AngularSpeed;
}

/**
  * @brief  重置角速度
  * @note   这个函数是在lidar停转之后需要
  * @param  None
  * @retval None
  */
void ResetAngularSpeed(void)
{
    AngularSpeed = 0;
}

/**
  * @brief  判断Lidar是否转回零点
  * @note   这里将零点设置为最短光栅的下降沿
  * @param  None
  * @retval 返回FALSE或者TRUE
  */
BOOL IsZeroPoint(void)
{
    if (ZeroFlag == ZERO_POINT)
    {
        ZeroFlag = NON_ZERO_POINT;
        return TRUE;
    }

    return FALSE;
}

/**
  * @brief  计算Lidar当前角度
  * @note   计算方法为：总光栅数 * 经过光栅的个数 + Timer计数器当前计数值 *角速度
  * @param  None
  * @retval Lidar当前角度
  */
u16 GetLidarAngle(void)
{
    u16 position = 0;
    u32 tim = htim15.Instance->CNT;;

	//计算位置是从下降沿开始的
	if (CaptureState == RISING_EDGE)
	{
		//tim在上升沿被清零
		if(GratingIndex !=0)
			position = (ANGLE_PER_GRATING * GratingIndex + (u32)(tim + (Period - PulseWidth))*AngularSpeed / 1000000) + (ANGLE_PER_GRATING/4);
		else
			position = (ANGLE_PER_GRATING * GratingIndex + (u32)(tim + (Period - PulseWidth))*AngularSpeed / 1000000);		
		
	}
	else
	{
		//小光栅比正常的光栅小(ANGLE_PER_GRATING/4)度，若采用小光栅的下降沿作为0度，其他光栅的下降沿将对应(ANGLE_PER_GRATING/4)度
		if (GratingIndex != 0)
			position = (ANGLE_PER_GRATING * GratingIndex + (tim - PulseWidth)*AngularSpeed / 1000000) + (ANGLE_PER_GRATING/4);
		else
			position = (ANGLE_PER_GRATING * GratingIndex + (tim - PulseWidth)*AngularSpeed / 1000000);

	}
	
    return position;
}

/**
  * @brief  判断lidar是否停转
  * @note   TIM2会在Lidar回到零点时清零计数器，若是Lidar停转，TIM2将无法清零，也就会发生更新事件
  所以通过检测TIM2更新标志位来判断是否停转
  * @param  None
  * @retval 停转(TRUE)或者正常运行(FALSE)
  */
BOOL IsLidarStall(void)
{
//    if (TIM_GetFlagStatus(TIM4, TIM_FLAG_Update) == SET)
//    {
//		TIM_ClearFlag(TIM4,TIM_FLAG_Update);
//        LidarStall = TRUE;
//    }

    return LidarStall;

}

/**
 * @brief  判断lidar停转时发送速度的间隔
 * @note   添加该判断条件是为了减少在停转期间发给STM8的数据
 * @param  None
 * @retval TRUE发送 OR FALSE不发送
 */
BOOL IsTimeToSendSpeed(void)
{
//    if (TIM_GetFlagStatus(TIM2, TIM_FLAG_Update) == SET)
//    {
//        TIM_ClearFlag(TIM2, TIM_FLAG_Update);
//        return TRUE;
//    }

    return FALSE;
}

/**
 * @brief  发送速度给STM8
 * @note   该函数在没有雷达数据发送时启用
 * @param  speed : 速度值
 * @retval TRUE发送 OR FALSE不发送
 */
 void SendSpeed(u16 speed)
{
    u8 i = 0;
    PackageDataStruct package;
    u32 out_len;
	struct
	{
		u16 temperature;
		u16 CurrSpeed;
		u16 GivenSpeed;
	}header_data;
	
	header_data.temperature = 0;
	header_data.CurrSpeed = AngularSpeed;
	header_data.GivenSpeed = speed;
	
    package.DataID = PACK_SET_SPEED;
    package.DataInBuff = (u8 *)&header_data;
    package.DataInLen = sizeof(header_data);
    package.DataOutBuff = ComBuffer.TxBuffer;
    package.DataOutLen = &out_len;
    Package(package);
    
    
//    for (i = 0; i < out_len; i++)
//    {
//        while ((USART1->SR & 0X40) == 0)
//            ;
//        USART1->DR = (u8)ComBuffer.TxBuffer[i];
//    }
}

/**
  * @brief  获取当前光栅的计数值
  * @param  None
  * @retval None
  */
u8 GetCurrentGratingIndex(void)
{
	if(GratingIndex>GRATING_AMOUNT-1)
	{
		GratingIndex = 0;
	}
	
	return GratingIndex;
}

/************************ (C) COPYRIGHT INMOTION ROBOT *****END OF FILE****/
