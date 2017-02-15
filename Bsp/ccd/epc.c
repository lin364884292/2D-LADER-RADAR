/**
  * @file        epc.c
  * @author      林佳伟
  * @version     V01
  * @date        2017.01.11
  * @brief       epc芯片驱动
  * @note
  *
  * @attention   COYPRIGHT INMOTION ROBOT
  */
#include "epc.h"
#include "adc.h"
#include "sysconfig.h"
#include "i2c.h"
#include "tim.h"
#include "adc.h"


uint8_t DataReady = 0;

void CCDInit(void)
{
    /*
    RD_DIR = GND --> differential mode
    ROI_SEL = GND --> read pixel 0..1023
    Gain = GND --> Gain = 2
    HOR_BIN = High-z --> 1pixel(disable horizontal binning)
    VIDEO_CM  = High-z
    [BW0:BW1] = ? --> video bandwidth
    */

    
    // power-down = L

    
}

void AcquisitionData(void)
{
    
}

TempSensorTypeDef ReadTempSensor(void)
{
    TempSensorTypeDef tmp;
    
    return tmp;
}


uint8_t ReadChipRevNo(void)
{
    return 0;
}


void StartCCDCapture(void)
{
    HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_4); // start SHUTTER
    //HAL_TIM_Base_Start_IT(&htim16); // start clr_data
    HAL_ADC_Start_DMA(&hadc1,(uint32_t*)CCD_DataBuffer,sizeof(CCD_DataBuffer)/sizeof(CCD_DataBuffer[0])); //start adc dma
    //HAL_TIM_IC_Start(&htim4,TIM_CHANNEL_3); // start read counter
    //HAL_TIM_Base_Start_IT(&htim4);
}

void StartRead(void)
{
    HAL_ADC_Start_DMA(&hadc1,(uint32_t*)CCD_DataBuffer,PIXEL_1_FPS); //start adc dma
     //HAL_DMA_Start_IT(&hdma_adc1, (uint32_t)hadc1.Instance->DR, *(uint32_t*)CCD_DataBuffer, PIXEL_1_FPS);
    
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1); //READ
    //__HAL_TIM_MOE_ENABLE(&htim1);
}

void ClearData(void)
{
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_RESET);   
}


void ClearPix(void)
{
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET);

    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET);   
}


/*  interrupt callback*/

//void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
//{
//    if(htim == &htim4)
//    {
//      HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_1);  
//    }
//}


////产生CLR_DATA时序
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{
//    if(htim == &htim16)
//    {
//       if(htim2.Instance->CNT == 30)
//        {
//            HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_SET);
//        }
//        else
//        {
//            HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_RESET);
//        }
//    }
//    
//    if(htim == &htim4)
//    {
//      HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_1);  
//    }

//}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if(hadc == &hadc1)
    {        
        HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_1);
        //HAL_DMA_Abort(&hdma_adc1);
       // HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_SET);
        ClearData();
        DataReady = 1;
    }
}


void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim == &htim2)
    {
//       HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_RESET);
//       StartRead(); 
    }
}

//Data Ready
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == GPIO_PIN_13)
    {
       // HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_RESET);
        StartRead();
    }
}