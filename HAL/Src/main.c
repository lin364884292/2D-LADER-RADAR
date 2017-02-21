/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f3xx_hal.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "usart_user.h"
#include "sysconfig.h"
#include "angle_and_speed.h"
#include "epc.h"
#include "transmit.h"  
#include "stdint.h"  
#include "algorithm.h"
#include "gpio_user.h"

uint16_t LastAngle = 0;
static uint16_t Buffer[PIXEL_1_FPS];

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
/**
  * @brief  将Lidar 2D数据按鿚讯协议打�?
  * @param  None
  * @retval None
  */
static void PackageAndSendTxData(void)
{   
	u32 out_len = 0;

	PackageDataStruct package;

    //目标速度
    LidarData.GivenSpeed = SystemConfig.GivenSpeed;
    //当前速度
    LidarData.CurrSpeed = GetAngularSpeed();
    
	package.DataID = PACK_LIDAR_DATA;
	package.DataInBuff = (u8*)&LidarData;
	package.DataInLen = sizeof(LidarDataTypeDef);
	package.DataOutBuff = ComBuffer.TxBuffer;
	package.DataOutLen = &out_len;
	
    Package(package);
    
    HAL_UART_Transmit_DMA(&huart1,ComBuffer.TxBuffer,out_len);

}

static void SendMaxPixel(void)
{
	u32 out_len = 0;
    u16 max;
	PackageDataStruct package;
    
    max = GetPixVmax().PixV;
    CCD_DataBuffer[3] = DEBUG_GET_MAX_PIXEL_VALUE;
    
	package.DataID = PACK_DEBUG_MODE;
	package.DataInBuff = (u8*)&max;
	package.DataInLen = sizeof(max);
	package.DataOutBuff = ComBuffer.TxBuffer;
	package.DataOutLen = &out_len;
	
    Package(package);
    
    HAL_UART_Transmit(&huart1,ComBuffer.TxBuffer,out_len,10);    
}

/**
  * @brief  整合Lidar�?2D数据
  * @param  angle: Lidar当前的角�?
  * @retval None
  */
static void Intergrate2DData(u16 angle)
{
    float distance = 0;
    float pix_index = 0;
    u16 confidence;
	
    //灰度质心法求得像素质心位�?
    GetCentroid(Buffer, &pix_index);

    //调整像素偏移�?(校准得来)
    pix_index += PixOffset;
		
	//温度补偿
	//pix_index += TemperatureCompensation();
	
    //估算出距�?
    GetDistance(pix_index, &distance);

    //大于�?大距离的置零
//    if ((u16)distance > MAX_DISTANCE)
//        distance = 0;

    //将距离数据存入测距数组，同时旋转角度，使零度对齐机器人正前方
    //旋转角度的原因是，光电对管和镜头方向有角度偏�?
    angle = (angle + SystemConfig.AngleOffset) % 360;
	
    confidence = GetPixVmax().PixV;
    LidarData.PointData[angle].Distance = distance; //(u16)(angle*10);
	
    //处理置信度数�?(置信度为像素峰�??,大于255的部分，�?255处理)
    
    confidence = confidence>>8; 
    if (confidence > 255)
        confidence = 255;
	
    LidarData.PointData[angle].Confidence = (u8)confidence;
}


static void ResetLidarData(void)
{
    u16 i;
    for (i = 0; i < 360; i++)
    {
		LidarData.PointData[i].Confidence=0;
		LidarData.PointData[i].Distance=0;
    }
}

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
   uint8_t id = 0x00;
   uint16_t current_angle = 0;
    
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_TIM15_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM6_Init();
  MX_TIM16_Init();

  /* USER CODE BEGIN 2 */
    UART_SetDMA();   
    InitSystemConfig();  
    
    //HAL_TIM_Base_Start_IT(&htim2);
   /* Run the ADC calibration in differential mode */      
//    HAL_ADC_Stop(&hadc1);  
  if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_DIFFERENTIAL_ENDED) != HAL_OK)
  {
    /* Start Conversation Error */
    Error_Handler();
  }

  //启动各部分引脚功�?
    HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
    InitSpeedCapture(); 
    HAL_GPIO_WritePin(PWR_DOWN_GPIO_Port,PWR_DOWN_Pin,GPIO_PIN_SET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(PWR_DOWN_GPIO_Port,PWR_DOWN_Pin,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(TEST_MODE_GPIO_Port,TEST_MODE_Pin,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RD_DIR_GPIO_Port,RD_DIR_Pin,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ROI_SEL_GPIO_Port,ROI_SEL_Pin,GPIO_PIN_RESET);  
    
    ClearPix();
    ClearData();
    StartCCDCapture();
    PackageAndSendTxData();
  //通过I2C配置epc�?
//  
//  if(HAL_I2C_Master_Transmit(&hi2c1,0x20,&id,1,1000) != HAL_OK)
//  {
//      printf("i2c t error\n");
//  }
//  HAL_Delay(100);

//  
//  if(HAL_I2C_Master_Receive(&hi2c1,0x20,&id,1,10) != HAL_OK)
//  {
//      printf("i2c r error\n");
//  }
//  else
//      printf("id = %d\n",id);    

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      LedToggle();
      if(1)
    {
        memcpy(Buffer,CCD_DataBuffer
        
        
        
        
        ,sizeof(CCD_DataBuffer));    
        DataReady = 0;
        if(IsZeroPoint() == TRUE)
        {
           PackageAndSendTxData(); 
            ResetLidarData();
        }
        current_angle = GetLidarAngle() % 360;

        //整合Lidar�?2D数据
        //低转速时可能会出现同�?个角度多次处理，过滤掉这�?操作，提高测距效�?
        if ((current_angle != LastAngle))
        {
            Intergrate2DData(current_angle);
            //记录当前位置
            LastAngle = current_angle;
        }
    }
    if (IsAllPulseWidthDataReady() == TRUE)
    {

        SetPulseWidthThreshold();
    }   
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
   
    HandleCmd();
  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_TIM1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
