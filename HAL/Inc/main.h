/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define VIDEO_P_Pin GPIO_PIN_0
#define VIDEO_P_GPIO_Port GPIOA
#define VIDEO_N_Pin GPIO_PIN_1
#define VIDEO_N_GPIO_Port GPIOA
#define ANA_TEST_2_Pin GPIO_PIN_2
#define ANA_TEST_2_GPIO_Port GPIOA
#define ANA_TEST_1_Pin GPIO_PIN_3
#define ANA_TEST_1_GPIO_Port GPIOA
#define P3V3_ADC_IN_Pin GPIO_PIN_4
#define P3V3_ADC_IN_GPIO_Port GPIOA
#define PA_LASER_Pin GPIO_PIN_6
#define PA_LASER_GPIO_Port GPIOA
#define LASER_CURRENT_MON_Pin GPIO_PIN_7
#define LASER_CURRENT_MON_GPIO_Port GPIOA
#define BOOT1_Pin GPIO_PIN_2
#define BOOT1_GPIO_Port GPIOB
#define SHUTTER_Pin GPIO_PIN_11
#define SHUTTER_GPIO_Port GPIOB
#define CLR_PIX_Pin GPIO_PIN_12
#define CLR_PIX_GPIO_Port GPIOB
#define DATA_RDY_Pin GPIO_PIN_13
#define DATA_RDY_GPIO_Port GPIOB
#define CLR_DATA_Pin GPIO_PIN_14
#define CLR_DATA_GPIO_Port GPIOB
#define ANGLE_Pin GPIO_PIN_15
#define ANGLE_GPIO_Port GPIOB
#define LED_Pin GPIO_PIN_8
#define LED_GPIO_Port GPIOA
#define PWR_DOWM_Pin GPIO_PIN_11
#define PWR_DOWM_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_12
#define SWDIO_GPIO_Port GPIOA
#define RD_DIR_Pin GPIO_PIN_4
#define RD_DIR_GPIO_Port GPIOB
#define ROI_SEL_Pin GPIO_PIN_5
#define ROI_SEL_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
