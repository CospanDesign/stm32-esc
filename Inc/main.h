/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
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
#include <stdio.h>
#include <stdint.h>

#include "stm32f3xx_ll_bus.h"
#include "stm32f3xx_ll_cortex.h"
#include "stm32f3xx_ll_rcc.h"
#include "stm32f3xx_ll_system.h"
#include "stm32f3xx_ll_utils.h"
#include "stm32f3xx_ll_gpio.h"
#include "stm32f3xx_ll_exti.h"
#include "stm32f3xx_ll_pwr.h"
#include "stm32f3xx_ll_dma.h"



/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define CURR_FBA_Pin GPIO_PIN_0
#define CURR_FBA_GPIO_Port GPIOA
#define CURR_FBB_Pin GPIO_PIN_1
#define CURR_FBB_GPIO_Port GPIOA
#define CURR_FBC_Pin GPIO_PIN_2
#define CURR_FBC_GPIO_Port GPIOA
#define TEMP_Pin GPIO_PIN_3
#define TEMP_GPIO_Port GPIOA
#define VSHUNTB_Pin GPIO_PIN_0
#define VSHUNTB_GPIO_Port GPIOB
#define FETC_BOT_Pin GPIO_PIN_1
#define FETC_BOT_GPIO_Port GPIOB
#define STATUS_LED_Pin GPIO_PIN_2
#define STATUS_LED_GPIO_Port GPIOB
#define VSHUNTC_Pin GPIO_PIN_11
#define VSHUNTC_GPIO_Port GPIOB
#define PHASEA_Pin GPIO_PIN_12
#define PHASEA_GPIO_Port GPIOB
#define VBUS_Pin GPIO_PIN_13
#define VBUS_GPIO_Port GPIOB
#define PHASEB_Pin GPIO_PIN_14
#define PHASEB_GPIO_Port GPIOB
#define PHASEC_Pin GPIO_PIN_15
#define PHASEC_GPIO_Port GPIOB
#define FETA_TOP_Pin GPIO_PIN_8
#define FETA_TOP_GPIO_Port GPIOA
#define FETB_TOP_Pin GPIO_PIN_9
#define FETB_TOP_GPIO_Port GPIOA
#define FETC_TOP_Pin GPIO_PIN_10
#define FETC_TOP_GPIO_Port GPIOA
#define FETA_BOT_Pin GPIO_PIN_11
#define FETA_BOT_GPIO_Port GPIOA
#define FETB_BOT_Pin GPIO_PIN_12
#define FETB_BOT_GPIO_Port GPIOA


/* USER CODE BEGIN Private defines */


/* USER CODE END Private defines */

void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)

#endif /* __MAIN_H */
