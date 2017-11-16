/**
 ******************************************************************************
 * @file    stm32F401_nucleo_ihm11m1.c
 * @author  IPC
 * @version V1.0.0
 * @date    10/07/2016
 * @brief   This file provides the interface between the MC-lib and STM ESC F401xx
 ******************************************************************************
 *
 * COPYRIGHT(c) 2015 STMicroelectronics
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

#include "main.h"
#include "adc.h"
#include "mc_adapter.h"
#include "6Step_Lib.h"
#include "stm32f3xx_hal.h"
#include "stm32f3xx_hal_gpio.h"

uint8_t led_status_test = 0;

struct ADC_ELEMENT_T{
  ADC_HandleTypeDef* adc;
  uint8_t index;
  uint32_t sample_time;
};

static struct ADC_ELEMENT_T ADC_SEL[] = {
  {&hadc1, ADC_CHANNEL_1,  ADC_SAMPLETIME_2CYCLES_5},    //Current FB A
  {&hadc1, ADC_CHANNEL_2,  ADC_SAMPLETIME_2CYCLES_5},    //Current FB B
  {&hadc1, ADC_CHANNEL_3,  ADC_SAMPLETIME_2CYCLES_5},    //Current FB C
  {&hadc1, ADC_CHANNEL_4,  ADC_SAMPLETIME_61CYCLES_5},   //Temperature
  {&hadc3, ADC_CHANNEL_5,  ADC_SAMPLETIME_61CYCLES_5},   //VBUS
  {&hadc3, ADC_CHANNEL_12, ADC_SAMPLETIME_61CYCLES_5},   //VSHUTNB
  {&hadc4, ADC_CHANNEL_3,  ADC_SAMPLETIME_19CYCLES_5},   //Bemf A
  {&hadc4, ADC_CHANNEL_4,  ADC_SAMPLETIME_19CYCLES_5},   //Bemf B
  {&hadc4, ADC_CHANNEL_5,  ADC_SAMPLETIME_19CYCLES_5}    //Bemf C
};


extern SIXSTEP_Base_InitTypeDef SIXSTEP_parameters; /*!< Main SixStep structure*/
extern SIXSTEP_PI_PARAM_InitTypeDef_t PI_parameters; /*!< SixStep PI regulator structure*/
//  extern MotorDriver_TypeDef STSPIN230MotorDriver;

extern void MC_ADCx_SixStep_Bemf(ADC_HandleTypeDef *hadc);
extern void MC_TIMx_SixStep_timebase(void);
extern void MC_SysTick_SixStep_MediumFrequencyTask(void);



/**
 * @brief  Select the new ADC Channel
 * @param  adc_ch
 * @retval None
 */
void MC_SixStep_ADC_Channel(uint32_t adc_ch)
{
  ADC_ChannelConfTypeDef adc_channel;

  adc_channel.Channel = ADC_SEL[adc_ch].index;
  adc_channel.Rank = 1;
  adc_channel.SingleDiff = ADC_SINGLE_ENDED;
  adc_channel.SamplingTime = ADC_SEL[adc_ch].sample_time;
  adc_channel.OffsetNumber = ADC_OFFSET_NONE;
  adc_channel.Offset = 0;

  //    __HAL_ADC_DISABLE(&ADCx);
  /* Clear the old SQx bits for the selected rank */
  //    ADCx.Instance->SQR3 &= ~ADC_SQR3_RK(ADC_SQR3_SQ1, 1);
  /* Set the SQx bits for the selected rank */
  //    ADCx.Instance->SQR3 |= ADC_SQR3_RK(adc_ch, 1);
  //    __HAL_ADC_ENABLE(&ADCx);

  __HAL_ADC_DISABLE(ADC_SEL[adc_ch].adc);
  //    ADC_SEL[adc_ch].adc->Instance->SQR3 &= ~ADC_SQR3_RK(ADC_SQR3_SQ1, 1);
  //    ADC_SEL[adc_ch].adc->Instance->SQR3 |= ADC_SQR3_RK(adc_ch, 1);
  HAL_ADC_ConfigChannel(ADC_SEL[adc_ch].adc, &adc_channel);
  __HAL_ADC_ENABLE(ADC_SEL[adc_ch].adc);
}
/**
 * @}
 */

void MC_SixStep_ESC_Init()
{
  TIM_ClearInputConfigTypeDef sClearInputConfig;
  //    ADC_ChannelConfTypeDef sConfig;

  /******************** ETR CONFIGURATION **************************P*********/
  sClearInputConfig.ClearInputState = 1;
  sClearInputConfig.ClearInputSource = TIM_CLEARINPUTSOURCE_ETR;
  sClearInputConfig.ClearInputPolarity = TIM_CLEARINPUTPOLARITY_NONINVERTED;
  sClearInputConfig.ClearInputPrescaler = TIM_CLEARINPUTPRESCALER_DIV1;
  sClearInputConfig.ClearInputFilter = 0;
  HAL_TIM_ConfigOCrefClear(&HF_TIMx, &sClearInputConfig, HF_TIMx_CH1);
  HAL_TIM_ConfigOCrefClear(&HF_TIMx, &sClearInputConfig, HF_TIMx_CH2);
  HAL_TIM_ConfigOCrefClear(&HF_TIMx, &sClearInputConfig, HF_TIMx_CH3);
  /***************************************************************************/

  __HAL_FREEZE_TIM1_DBGMCU();  /* Stop TIM during Breakpoint */

  __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_BREAK); /* Enable the TIM Break interrupt */

  /******************** REGULAR CHANNELS CONFIGURATION *************************/
  //    sConfig.Channel = ADC_CH_1; /* Current feedabck */
  //    sConfig.Rank = 1;
  //    sConfig.SamplingTime = ADC_CH_1_ST;
  //    sConfig.Offset = 0;
  //    HAL_ADC_ConfigChannel(&hadc1, &sConfig);
  //    sConfig.Channel = ADC_CH_3; /* Bus voltage */
  //    sConfig.SamplingTime = ADC_CH_3_ST;
  //    HAL_ADC_ConfigChannel(&hadc1, &sConfig);
  //    sConfig.Channel = ADC_CH_4; /* Temperature feedback */
  //    sConfig.SamplingTime = ADC_CH_4_ST;
  //    HAL_ADC_ConfigChannel(&hadc1, &sConfig);
  //    sConfig.Channel = ADC_Bemf_CH1; /* BEMF feedback phase A */
  //    sConfig.SamplingTime = ADC_Bemf_CH1_ST;
  //    HAL_ADC_ConfigChannel(&hadc1, &sConfig);
  //    sConfig.Channel = ADC_Bemf_CH2; /* BEMF feedback phase B */
  //    sConfig.SamplingTime = ADC_Bemf_CH2_ST;
  //    HAL_ADC_ConfigChannel(&hadc1, &sConfig);
  //    sConfig.Channel = ADC_Bemf_CH3; /* BEMF feedback phase C */
  //    sConfig.SamplingTime = ADC_Bemf_CH3_ST;
  //    HAL_ADC_ConfigChannel(&hadc1, &sConfig);
  //    sConfig.Channel = ADC_CH_2; /* Potentiometer */
  //    sConfig.SamplingTime = ADC_CH_2_ST;
  //    HAL_ADC_ConfigChannel(&hadc1, &sConfig);
  /***************************************************************************/
  HAL_ADC_Start_IT(&hadc1);
  HAL_ADC_Start_IT(&hadc3);
  HAL_ADC_Start_IT(&hadc4);
}
/**
 * @}
 */

/** @defgroup START_DAC    START_DAC
 *  @{
 @brief Start DAC for debug
 */
/**
 * @brief  Start DAC for debug
 * @retval None
 */
void START_DAC()
{

}
/**
 * @}
 */
/** @defgroup STOP_DAC    STOP_DAC
 *  @{
 @brief Stop DAC for debug
 */
/**
 * @brief  Stop DAC for debug
 * @retval None
 */
void STOP_DAC()
{

}
/**
 * @}
 */
/** @defgroup SET_DAC_value    SET_DAC_value
 *  @{
 @brief Set DAC value for debug
 */
/**
 * @brief  Set DAC value for debug
 * @param  dac_value: information to plot through DAC
 * @retval None
 */
void SET_DAC_value(uint16_t dac_value)
{

}
/**
 * @}
 */


/** @defgroup HAL_ADC_ConvCpltCallback    HAL_ADC_ConvCpltCallback
 *  @{
 * @brief ADC callback
 */
/**
 * @brief  ADC callback
 * @param  hadc
 * @retval None
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  MC_ADCx_SixStep_Bemf(hadc);
}
/**
 * @}
 */

/** @defgroup HAL_TIM_PeriodElapsedCallback    HAL_TIM_PeriodElapsedCallback
 *  @{
 * @brief htim callback
 */
/**
 * @brief  htim callback
 * @param  htim
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  MC_TIMx_SixStep_timebase();
}
/**
 * @}
 */

/** @defgroup HAL_SYSTICK_Callback    HAL_SYSTICK_Callback
 *  @{
 * @brief Systick callback
 */
/**
 * @brief  Systick callback
 * @retval None
 */

void HAL_SYSTICK_Callback()
{
  MC_SysTick_SixStep_MediumFrequencyTask();
}
/**
 * @}
 */

/** @defgroup HAL_GPIO_EXTI_Callback    HAL_GPIO_EXTI_Callback
 *  @{
 * @brief EXT callback
 */
/**
 * @brief  EXT callback
 * @param  GPIO_Pin
 * @retval None
 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  MC_EXT_button_SixStep();
}
/**
 * @}
 */

/** @defgroup EnableInput_CH1_E_CH2_E_CH3_D    EnableInput_CH1_E_CH2_E_CH3_D
 *  @{
 * @brief Enable Input channel for STSPIN230
 */
/**
 * @brief  Enable Input channel CH1 and CH2 for STSPIN230
 * @retval None
 */

void MC_SixStep_EnableInput_CH1_E_CH2_E_CH3_D()
{
  HAL_TIM_PWM_Start(&HF_TIMx,HF_TIMx_CH1);           //TIM1_CH1 ENABLE
  HAL_TIMEx_PWMN_Start(&HF_TIMx,HF_TIMx_CH1) ;

  HAL_TIM_PWM_Start(&HF_TIMx,HF_TIMx_CH2);           //TIM1_CH2 ENABLE  
  HAL_TIMEx_PWMN_Start(&HF_TIMx,HF_TIMx_CH2) ;

  HAL_TIM_PWM_Stop(&HF_TIMx,HF_TIMx_CH3);           //TIM1_CH3 DISABLE  
  HAL_TIMEx_PWMN_Stop(&HF_TIMx,HF_TIMx_CH3) ;  
}

/**
 * @}
 */

/** @defgroup EnableInput_CH1_E_CH2_D_CH3_E    EnableInput_CH1_E_CH2_D_CH3_E
 *  @{
 * @brief Enable Input channel for STSPIN230
 */
/**
 * @brief  Enable Input channel CH1 and CH3 for STSPIN230
 * @retval None
 */

void  MC_SixStep_EnableInput_CH1_E_CH2_D_CH3_E()
{
  HAL_TIM_PWM_Start(&HF_TIMx,HF_TIMx_CH1);           //TIM1_CH1 ENABLE 
  HAL_TIMEx_PWMN_Start(&HF_TIMx,HF_TIMx_CH1) ;

  HAL_TIM_PWM_Stop(&HF_TIMx,HF_TIMx_CH2);           //TIM1_CH2  DISABLE 
  HAL_TIMEx_PWMN_Stop(&HF_TIMx,HF_TIMx_CH2) ;

  HAL_TIM_PWM_Start(&HF_TIMx,HF_TIMx_CH3);           //TIM1_CH3 ENABLE  
  HAL_TIMEx_PWMN_Start(&HF_TIMx,HF_TIMx_CH3) ;    
}

/**
 * @}
 */

/** @defgroup EnableInput_CH1_D_CH2_E_CH3_E    EnableInput_CH1_D_CH2_E_CH3_E
 *  @{
 * @brief Enable Input channel for STSPIN230
 */
/**
 * @brief  Enable Input channel CH2 and CH3 for STSPIN230
 * @retval None
 */

void MC_SixStep_EnableInput_CH1_D_CH2_E_CH3_E()
{
  HAL_TIM_PWM_Stop(&HF_TIMx,HF_TIMx_CH1);           //TIM1_CH1 DISABLE 
  HAL_TIMEx_PWMN_Stop(&HF_TIMx,HF_TIMx_CH1) ;

  HAL_TIM_PWM_Start(&HF_TIMx,HF_TIMx_CH2);           //TIM1_CH2 ENABLE  
  HAL_TIMEx_PWMN_Start(&HF_TIMx,HF_TIMx_CH2) ;

  HAL_TIM_PWM_Start(&HF_TIMx,HF_TIMx_CH3);           //TIM1_CH3 ENABLE  
  HAL_TIMEx_PWMN_Start(&HF_TIMx,HF_TIMx_CH3) ;  
}

/**
 * @}
 */

/** @defgroup DisableInput_CH1_D_CH2_D_CH3_D    DisableInput_CH1_D_CH2_D_CH3_D
 *  @{
 * @brief Disable All Input channels for STSPIN230
 */
/**
 * @brief  Enable Input channel CH2 and CH3 for STSPIN230
 * @retval None
 */

void MC_SixStep_DisableInput_CH1_D_CH2_D_CH3_D()
{
  HAL_TIM_PWM_Stop(&HF_TIMx,HF_TIMx_CH1);           //TIM1_CH1 DISABLE 
  HAL_TIMEx_PWMN_Stop(&HF_TIMx,HF_TIMx_CH1) ;

  HAL_TIM_PWM_Stop(&HF_TIMx,HF_TIMx_CH2);           //TIM1_CH2  DISABLE 
  HAL_TIMEx_PWMN_Stop(&HF_TIMx,HF_TIMx_CH2) ;

  HAL_TIM_PWM_Stop(&HF_TIMx,HF_TIMx_CH3);           //TIM1_CH3 DISABLE  
  HAL_TIMEx_PWMN_Stop(&HF_TIMx,HF_TIMx_CH3) ;  
}

/**
 * @}
 */

/** @defgroup Start_PWM_driving    Start_PWM_driving
 *  @{
 * @brief Enable the PWM generation on Input channels
 */
/**
 * @brief  Enable PWM channels for STSPIN230
 * @retval None
 */

void MC_SixStep_Start_PWM_driving()
{
  HAL_TIM_PWM_Start(&HF_TIMx, HF_TIMx_CH1);           //TIM1_CH1 ENABLE   
  HAL_TIMEx_PWMN_Start(&HF_TIMx,HF_TIMx_CH1) ;  

  HAL_TIM_PWM_Start(&HF_TIMx, HF_TIMx_CH2);           //TIM1_CH2 ENABLE   
  HAL_TIMEx_PWMN_Start(&HF_TIMx,HF_TIMx_CH2) ;  

  HAL_TIM_PWM_Start(&HF_TIMx, HF_TIMx_CH3);           //TIM1_CH3 ENABLE  
  HAL_TIMEx_PWMN_Start(&HF_TIMx,HF_TIMx_CH3) ;  

}

/**
 * @}
 */

/** @defgroup Stop_PWM_driving    Stop_PWM_driving
 *  @{
 * @brief Disable the PWM generation on Input channels
 */
/**
 * @brief  Disable PWM channels for STSPIN230
 * @retval None
 */

void MC_SixStep_Stop_PWM_driving()
{
  HAL_TIM_PWM_Stop(&HF_TIMx, HF_TIMx_CH1);           //TIM1_CH1 DISABLE   
  HAL_TIMEx_PWMN_Stop(&HF_TIMx,HF_TIMx_CH1) ;  

  HAL_TIM_PWM_Stop(&HF_TIMx, HF_TIMx_CH2);           //TIM1_CH2 DISABLE   
  HAL_TIMEx_PWMN_Stop(&HF_TIMx,HF_TIMx_CH2) ;  

  HAL_TIM_PWM_Stop(&HF_TIMx, HF_TIMx_CH3);           //TIM1_CH3 DISABLE  
  HAL_TIMEx_PWMN_Stop(&HF_TIMx,HF_TIMx_CH3) ;  
}

/**
 * @}
 */

/** @defgroup HF_TIMx_SetDutyCycle_CH1    HF_TIMx_SetDutyCycle_CH1
 *  @{
 * @brief Set the Duty Cycle value for CH1
 */
/**
 * @brief  Set the Duty Cycle value for CH1
 * @retval None
 */

void MC_SixStep_HF_TIMx_SetDutyCycle_CH1(uint16_t CCR_value)
{
  HF_TIMx.Instance->HF_TIMx_CCR1 = CCR_value;  
}


/**
 * @}
 */

/** @defgroup HF_TIMx_SetDutyCycle_CH2    HF_TIMx_SetDutyCycle_CH2
 *  @{
 * @brief Set the Duty Cycle value for CH2
 */
/**
 * @brief  Set the Duty Cycle value for CH2
 * @retval None
 */

void MC_SixStep_HF_TIMx_SetDutyCycle_CH2(uint16_t CCR_value)
{
  HF_TIMx.Instance->HF_TIMx_CCR2 = CCR_value;  
}
/**
 * @}
 */

/** @defgroup HF_TIMx_SetDutyCycle_CH3    HF_TIMx_SetDutyCycle_CH3
 *  @{
 * @brief Set the Duty Cycle value for CH3
 */
/**
 * @brief  Set the Duty Cycle value for CH3
 * @retval None
 */
void MC_SixStep_HF_TIMx_SetDutyCycle_CH3(uint16_t CCR_value)
{
  HF_TIMx.Instance->HF_TIMx_CCR3 = CCR_value;  
}

/**
 * @}
 */

/** @defgroup Current_Reference_Start    Current_Reference_Start
 *  @{
 * @brief Enable the Current Reference generation
 */
/**
 * @brief  Enable the Current Reference generation
 * @retval None
 */

void MC_SixStep_Current_Reference_Start()
{
  MC_SixStep_Start_PWM_driving();
  SIXSTEP_parameters.pulse_value=STARTUP_DUTY_CYCLE;

}

/**
 * @}
 */


/** @defgroup Current_Reference_Stop    Current_Reference_Stop
 *  @{
 * @brief Disable the Current Reference generation
 */
/**
 * @brief  Disable the Current Reference generation
 * @retval None
 */

void MC_SixStep_Current_Reference_Stop()
{
  MC_SixStep_Stop_PWM_driving();
  SIXSTEP_parameters.pulse_value=STARTUP_DUTY_CYCLE;
}

/**
 * @}
 */


/** @defgroup Current_Reference_Setvalue    Current_Reference_Setvalue
 *  @{
 * @brief Set the value for Current Reference
 */
/**
 * @brief  Set the value for Current Reference
 * @retval None
 */
void MC_SixStep_Current_Reference_Setvalue(uint16_t Iref)
{
  SIXSTEP_parameters.pulse_value=Iref;
}

/**
 * @}
 */


/** @defgroup Bemf_delay_calc    Bemf_delay_calc
 *  @{
 * @brief Bemf delay calculation
 */
/**
 * @brief  Bemf delay calculation
 * @retval None
 */
void Bemf_delay_calc()
{
  if(PI_parameters.Reference>=0)
  {
    if(SIXSTEP_parameters.speed_fdbk_filtered<=12000 && SIXSTEP_parameters.speed_fdbk_filtered>10000)
    {
      SIXSTEP_parameters.demagn_value = DEMAGN_VAL_2;
    }
    else if(SIXSTEP_parameters.speed_fdbk_filtered<=10000 && SIXSTEP_parameters.speed_fdbk_filtered>9400)
    {
      SIXSTEP_parameters.demagn_value = DEMAGN_VAL_3;
    }
    else if(SIXSTEP_parameters.speed_fdbk_filtered<=9400 && SIXSTEP_parameters.speed_fdbk_filtered>7600)
    {
      SIXSTEP_parameters.demagn_value = DEMAGN_VAL_4;
    }
    else if(SIXSTEP_parameters.speed_fdbk_filtered<=7600 && SIXSTEP_parameters.speed_fdbk_filtered>6000)
    {
      SIXSTEP_parameters.demagn_value = DEMAGN_VAL_5;
    }
    else if(SIXSTEP_parameters.speed_fdbk_filtered<=6000 && SIXSTEP_parameters.speed_fdbk_filtered>5400)
    {
      SIXSTEP_parameters.demagn_value = DEMAGN_VAL_6;
    }
    else if(SIXSTEP_parameters.speed_fdbk_filtered<=5400 && SIXSTEP_parameters.speed_fdbk_filtered>4750)
    {
      SIXSTEP_parameters.demagn_value = DEMAGN_VAL_7;
    }
    else if(SIXSTEP_parameters.speed_fdbk_filtered<=4750 && SIXSTEP_parameters.speed_fdbk_filtered>4200)
    {
      SIXSTEP_parameters.demagn_value = DEMAGN_VAL_8;
    }
    else if(SIXSTEP_parameters.speed_fdbk_filtered<=4200 && SIXSTEP_parameters.speed_fdbk_filtered>2600)
    {
      SIXSTEP_parameters.demagn_value = DEMAGN_VAL_9;
    }
    else if(SIXSTEP_parameters.speed_fdbk_filtered<=2600 && SIXSTEP_parameters.speed_fdbk_filtered>1800)
    {
      SIXSTEP_parameters.demagn_value = DEMAGN_VAL_10;
    }
    else if(SIXSTEP_parameters.speed_fdbk_filtered<=1800 && SIXSTEP_parameters.speed_fdbk_filtered>1500)
    {
      SIXSTEP_parameters.demagn_value = DEMAGN_VAL_11;
    }
    else if(SIXSTEP_parameters.speed_fdbk_filtered<=1500 && SIXSTEP_parameters.speed_fdbk_filtered>1300)
    {
      SIXSTEP_parameters.demagn_value = DEMAGN_VAL_12;
    }
    else if(SIXSTEP_parameters.speed_fdbk_filtered<=1300 && SIXSTEP_parameters.speed_fdbk_filtered>1000)
    {
      SIXSTEP_parameters.demagn_value = DEMAGN_VAL_13;
    }
    else if(SIXSTEP_parameters.speed_fdbk_filtered<=1000 && SIXSTEP_parameters.speed_fdbk_filtered>500)
    {
      SIXSTEP_parameters.demagn_value = DEMAGN_VAL_14;
    }
  }
  else
  {
    if(SIXSTEP_parameters.speed_fdbk_filtered>=-12000 && SIXSTEP_parameters.speed_fdbk_filtered<-10000)
    {
      SIXSTEP_parameters.demagn_value = DEMAGN_VAL_1;
    }
    else if(SIXSTEP_parameters.speed_fdbk_filtered>=-10000 && SIXSTEP_parameters.speed_fdbk_filtered<-7800)
    {
      SIXSTEP_parameters.demagn_value = DEMAGN_VAL_2;
    }
    else if(SIXSTEP_parameters.speed_fdbk_filtered>=-7800 && SIXSTEP_parameters.speed_fdbk_filtered<-6400)
    {
      SIXSTEP_parameters.demagn_value = DEMAGN_VAL_3;
    }
    else if(SIXSTEP_parameters.speed_fdbk_filtered>=-6400 && SIXSTEP_parameters.speed_fdbk_filtered<-5400)
    {
      SIXSTEP_parameters.demagn_value = DEMAGN_VAL_4;
    }
    else if(SIXSTEP_parameters.speed_fdbk_filtered>=-5400 && SIXSTEP_parameters.speed_fdbk_filtered<-4650)
    {
      SIXSTEP_parameters.demagn_value = DEMAGN_VAL_5;
    }
    else if(SIXSTEP_parameters.speed_fdbk_filtered>=-4650 && SIXSTEP_parameters.speed_fdbk_filtered<-4100)
    {
      SIXSTEP_parameters.demagn_value = DEMAGN_VAL_6;
    }
    else if(SIXSTEP_parameters.speed_fdbk_filtered>=-4100 && SIXSTEP_parameters.speed_fdbk_filtered<-3650)
    {
      SIXSTEP_parameters.demagn_value = DEMAGN_VAL_7;
    }
    else if(SIXSTEP_parameters.speed_fdbk_filtered>=-3650 && SIXSTEP_parameters.speed_fdbk_filtered<-3300)
    {
      SIXSTEP_parameters.demagn_value = DEMAGN_VAL_8;
    }
    else if(SIXSTEP_parameters.speed_fdbk_filtered>=-3300 && SIXSTEP_parameters.speed_fdbk_filtered<-2650)
    {
      SIXSTEP_parameters.demagn_value = DEMAGN_VAL_9;
    }
    else if(SIXSTEP_parameters.speed_fdbk_filtered>=-2600 && SIXSTEP_parameters.speed_fdbk_filtered<-1800)
    {
      SIXSTEP_parameters.demagn_value = DEMAGN_VAL_10;
    }
    else if(SIXSTEP_parameters.speed_fdbk_filtered>=-1800 && SIXSTEP_parameters.speed_fdbk_filtered<-1500)
    {
      SIXSTEP_parameters.demagn_value = DEMAGN_VAL_11;
    }
    else if(SIXSTEP_parameters.speed_fdbk_filtered>=-1500 && SIXSTEP_parameters.speed_fdbk_filtered<-1300)
    {
      SIXSTEP_parameters.demagn_value = DEMAGN_VAL_12;
    }
    else if(SIXSTEP_parameters.speed_fdbk_filtered>=-1300 && SIXSTEP_parameters.speed_fdbk_filtered<-1000)
    {
      SIXSTEP_parameters.demagn_value = DEMAGN_VAL_13;
    }
    else if(SIXSTEP_parameters.speed_fdbk_filtered>=-1000 && SIXSTEP_parameters.speed_fdbk_filtered<-500)
    {
      SIXSTEP_parameters.demagn_value = DEMAGN_VAL_14;
    }

  }
}

void set_led_status(uint8_t enable){
  led_status_test = (led_status_test == 0) ? 1: 0;
  HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, (GPIO_PinState) enable);
}








