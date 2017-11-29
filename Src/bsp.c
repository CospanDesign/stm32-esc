#include <math.h>
#include "main.h"
#include "adc.h"
#include "bsp.h"
#include "motor_control.h"
#include "motor_control_params.h"
#include "stm32f3xx_hal.h"
#include "stm32f3xx_hal_gpio.h"

uint8_t led_status_test = 0;
uint32_t current_adc_ch;
uint8_t adc_busy = 0;

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
  {&hadc4, ADC_CHANNEL_3,  ADC_SAMPLETIME_1CYCLE_5},     //Bemf A
  {&hadc4, ADC_CHANNEL_4,  ADC_SAMPLETIME_1CYCLE_5},     //Bemf B
  {&hadc4, ADC_CHANNEL_5,  ADC_SAMPLETIME_1CYCLE_5}      //Bemf C

};

#define BEMF_BUFFER_SIZE 3
uint32_t bemf_buffer[BEMF_BUFFER_SIZE];

extern void mc_bemf_callback(uint32_t *adc_buffer, uint8_t length);
extern void mc_current_ma_callback(int32_t value);
extern void mc_vbus_mv_callback(int32_t value);
extern void mc_temp_c_callback(uint32_t value);
extern void mc_systick_task(void);
extern void erpm_task(uint8_t center);
extern void mc_break_callback();
extern void mc_center_pwm_callback();


/** HAL_ADC_ConvCpltCallback    HAL_ADC_ConvCpltCallback
 *  brief ADC callback
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  int32_t temp;
  float current;
  float steinhart;
  if (hadc->Instance == ADC4){
    mc_bemf_callback(bemf_buffer, BEMF_BUFFER_SIZE);
    return;
  }

  switch (current_adc_ch){
    case CURRENT_FBA:
    case CURRENT_FBB:
    case CURRENT_FBC:
      temp = (int32_t) HAL_ADC_GetValue(hadc) - CURRENT_ZERO;
      current = temp * 18.3;
      temp = rintf(current);
      mc_current_ma_callback(temp);
      break;
    case VBUS:
      //Conver to mV
      mc_vbus_mv_callback(rint(HAL_ADC_GetValue(hadc) * VBUS_MULT));
      break;
    case TEMPERATURE:
      //Convert to Centigrade
      steinhart = HAL_ADC_GetValue(hadc);
      steinhart = ((4700 * 4096) / steinhart) - 4700;
      steinhart = steinhart / THERM_RES_NOMINAL;
      steinhart = log(steinhart);
      steinhart = steinhart / TEMP_COEF;
      steinhart = steinhart + (1.0 / (THERM_TEMP_NOMINAL + 273.15));
      steinhart = 1.0 / steinhart;
      steinhart -= 273.15;
      //mc_steinhart_c_callback(rint(HAL_ADC_GetValue(hadc) * TEMP_MULT));
      mc_temp_c_callback(rint(steinhart));
      break;
    default:
      break;
  } 
  adc_busy = 0;
}

/** HAL_TIM_PeriodElapsedCallback    HAL_TIM_PeriodElapsedCallback
 *  brief htim callback
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  //Called at the center of a PWM cycle
  if (htim->Instance == ERPM_TIMER.Instance){
    erpm_task(__HAL_TIM_IS_TIM_COUNTING_DOWN(&PWM_TIMER));
  }
  else if (htim->Instance == PWM_TIMER.Instance){
    if (!__HAL_TIM_IS_TIM_COUNTING_DOWN(&PWM_TIMER)){
      //This is called in the middle of a PWM Cycle, the best time to capture back emf and Current Driver
      mc_center_pwm_callback();
    }
  }
}

void HAL_TIMEx_BreakCallback(TIM_HandleTypeDef *htim)
{
  //Not yet implemented, this signal needs to be connected to an outside line
  mc_break_callback();
}

/** System SYSTICK callback
 *  Millisecond Callback
 */
void HAL_SYSTICK_Callback()
{
  mc_systick_task();
}

uint8_t bsp_adc_busy(){
  return adc_busy;
}

/**
 * Call an individual channel for the ADC
 */
void bsp_adc_channel(uint32_t adc_ch)
{
  if (adc_busy){
    //XXX: THIS COULD CAUSE A HEADACHE LATER!!!
    return;
  }
  adc_busy = 1;
  current_adc_ch = adc_ch;
  ADC_ChannelConfTypeDef bsp_adc_chan;

  bsp_adc_chan.Channel = ADC_SEL[adc_ch].index;
  bsp_adc_chan.Rank = 1;
  bsp_adc_chan.SingleDiff = ADC_SINGLE_ENDED;
  bsp_adc_chan.SamplingTime = ADC_SEL[adc_ch].sample_time;
  bsp_adc_chan.OffsetNumber = ADC_OFFSET_NONE;
  bsp_adc_chan.Offset = 0;
  HAL_ADC_ConfigChannel(ADC_SEL[adc_ch].adc, &bsp_adc_chan);
  HAL_ADC_Start_IT(ADC_SEL[adc_ch].adc);
}

void inline bsp_bemf_adc_capture()
{
  HAL_ADC_Start_IT(&hadc4);
}

void inline bsp_read_current_channel(uint8_t channel)
{
  switch(channel){
    case 1:
      bsp_adc_channel(CURRENT_FBA);
      break;
    case 2:
      bsp_adc_channel(CURRENT_FBB);
      break;
    case 3:
      bsp_adc_channel(CURRENT_FBC);
      break;
    default:
      break;
  }
}

void bsp_read_temperature()
{
  bsp_adc_channel(TEMPERATURE);
}
void bsp_read_vbus()
{
  bsp_adc_channel(VBUS);
}

void bsp_esc_init()
{
  adc_busy = 0;

  __HAL_TIM_SetAutoreload(&PWM_TIMER, (uint16_t) MAX_PWM & 0xFFFF);

/*
  HAL_ADC_Start_DMA(&hadc4, (uint32_t *)bemf_buffer, BEMF_BUFFER_SIZE);

*/
  __HAL_FREEZE_TIM1_DBGMCU();  // Stop TIM during Breakpoint
  __HAL_TIM_ENABLE_IT(&PWM_TIMER, TIM_IT_BREAK); // Enable the TIM Break interrupt
  __HAL_TIM_ENABLE_IT(&PWM_TIMER, TIM_IT_UPDATE);
}

/**  bsp_enable_input_CH1_E_CH2_E_CH3_D    bsp_enable_input_CH1_E_CH2_E_CH3_D
 *  @{
 * brief Enable Input channel for STSPIN230
 */
void bsp_enable_input_CH1_E_CH2_E_CH3_D()
{
  HAL_TIM_PWM_Start(&PWM_TIMER,PWM_TIMER_CH1);           //TIM1_CH1 ENABLE
  HAL_TIMEx_PWMN_Start(&PWM_TIMER,PWM_TIMER_CH1) ;

  HAL_TIM_PWM_Start(&PWM_TIMER,PWM_TIMER_CH2);           //TIM1_CH2 ENABLE  
  HAL_TIMEx_PWMN_Start(&PWM_TIMER,PWM_TIMER_CH2) ;

  HAL_TIM_PWM_Stop(&PWM_TIMER,PWM_TIMER_CH3);           //TIM1_CH3 DISABLE  
  HAL_TIMEx_PWMN_Stop(&PWM_TIMER,PWM_TIMER_CH3) ;  
}

/**  bsp_enable_input_CH1_E_CH2_D_CH3_E    bsp_enable_input_CH1_E_CH2_D_CH3_E
 *  @{
 * brief Enable Input channel for STSPIN230
 */
void  bsp_enable_input_CH1_E_CH2_D_CH3_E()
{
  HAL_TIM_PWM_Start(&PWM_TIMER,PWM_TIMER_CH1);           //TIM1_CH1 ENABLE 
  HAL_TIMEx_PWMN_Start(&PWM_TIMER,PWM_TIMER_CH1) ;

  HAL_TIM_PWM_Stop(&PWM_TIMER,PWM_TIMER_CH2);           //TIM1_CH2  DISABLE 
  HAL_TIMEx_PWMN_Stop(&PWM_TIMER,PWM_TIMER_CH2) ;

  HAL_TIM_PWM_Start(&PWM_TIMER,PWM_TIMER_CH3);           //TIM1_CH3 ENABLE  
  HAL_TIMEx_PWMN_Start(&PWM_TIMER,PWM_TIMER_CH3) ;    
}

/**  bsp_enable_input_CH1_D_CH2_E_CH3_E    bsp_enable_input_CH1_D_CH2_E_CH3_E
 *  @{
 * brief Enable Input channel for STSPIN230
 */
void bsp_enable_input_CH1_D_CH2_E_CH3_E()
{
  HAL_TIM_PWM_Stop(&PWM_TIMER,PWM_TIMER_CH1);           //TIM1_CH1 DISABLE 
  HAL_TIMEx_PWMN_Stop(&PWM_TIMER,PWM_TIMER_CH1) ;

  HAL_TIM_PWM_Start(&PWM_TIMER,PWM_TIMER_CH2);           //TIM1_CH2 ENABLE  
  HAL_TIMEx_PWMN_Start(&PWM_TIMER,PWM_TIMER_CH2) ;

  HAL_TIM_PWM_Start(&PWM_TIMER,PWM_TIMER_CH3);           //TIM1_CH3 ENABLE  
  HAL_TIMEx_PWMN_Start(&PWM_TIMER,PWM_TIMER_CH3) ;  
}

/**  bsp_disable_input_CH1_D_CH2_D_CH3_D    bsp_disable_input_CH1_D_CH2_D_CH3_D
 *  @{
 * brief Disable All Input channels for STSPIN230
 */
void bsp_disable_input_CH1_D_CH2_D_CH3_D()
{
  HAL_TIM_PWM_Stop(&PWM_TIMER,PWM_TIMER_CH1);           //TIM1_CH1 DISABLE 
  HAL_TIMEx_PWMN_Stop(&PWM_TIMER,PWM_TIMER_CH1) ;

  HAL_TIM_PWM_Stop(&PWM_TIMER,PWM_TIMER_CH2);           //TIM1_CH2  DISABLE 
  HAL_TIMEx_PWMN_Stop(&PWM_TIMER,PWM_TIMER_CH2) ;

  HAL_TIM_PWM_Stop(&PWM_TIMER,PWM_TIMER_CH3);           //TIM1_CH3 DISABLE  
  HAL_TIMEx_PWMN_Stop(&PWM_TIMER,PWM_TIMER_CH3) ;  
}


void bsp_freewheeling()
{
  bsp_pwm_set_duty_cycle_ch1(0);
  bsp_pwm_set_duty_cycle_ch2(0);
  bsp_pwm_set_duty_cycle_ch2(0);

  HAL_TIM_PWM_Start(&PWM_TIMER,PWM_TIMER_CH1);           //Enable only the lowside
  HAL_TIMEx_PWMN_Start(&PWM_TIMER,PWM_TIMER_CH1) ;

  HAL_TIM_PWM_Start(&PWM_TIMER,PWM_TIMER_CH2);           //Enable only the lowside
  HAL_TIMEx_PWMN_Start(&PWM_TIMER,PWM_TIMER_CH2) ;

  HAL_TIM_PWM_Start(&PWM_TIMER,PWM_TIMER_CH3);           //Enable only the low side
  HAL_TIMEx_PWMN_Start(&PWM_TIMER,PWM_TIMER_CH3) ;  

}

/**  bsp_start_pwm_driving    bsp_start_pwm_driving
 *  @{
 * brief Enable the PWM generation on Input channels
 */
void bsp_start_pwm_driving()
{
  HAL_TIM_PWM_Start(&PWM_TIMER, PWM_TIMER_CH1);           //TIM1_CH1 ENABLE   
  HAL_TIMEx_PWMN_Start(&PWM_TIMER,PWM_TIMER_CH1) ;  

  HAL_TIM_PWM_Start(&PWM_TIMER, PWM_TIMER_CH2);           //TIM1_CH2 ENABLE   
  HAL_TIMEx_PWMN_Start(&PWM_TIMER,PWM_TIMER_CH2) ;  

  HAL_TIM_PWM_Start(&PWM_TIMER, PWM_TIMER_CH3);           //TIM1_CH3 ENABLE  
  HAL_TIMEx_PWMN_Start(&PWM_TIMER,PWM_TIMER_CH3) ;  

}

/**  bsp_stop_pwm_driving    bsp_stop_pwm_driving
 *  @{
 * brief Disable the PWM generation on Input channels
 */
void bsp_stop_pwm_driving()
{
  HAL_TIM_PWM_Stop(&PWM_TIMER, PWM_TIMER_CH1);           //TIM1_CH1 DISABLE   
  HAL_TIMEx_PWMN_Stop(&PWM_TIMER,PWM_TIMER_CH1) ;  

  HAL_TIM_PWM_Stop(&PWM_TIMER, PWM_TIMER_CH2);           //TIM1_CH2 DISABLE   
  HAL_TIMEx_PWMN_Stop(&PWM_TIMER,PWM_TIMER_CH2) ;  

  HAL_TIM_PWM_Stop(&PWM_TIMER, PWM_TIMER_CH3);           //TIM1_CH3 DISABLE  
  HAL_TIMEx_PWMN_Stop(&PWM_TIMER,PWM_TIMER_CH3) ;  
}


void bsp_pwm_set_duty_cycle_percent(uint32_t channel, float percent_duty_cycle)
{
  uint16_t period;
  uint16_t pulse;

  period = __HAL_TIM_GetAutoreload(&PWM_TIMER);
  if (percent_duty_cycle == 0){
    pulse = 0;
  }
  else {
    pulse = (uint16_t) rintf(((float) period) / 100.0 * percent_duty_cycle);
  }
  __HAL_TIM_SET_COMPARE(&PWM_TIMER, channel, pulse);
}

/**  bsp_pwm_set_duty_cycle_ch1    bsp_pwm_set_duty_cycle_ch1
 *  @{
 * brief Set the Duty Cycle value for CH1
 */
void bsp_pwm_set_duty_cycle_ch1(uint16_t CCR_value)
{
  //PWM_TIMER.Instance->PWM_TIMER_CCR1 = CCR_value;  
  __HAL_TIM_SET_COMPARE(&PWM_TIMER, TIM_CHANNEL_1, CCR_value);
}

/**  bsp_pwm_set_duty_cycle_ch2    bsp_pwm_set_duty_cycle_ch2
 *  @{
 * brief Set the Duty Cycle value for CH2
 */
void bsp_pwm_set_duty_cycle_ch2(uint16_t CCR_value)
{
  //PWM_TIMER.Instance->PWM_TIMER_CCR2 = CCR_value;  
  __HAL_TIM_SET_COMPARE(&PWM_TIMER, TIM_CHANNEL_2, CCR_value);
}

/**  bsp_pwm_set_duty_cycle_ch3    bsp_pwm_set_duty_cycle_ch3
 *  @{
 * brief Set the Duty Cycle value for CH3
 */
void bsp_pwm_set_duty_cycle_ch3(uint16_t CCR_value)
{
  //PWM_TIMER.Instance->PWM_TIMER_CCR3 = CCR_value;  
  __HAL_TIM_SET_COMPARE(&PWM_TIMER, TIM_CHANNEL_3, CCR_value);
}

void bsp_enable_erpm_timer(uint8_t enable)
{
  if (enable) 
	  HAL_TIM_Base_Start_IT(&ERPM_TIMER);
  else
	  HAL_TIM_Base_Stop_IT(&ERPM_TIMER);
}

void bsp_set_ol_rpm(uint32_t rpm_request)
{
  //This is a combination of the auto reload register and clock divisor
  //If there is no divisor the lowest rpm I can get to is 72MHz
  uint32_t rpm_ticks = HAL_RCC_GetSysClockFreq() / rpm_request / NUM_POLE_PAIRS;
  uint16_t prescaler = 1;

  while (rpm_ticks > 0xFFFF) {
    prescaler = prescaler << 1;
    rpm_ticks = rpm_ticks >> 1;
  }

  //Update Prescaler
  ERPM_TIMER.Init.Prescaler = (uint16_t) prescaler & 0xFFFF;
  __HAL_TIM_SET_PRESCALER(&ERPM_TIMER, (uint16_t) prescaler & 0xFFFF);
  //Update Ticks
  __HAL_TIM_SetAutoreload(&ERPM_TIMER, (uint16_t) rpm_ticks & 0xFFFF);
}

void bsp_set_led_status(uint8_t enable)
{
  led_status_test = (led_status_test == 0) ? 1: 0;
  HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, (GPIO_PinState) enable);
}


