#ifndef __STM32F3XX_MIT_BOARD_H
#define __STM32F3XX_MIT_BOARD_H

#include "stm32f3xx_hal.h"
#include "stm32_hal_legacy.h"
#include "main.h"
#include "tim.h"
#include "adc.h"


//Board Specifi Defines

#define PWM_TIMER               htim1
#define ERPM_TIMER              htim4

#define ADC_CH_1_ST             ADC_SAMPLETIME_3CYCLES   /*CURRENT sampling time */
#define ADC_CH_2_ST             ADC_SAMPLETIME_84CYCLES  /*SPEED sampling time*/
#define ADC_CH_3_ST             ADC_SAMPLETIME_84CYCLES  /*VBUS sampling time*/
#define ADC_CH_4_ST             ADC_SAMPLETIME_84CYCLES  /*TEMP sampling time*/
#define ADC_Bemf_CH1_ST         ADC_SAMPLETIME_28CYCLES  /*BEMF1 sampling time*/
#define ADC_Bemf_CH2_ST         ADC_SAMPLETIME_28CYCLES  /*BEMF2 sampling time*/
#define ADC_Bemf_CH3_ST         ADC_SAMPLETIME_28CYCLES  /*BEMF3 sampling time*/

#define PWM_TIMER_CH1           TIM_CHANNEL_1
#define PWM_TIMER_CH2           TIM_CHANNEL_2
#define PWM_TIMER_CH3           TIM_CHANNEL_3
#define PWM_TIMER_CCR1          CCR1            /*Channel 1*/
#define PWM_TIMER_CCR2          CCR2            /*Channel 2*/
#define PWM_TIMER_CCR3          CCR3            /*Channel 3*/


#define CONV_ADC_CURRENT        (float)((3.3 / 4096.0) / 4.394 / 0.01)
#define CURRENT_ZERO            1757             /* This equals 0: 1.416 / (3.3 / 4096) */

#define VBUS_MULT               ((float) ((34.2 * 1000) / 4095))

#define TEMP_MULT               ((float) 0.035)

#define THERM_RES_NOMINAL       10000
#define THERM_TEMP_NOMINAL      25
#define TEMP_COEF               3428



enum ADC_ELEMENT_NAMES {
  CURRENT_FBA = 0,
  CURRENT_FBB,
  CURRENT_FBC,
  TEMPERATURE,
  VBUS,
  VSHUNTB,
  BEMF_A,
  BEMF_B,
  BEMF_C
};

// brief  API function for STM32 instruction
void bsp_bemf_delay_calc(void);
void bsp_bemf_adc_capture(void);
void bsp_adc_channel(uint32_t);
void bsp_read_current_channel(uint8_t channel);
void bsp_read_temperature();
void bsp_read_vbus();
void bsp_esc_init(void);
void bsp_enable_input_CH1_E_CH2_E_CH3_D(void);
void bsp_enable_input_CH1_E_CH2_D_CH3_E(void);
void bsp_enable_input_CH1_D_CH2_E_CH3_E(void);
void bsp_disable_input_CH1_D_CH2_D_CH3_D(void);
void bsp_start_pwm_driving(void);
void bsp_stop_pwm_driving(void);
void bsp_pwm_set_duty_cycle_ch1(uint16_t);
void bsp_pwm_set_duty_cycle_ch2(uint16_t);
void bsp_pwm_set_duty_cycle_ch3(uint16_t);
void bsp_enable_erpm_timer(uint8_t enable);
void bsp_set_ol_rpm(uint32_t frequency);
void bsp_set_led_status(uint8_t enable);

#endif
