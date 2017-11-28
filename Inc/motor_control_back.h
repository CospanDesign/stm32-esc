/**
 ******************************************************************************
 * @file    motor_control.h
 * @author  System lab
 * @version V1.0.0
 * @date    06-July-2015
 * @brief   This header file provides the set of functions for Motor Control
            library
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
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
#ifndef __6STEP_LIB_H
#define __6STEP_LIB_H

#include "bsp.h"
#include "motor_control_params.h"

#include "math.h"
#include "stdlib.h"
#include "stdio.h"

/** @addtogroup MIDDLEWARES     MIDDLEWARES
  * @brief  Middlewares Layer
  * @{
  */


/** @addtogroup mc_6-STEP_LIB    mc_6-STEP_LIB
  * @brief  Motor Control driver
  * @{
  */

/** @defgroup Exported_types  Exported_types
* @{
*/
/**
  * @brief  Six Step parameters
  */
typedef enum
{
    IDLE,                               /* 0 */
    STARTUP,                            /* 1 */
    VALIDATION,                         /* 2 */
    STOP,                               /* 3 */
    START,                              /* 4 */
    RUN,                                /* 5 */
    ALIGNMENT,                          /* 6 */
    SPEEDFBKERROR,                      /* 7 */
    OVERCURRENT,                        /* 8 */
    STARTUP_FAILURE,                    /* 9 */
    STARTUP_BEMF_FAILURE                /* 10 */
} mc_status_t;

/**
  * @}
  */


/** @defgroup Exported_types  Exported_types
* @{
*/
/**
  * @brief  Six Step parameters
  */
typedef struct
{
  uint32_t ERPM_TIMER_PSC;               /*!< Prescaler variable for low frequency timer*/
  uint32_t ERPM_TIMER_ARR;               /*!< ARR variable for low frequency timer*/
  uint32_t PWM_TIMER_PSC;                /*!< Prescaler variable for high frequency timer*/
  uint32_t PWM_TIMER_ARR;                /*!< ARR variable for high frequency timer*/
  uint32_t PWM_TIMER_CCR;                /*!< CCR variable for high frequency timer*/
  uint8_t step_position;                 /*!< Step number variable for SixStep algorithm*/
  mc_status_t STATUS;                    /*!< Status variable for SixStep algorithm*/
  uint8_t  status_prev;                  /*!< Previous status variable for SixStep algorithm*/
  uint16_t pulse_width;                  /*!< CCR value for SixStep algorithm*/
  uint16_t ARR_value;                    /*!< ARR vector for Accell compute*/
  uint32_t Regular_channel[4];           /*!< Buffer for ADC regular channel */
  uint32_t CurrentRegular_BEMF_ch;       /*!< ADC regular channel to select */
  uint32_t prescaler_value;              /*!< Prescaler value for low freq timer*/
  uint16_t numberofitemArr;              /*!< Number of elements */
  uint32_t ADC_BUFFER[4];                /*!< Buffer for ADC regular channel */
  uint32_t ADC_SEQ_CHANNEL[5];           /*!< Buffer for ADC regular channel */
  uint32_t ADC_Regular_Buffer[5];        /*!< Buffer for ADC regular channel */
  uint16_t ADC_BEMF_threshold_UP;        /*!< Voltage threshold for BEMF detection in up direction*/
  uint16_t ADC_BEMF_threshold_DOWN;      /*!< Voltage threshold for BEMF detection in down direction*/
  uint16_t demagn_counter;               /*!< Demagnetization counter*/
  uint16_t demagn_value;                 /*!< Demagnetization value*/
  int16_t speed_fdbk;                    /*!< Motor speed variable*/
  int16_t speed_fdbk_filtered;           /*!< Filtered Motor speed variable*/
  int16_t filter_depth;                  /*!< Filter depth for speed measuring*/
  uint16_t Current_Reference;            /*!< Currrent reference for SixStep algorithm*/
  uint16_t i_ref;                        /*!< Currrent reference for SixStep algorithm*/
  int32_t Integral_Term_sum;             /*!< Global Integral part for PI*/
  uint8_t CMD;                           /*!< Flag control for Motor Start/Stop*/
  uint8_t ALIGN_OK;                      /*!< Flag control for Motor Alignment*/
  uint8_t ALIGNMENT;                     /*!< Flag control for Motor Alignment ongoing*/
  uint8_t bemf_state_1;                  /*!< Bemf variable */
  uint8_t bemf_state_2;                  /*!< Bemf variable */
  uint8_t bemf_state_3;                  /*!< Bemf variable */
  uint8_t bemf_state_4;                  /*!< Bemf variable */
  uint8_t bemf_state_5;                  /*!< Bemf variable */
  uint8_t bemf_state_6;                  /*!< Bemf variable */
  uint16_t Speed_Loop_Time;              /*!< Speed loop variable for timing */
  uint8_t ARR_OK;                        /*!< ARR flag control for Accell status */
  uint8_t VALIDATION_OK;                 /*!< Validation flag for Closed loop control begin */
  uint8_t SPEED_VALIDATED;               /*!< Validation flag for Speed before closed loop control */
  uint16_t Speed_target_ramp;            /*!< Target Motor Speed */
  uint16_t Speed_target_time;            /*!< Target Motor Ramp time */
  uint16_t Ramp_Start;                   /*!< Ramp time start*/
  uint16_t bemf_delay_start;             /*!< Bemf variable */
  uint16_t MediumFrequencyTask_flag;     /*!< Flag for Medium Task Frequency */
  uint32_t SYSCLK_frequency;             /*!< System clock main frequency */
  uint8_t BEMF_OK;                       /*!<  */
  uint8_t CL_READY;                      /*!<  */
  uint8_t BEMF_Tdown_count;              /*!< BEMF Consecutive Threshold Falling Crossings Counter */
  uint16_t IREFERENCE;                   /*!< Currrent reference*/
  uint16_t NUMPOLESPAIRS;                /*!< Number of motor pole pairs  */
  uint32_t ACCEL;                        /*!< Acceleration start-up parameter*/
  uint16_t KP;                           /*!< KP parameter for PI regulator */
  uint16_t KI;                           /*!< KI parameter for PI regulator */
}  mc_param_t;             /*!< Six Step Data Structure */

/**
  * @}
  */

/** @defgroup Exported_types  Exported_types
* @{
*/
/**
  * @brief  Six PI regulator parameters
  */

typedef struct
{
  int16_t Reference;                    /*!< Refence value for PI regulator */
  int16_t Kp_Gain;                      /*!< Kp value for PI regulator */
  int16_t Ki_Gain;                      /*!< Ki value for PI regulator */
  int16_t Lower_Limit_Output;           /*!< Min output value for PI regulator */
  int16_t Upper_Limit_Output;           /*!< Max output value for PI regulator */
  int8_t Max_PID_Output;                /*!< Max Saturation indicator flag */
  int8_t Min_PID_Output;                /*!< Min Saturation indicator flag */
} mc_pi_param_t, *SIXSTEP_pi_PARAM_InitTypeDef_t;  /*!< PI Data Structure */

/**
  * @}
  */

/** @defgroup Exported_function_6StepLib  Exported_function_6StepLib
* @{
*/

void mc_init(void);
void mc_reset(void);
void mc_start_motor(void);
void mc_stop_motor(void);
void mc_set_speed(int16_t);

int32_t mc_get_el_rpm();
int32_t mc_get_mech_rpm();

//Helpful Info
uint32_t mc_get_adc_sample_count();
uint32_t mc_get_bemf_start_index();
uint32_t mc_get_bemf_buffer_depth();
uint8_t *mc_get_bemf_buffer_handle();

/**
  * @}
  */

/**  mc_6-STEP_LIB
  * @}
  */

/**  MIDDLEWARES
  * @}
  */

#endif
