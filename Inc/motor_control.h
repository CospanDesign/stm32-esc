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

typedef enum
{
    STATE_IDLE                 = 0,
    STATE_OPEN_LOOP            = 1,
    STATE_CLOSED_LOOP          = 2,
    STATE_SPEED_FBK_ERROR      = 10,
    STATE_OVERCURRENT          = 11,
    STATE_STARTUP_FAILURE      = 12,
    STATE_STARTUP_BEMF_FAILURE = 13
} mc_state_t;


typedef struct
{
  mc_state_t state;
  uint8_t step_pos;
  uint16_t pulse_width;
  uint8_t error;
  int16_t reference;
  int32_t rpm;
  int32_t erpm;
  int32_t current_ma;
  uint8_t current_leg_select;
  uint32_t vbus_mv;
  uint32_t temp_c;
  uint8_t idle_adc_select;
}  mc_param_t;

//Closed Loop Parameters
typedef struct
{
  int16_t kp;
  int16_t ki;
} mc_cl_param_t;

//Open Loop Parameters
typedef struct
{
  uint16_t target_rpm;
  uint32_t steps;
  uint32_t step;
  uint32_t revolutions;
  uint32_t over_current_flag;
  uint32_t over_current_count;
} mc_ol_param_t;



void mc_init(void);
void mc_reset(void);
void mc_start_motor(void);
void mc_stop_motor(void);
void mc_set_speed(int16_t);

int32_t mc_get_erpm();
int32_t mc_get_rpm();
int32_t mc_get_current();

uint16_t mc_get_ol_target_rpm();
uint8_t mc_get_error();
uint8_t mc_get_state();
uint32_t mc_get_bus_voltage();
uint32_t mc_get_temperature();

//Debug Info
uint32_t mc_get_adc_sample_count();
uint32_t mc_get_bemf_start_index();
uint32_t mc_get_bemf_buffer_depth();
uint8_t *mc_get_bemf_buffer_handle();

#endif
