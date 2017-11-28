#include "motor_control.h"

//#define DEBUG_MOTOR 1

#define MOTOR_DIRECTION (mc_param.reference > 0)
//These are all set in the motor params
//static uint8_t  num_pole_pairs        = NUM_POLE_PAIRS;

mc_param_t mc_param;            // Motor Controller Parameters
mc_cl_param_t mc_cl_param;     // Closed loop parameters
mc_ol_param_t mc_ol_param;     // Open Loop parameters

/*****************************************************************************
 * Local prototypes
 ****************************************************************************/
void mc_bemf_callback(uint32_t *adc_buffer, uint8_t length);
void mc_current_ma_callback(int32_t value);
void mc_vbus_mv_callback(uint32_t value);
void mc_temp_c_callback(uint32_t value);

void mc_systick_task(void);
void erpm_task(void);

void open_loop_current_regulator();
void open_loop_step();
void open_loop_controller();

void closed_loop_controller();

/*****************************************************************************
 * User Interface
 ****************************************************************************/
void mc_init(void)
{
  bsp_esc_init();
  mc_param.error = 0;
  mc_param.state = STATE_IDLE;
  mc_reset();
}
void mc_reset(void)
{
  //Record the error of the motor controller
  switch(mc_param.state){
    case STATE_SPEED_FBK_ERROR:
    case STATE_OVERCURRENT:
    case STATE_STARTUP_FAILURE:
    case STATE_STARTUP_BEMF_FAILURE:
      mc_param.error = mc_param.state;
    break;
    default:
      mc_param.error = 0;
    break;
  }

  //Reset all the parameters
  mc_param.state = STATE_IDLE;
  mc_param.step_pos = 1;
  mc_param.pulse_width = (uint16_t) OL_STARTUP_DUTY_CYCLE;
  mc_param.reference = CL_MIN_SPEED;
  mc_param.rpm = 0;
  mc_param.erpm = 0;
  mc_param.current_ma = 0;
  mc_param.current_leg_select = 0;
  mc_param.vbus_mv = 0;
  mc_param.temp_c = 0;
  mc_param.idle_adc_select = 0;

  //Reset the open loop parameters
  mc_ol_param.target_rpm = (int16_t) OL_MINIMUM_RPM;
  mc_ol_param.steps = 0;
  mc_ol_param.step = 1;
  mc_ol_param.revolutions = 0;
  mc_ol_param.over_current_flag = 0;
  mc_ol_param.over_current_count = 0;

  //Reset the closed loop parameters
  mc_cl_param.kp = CL_KP_GAIN;
  mc_cl_param.ki = CL_KI_GAIN;
}

void mc_start_motor(void)
{
  mc_param.state = STATE_OPEN_LOOP;
  bsp_enable_erpm_timer(1);
}

void mc_stop_motor(void)
{
  bsp_enable_erpm_timer(0);
  bsp_set_led_status(0);
  mc_reset();
}

/* mc_set_speed: Set the speed of the motor (Positive or negative)
 * -CL_MAX_SPEED - CL_MAX_SPEED0
 */
void mc_set_speed(int16_t speed)
{
  if (abs(speed) < CL_MIN_SPEED){
    if (speed < 0)
      mc_param.reference = -CL_MIN_SPEED;
    else
      mc_param.reference = CL_MIN_SPEED;
  }
  else if (abs(speed) > CL_MAX_SPEED){
    if (speed < 0)
      mc_param.reference = -CL_MAX_SPEED;
    else
      mc_param.reference = CL_MAX_SPEED;
  }
  else
    mc_param.reference = speed;
}

int32_t mc_get_erpm()
{
  return mc_param.erpm;
}
int32_t mc_get_rpm()
{
  return mc_param.rpm;
}

uint16_t mc_get_ol_target_rpm()
{
  return mc_ol_param.target_rpm;
}

uint8_t mc_get_error()
{
  return (mc_ol_param.over_current_count > 0) ? mc_param.error | 0x80 : mc_param.error;
}


uint8_t mc_get_state()
{
  return (uint8_t) mc_param.state;
}

int32_t mc_get_current()
{
  return mc_param.current_ma;
}

uint32_t mc_get_bus_voltage()
{
  return mc_param.vbus_mv;
}

uint32_t mc_get_temperature()
{
  return mc_param.temp_c;
}


/*****************************************************************************
 * Debug Info (Public Functions)
 ****************************************************************************/
uint32_t mc_get_adc_sample_count()
{
  return 0;
}
uint32_t mc_get_bemf_start_index()
{
  return 0;
}
uint32_t mc_get_bemf_buffer_depth()
{
  return 0;
}
uint8_t *mc_get_bemf_buffer_handle()
{
  return NULL;
}


/*****************************************************************************
 * Private Functions
 ****************************************************************************/

void mc_bemf_callback(uint32_t *adc_buffer, uint8_t length)
{
}
void mc_current_ma_callback(int32_t value)
{
  mc_param.current_ma = value;
  open_loop_current_regulator();
}
void mc_vbus_mv_callback(uint32_t value)
{
  mc_param.vbus_mv = value;
}
void mc_temp_c_callback(uint32_t value)
{
  mc_param.temp_c = value;
}

void mc_break_callback()
{
  mc_param.state = STATE_OVERCURRENT;
  mc_stop_motor();
}


void mc_systick_task(void)
{

  bsp_set_led_status(0);
  switch(mc_param.state){
    case STATE_IDLE:
      switch (mc_param.idle_adc_select){
        case 0:
          bsp_read_vbus();
          mc_param.idle_adc_select++;
          break;
        case 1:
          bsp_read_temperature();
          mc_param.idle_adc_select++;
          break;
        case 2:
          bsp_read_current_channel(1);
          mc_param.idle_adc_select++;
        case 3:
          bsp_read_current_channel(2);
          mc_param.idle_adc_select++;
        case 4:
          bsp_read_current_channel(3);
          mc_param.idle_adc_select = 0;
      }
      break;
    case STATE_OPEN_LOOP:
      bsp_read_current_channel(mc_param.current_leg_select);
      break;
    default:
      break;
  }
}

void erpm_task(void)
{
  if (mc_param.state == STATE_OPEN_LOOP){
    open_loop_controller();
  }
}

/*****************************************************************************
 * Control Functions
 ****************************************************************************/

/*****************************************************************************
 * Open Loop Control Functions
 ****************************************************************************/
void open_loop_current_regulator()
{
  if (abs(mc_param.current_ma) > (uint32_t) OL_MAX_STARTUP_CURRENT){
    //Disable the PWM until this current drops below this level
    mc_ol_param.over_current_count++;
    mc_ol_param.over_current_flag = 1;
  }
  else
    mc_ol_param.over_current_flag = 0;

  if (mc_param.state == STATE_OPEN_LOOP)
    open_loop_step();
}

void open_loop_step()
{
  //Check for over current conditions
  if (mc_ol_param.over_current_flag){
    //Disable all outputs
    bsp_disable_input_CH1_D_CH2_D_CH3_D();
    return;
  }

  #if !defined(DEBUG_MOTOR)
  bsp_set_led_status(1);
  switch (mc_param.step_pos){
    case 1: // 1 -> 2
			bsp_pwm_set_duty_cycle_ch1(mc_param.pulse_width);
			bsp_pwm_set_duty_cycle_ch2(0);
			bsp_pwm_set_duty_cycle_ch3(0);
			bsp_enable_input_CH1_E_CH2_E_CH3_D();
      mc_param.current_leg_select = 2;  //Read current from channel 2
      break;
    case 2: // 1 -> 3
			bsp_pwm_set_duty_cycle_ch1(mc_param.pulse_width);
			bsp_pwm_set_duty_cycle_ch2(0);
			bsp_pwm_set_duty_cycle_ch3(0);
			bsp_enable_input_CH1_E_CH2_D_CH3_E();
      mc_param.current_leg_select = 3;  //Read current from channel 3
      break;
    case 3: // 2 -> 3
			bsp_pwm_set_duty_cycle_ch1(0);
			bsp_pwm_set_duty_cycle_ch2(mc_param.pulse_width);
			bsp_pwm_set_duty_cycle_ch3(0);
			bsp_enable_input_CH1_D_CH2_E_CH3_E();
      mc_param.current_leg_select = 3;  //Read current from channel 3
      break;
    case 4: // 2 -> 1
			bsp_pwm_set_duty_cycle_ch1(0);
			bsp_pwm_set_duty_cycle_ch2(mc_param.pulse_width);
			bsp_pwm_set_duty_cycle_ch3(0);
			bsp_enable_input_CH1_E_CH2_E_CH3_D();
      mc_param.current_leg_select = 1;  //Read current from channel 1
      break;
    case 5: // 3 -> 1
			bsp_pwm_set_duty_cycle_ch1(0);
			bsp_pwm_set_duty_cycle_ch2(0);
			bsp_pwm_set_duty_cycle_ch3(mc_param.pulse_width);
			bsp_enable_input_CH1_E_CH2_D_CH3_E();
      mc_param.current_leg_select = 1;  //Read current from channel 1
      break;
    case 6: // 3 -> 2
			bsp_pwm_set_duty_cycle_ch1(0);
			bsp_pwm_set_duty_cycle_ch2(0);
			bsp_pwm_set_duty_cycle_ch3(mc_param.pulse_width);
			bsp_enable_input_CH1_D_CH2_E_CH3_E();
      mc_param.current_leg_select = 2;  //Read current from channel 2
      break;
  }
  #endif
}

void open_loop_controller()
{

  //We use the current feedback to determine the if we can keep this state on
  //We increment the number of steps every time we complete a phase
  if ((mc_ol_param.steps % 6) == 0){
    if (mc_ol_param.steps > 0) {
      //Reached another revolution
      mc_ol_param.revolutions += 1;
    }
  }
  if ((mc_ol_param.revolutions % OL_REVOLUTIONS_STEP) == 0){
    //We have established ourselves at this revolution, it's time to go to the next one
    if (mc_ol_param.target_rpm < CL_MIN_SPEED) {
      bsp_set_ol_rpm(mc_ol_param.target_rpm);
      mc_ol_param.target_rpm += OL_ACCELLERATION;
    }
    else {
      //We are done with open loop, now it's time to switch to closed loop
      mc_param.state = STATE_CLOSED_LOOP;
      return;
    }
  }

  open_loop_step();

  mc_ol_param.steps += 1;
  if (MOTOR_DIRECTION){
    if (mc_param.step_pos >= 6)
      mc_param.step_pos = 1;
    else
      mc_param.step_pos++;
  }
  else {
    if (mc_param.step_pos <= 1)
      mc_param.step_pos = 6;
    else
      mc_param.step_pos--;
  }


}

/*****************************************************************************
 * Closed Loop Control Functions
 ****************************************************************************/
void closed_loop_controller()
{
}

