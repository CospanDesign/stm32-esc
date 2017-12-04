#include "motor_control.h"

#define DEBUG_MOTOR 1

#define MOTOR_DIRECTION (mc_param.reference > 0)
//These are all set in the motor params
//static uint8_t  num_pole_pairs        = NUM_POLE_PAIRS;
static uint16_t ol_startup_duty_cycle = OL_STARTUP_DUTY_CYCLE;

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
void mc_break_callback(); //Probably due to a debug situation

void mc_systick_task(void);
void mc_erpm_task(uint8_t center);

void mc_open_loop_current_regulator();
void mc_step();
void mc_open_loop_controller();

void mc_closed_loop_controller();

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
  mc_param.error |= mc_param.state;

  //Reset all the parameters
  mc_param.state = STATE_IDLE;
  mc_param.step_pos = 1;
  mc_param.pulse_width = ol_startup_duty_cycle;
  mc_param.reference = CL_MIN_SPEED;
  mc_param.rpm = 0;
  mc_param.erpm = 0;
  mc_param.current_ma = 0;
  mc_param.current_leg_select = 0;
  mc_param.bemf_leg_select = 0;
  mc_param.vbus_mv = 0;
  mc_param.temp_c = 0;
  mc_param.adc_select = 0;
  mc_param.pole_count = 0;
  mc_param.over_current_flag = 0;
  mc_param.over_temp_flag = 0;
  mc_param.revolutions = 0;
  mc_param.rpm_counter = 0;

  //Reset the open loop parameters
  mc_ol_param.target_rpm = (int16_t) OL_MINIMUM_RPM;
  mc_ol_param.steps = 0;
  mc_ol_param.step = 1;
  mc_ol_param.revolution_step_count = 0;
  mc_ol_param.kp = OL_KP_GAIN;

  //Reset the closed loop parameters
  mc_cl_param.kp = CL_KP_GAIN;
  mc_cl_param.ki = CL_KI_GAIN;

  //Make sure the motor is off
  bsp_pwm_set_duty_cycle_ch1(0);
  bsp_pwm_set_duty_cycle_ch2(0);
  bsp_pwm_set_duty_cycle_ch2(0);
}

void mc_start_motor(void)
{
  bsp_disable_input_CH1_D_CH2_D_CH3_D();
  mc_param.state = STATE_OPEN_LOOP;
  //bsp_enable_erpm_timer(1);
}

void mc_stop_motor(void)
{
  bsp_set_led_status(0);
  mc_reset();
  bsp_freewheeling();
  //bsp_enable_erpm_timer(0);
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
  mc_param.error |= (mc_param.over_current_flag << 7);
  mc_param.error |= (mc_param.over_temp_flag << 6);
  return mc_param.error;
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
 * Private Functions
 ****************************************************************************/

void mc_bemf_callback(uint32_t *adc_buffer, uint8_t length)
{
}
void mc_current_ma_callback(int32_t value)
{
  mc_param.current_ma = value;
  //Check to see if they system is drawing too much current
  if (abs(mc_param.current_ma) > (uint32_t) OL_STARTUP_OVERCURRENT)
  {
    //Overcurrent condition
    mc_param.error |= 1 << OVER_CURRENT_FLAG_POS;
    mc_stop_motor();
    return;
  }
}
void mc_vbus_mv_callback(uint32_t value)
{
  mc_param.vbus_mv = value;
}
void mc_temp_c_callback(uint32_t value)
{
  mc_param.temp_c = value;
  //Check temperature
  if (mc_param.temp_c > TEMP_HIGH)
  {
    mc_param.error |= 1 << OVER_TEMP_FLAG_POS;
    mc_stop_motor();
  }
}
void mc_break_callback()
{
  //mc_param.state = STATE_OVERCURRENT;
  mc_stop_motor();
}


/*****************************************************************************
 * Tasks
 ****************************************************************************/

void mc_systick_task(void)
{
  bsp_set_led_status(0);
  switch(mc_param.state){
    case STATE_IDLE:
      switch (mc_param.adc_select){
        case 0:
          bsp_read_vbus();
          mc_param.adc_select++;
          break;
        case 1:
          bsp_read_temperature();
          mc_param.adc_select++;
          break;
        case 2:
          bsp_read_current_channel(1);
          mc_param.adc_select++;
          break;
        case 3:
          bsp_read_current_channel(2);
          mc_param.adc_select++;
          break;
        case 4:
          bsp_read_current_channel(3);
          mc_param.adc_select = 0;
          break;
        default:
          mc_param.adc_select = 0;
          break;
      }
      break;
    default:
      break;
  }
  if (mc_param.rpm_counter < 250){
    mc_param.rpm_counter++;
  }
  else {
    mc_param.rpm = mc_param.revolutions * 4;
    mc_param.revolutions = 0;
  }
}
void mc_erpm_task(uint8_t center)
{
  /* CRAZY CHECKS ARE DONE, NOW TO THE NORMAL STUFF! */
  switch (mc_param.state)
  {
    case (STATE_OPEN_LOOP):
      //If we're not at the center of the cycle then don't read the current, read everything else
      if (center == 0){
        switch (mc_param.adc_select){
          case 0:
            bsp_read_vbus();
            mc_param.adc_select++;
            break;
          case 1:
            bsp_read_temperature();
            mc_param.adc_select = 0;
            break;
          default:
            mc_param.adc_select = 0;
            break;
        }
        mc_open_loop_controller();
      }
      else {
        /*  The center of the PWM cycle is where the current should be
         *  measured, otherwise there is a possibility that the FET will
         *  be transitioning. This will cause erronious measurements
         */
        bsp_read_current_channel(mc_param.current_leg_select);
      }
    break;
    case (STATE_CLOSED_LOOP):
//XXX: For now once we reach the closed loop stage
      mc_stop_motor();
      break;
    default:
      break;
  }
}


/*****************************************************************************
 * Control Functions
 ****************************************************************************/

/* Go to the next step of the PWM step
 *
 */
void mc_step()
{
  #if !defined(DEBUG_MOTOR)
  bsp_set_led_status(1);
  switch (mc_param.step_pos){
    case 1: // 1 -> 2
			bsp_pwm_set_duty_cycle_ch1(mc_param.pulse_width);
			bsp_pwm_set_duty_cycle_ch2(0);
			bsp_pwm_set_duty_cycle_ch3(0);
			bsp_enable_input_CH1_E_CH2_E_CH3_D();
      mc_param.bemf_leg_select = 3;
      mc_param.current_leg_select = 2;  //Read current from channel 2
      break;
    case 2: // 1 -> 3
			bsp_pwm_set_duty_cycle_ch1(mc_param.pulse_width);
			bsp_pwm_set_duty_cycle_ch2(0);
			bsp_pwm_set_duty_cycle_ch3(0);
			bsp_enable_input_CH1_E_CH2_D_CH3_E();
      mc_param.bemf_leg_select = 2;
      mc_param.current_leg_select = 3;  //Read current from channel 3
      break;
    case 3: // 2 -> 3
			bsp_pwm_set_duty_cycle_ch1(0);
			bsp_pwm_set_duty_cycle_ch2(mc_param.pulse_width);
			bsp_pwm_set_duty_cycle_ch3(0);
			bsp_enable_input_CH1_D_CH2_E_CH3_E();
      mc_param.bemf_leg_select = 1;
      mc_param.current_leg_select = 3;  //Read current from channel 3
      break;
    case 4: // 2 -> 1
			bsp_pwm_set_duty_cycle_ch1(0);
			bsp_pwm_set_duty_cycle_ch2(mc_param.pulse_width);
			bsp_pwm_set_duty_cycle_ch3(0);
			bsp_enable_input_CH1_E_CH2_E_CH3_D();
      mc_param.bemf_leg_select = 3;
      mc_param.current_leg_select = 1;  //Read current from channel 1
      break;
    case 5: // 3 -> 1
			bsp_pwm_set_duty_cycle_ch1(0);
			bsp_pwm_set_duty_cycle_ch2(0);
			bsp_pwm_set_duty_cycle_ch3(mc_param.pulse_width);
			bsp_enable_input_CH1_E_CH2_D_CH3_E();
      mc_param.bemf_leg_select = 2;
      mc_param.current_leg_select = 1;  //Read current from channel 1
      break;
    case 6: // 3 -> 2
			bsp_pwm_set_duty_cycle_ch1(0);
			bsp_pwm_set_duty_cycle_ch2(0);
			bsp_pwm_set_duty_cycle_ch3(mc_param.pulse_width);
			bsp_enable_input_CH1_D_CH2_E_CH3_E();
      mc_param.bemf_leg_select = 1;
      mc_param.current_leg_select = 2;  //Read current from channel 2
      break;
  }
  #endif

  //Increment the step
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
 * Open Loop Control Functions
 ****************************************************************************/

/*
 * Every period of the ERPM cycle this is called. It will analyze the 
 */
void mc_open_loop_controller()
{

  mc_step();

  //We use the current feedback to determine the if we can keep this state on
  //We increment the number of steps every time we complete a phase
  if (mc_param.pole_count < (NUM_POLE_PAIRS - 1)){
    mc_param.pole_count++;
  }
  else {
    mc_param.pole_count = 0;
    mc_param.revolutions++;
    mc_ol_param.revolution_step_count += 1;
    //Check if we have reached the number of revolutions to increment the accelleration
    if (mc_ol_param.revolution_step_count >= (OL_REVOLUTIONS_STEP - 1)){
      mc_ol_param.revolution_step_count = 0;
      if (mc_ol_param.target_rpm < CL_MIN_SPEED) {
        mc_ol_param.target_rpm += OL_ACCELLERATION;
        bsp_set_ol_rpm(mc_ol_param.target_rpm);
      }
      else {
        //Done with open loop!
        mc_param.state = STATE_CLOSED_LOOP;
        return;
      }
    }
  }
}

/* Current is detected halfway through the duty cycle, the current is requested during
 * the middle of a cycle, when the current is read, this is called to analyze the results
 */
void mc_open_loop_current_regulator()
{
  if (abs(mc_param.current_ma) < (uint32_t) OL_MIN_STARTUP_CURRENT)
  {
    //Current is not within the amount allowed for startup, increase the PWM duty cycle
    if (mc_param.pulse_width < MAX_PWM - mc_ol_param.kp)
      mc_param.pulse_width += mc_ol_param.kp;
    else
      mc_param.pulse_width = MAX_PWM;
  }
  else if (abs(mc_param.current_ma > (uint32_t) OL_MAX_STARTUP_CURRENT))
  {
    //Current is too high for this current step, reduce it for the next cycle
    if (mc_param.pulse_width > mc_ol_param.kp)
      mc_param.pulse_width -= mc_ol_param.kp;
    else
      mc_param.pulse_width = 0;

    //Since it's too high for this cycle we need to set it to free wheeling so that it doesn't drive at max
    bsp_freewheeling();
  }
}

/*****************************************************************************
 * Closed Loop Control Functions
 ****************************************************************************/
void mc_closed_loop_controller()
{
}

