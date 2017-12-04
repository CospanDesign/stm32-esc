
#ifndef __MOTOR_CONTROL_PARAMS_H__
#define __MOTOR_CONTROL_PARAMS_H__

#define NUM_POLE_PAIRS                          7      /* Number of Motor Pole pairs */


#define OL_STARTUP_OVERCURRENT               4000
#define TEMP_HIGH                              50       /* This might need to change later */

/*!< ********************* Open loop control *********************************/
#define MAX_PWM                               256
//#define OL_STARTUP_DUTY_CYCLE                  32     /*StartUP Duty Cycle*/
#define OL_STARTUP_DUTY_CYCLE                  32     /*StartUP Duty Cycle*/
#define OL_MINIMUM_RPM                         60     /* 60 RPM = 1 Hertz */
#define OL_MAX_STARTUP_CURRENT               3000
#define OL_MIN_STARTUP_CURRENT               1000
//#define OL_MAX_STARTUP_CURRENT               10000
#define OL_ACCELLERATION                       10
#define OL_REVOLUTIONS_STEP                  1000     /* Every OL_REVOLUTIONS_STEP add OL_ACCELLERATION to the frequecy */
#define OL_KP_GAIN                              1


/*!< ********************* Closed Loop control *********************************/
#define CL_KP_GAIN                            300      /* Kp parameter for PI regulator */
#define CL_KI_GAIN                            100      /* Ki parameter for PI regulator */
#define CL_MAX_SPEED                        20000      /* Maximum Speed */
#define CL_MIN_SPEED                         2000      /* Minimum Speed */


#endif //__MOTOR_CONTROL_PARAMS_H__
