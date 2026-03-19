/*
 * Provides prototypes for auxiliary PWM functions
 */

#ifndef pwm_wrapper_h_
#define pwm_wrapper_h_

#include <stdio.h>
#include <math.h>
#include "MyRio.h"
#include "PWM.h"

/*
 * Sets up and enables pwm
 * (based on NI's myRIO Example - PWM,
 *  some comments are quotes)
 */
int pwm_set(MyRio_Pwm* pwm, MyRio1900Fpga60_ControlU8 reg, double duty);



#endif
