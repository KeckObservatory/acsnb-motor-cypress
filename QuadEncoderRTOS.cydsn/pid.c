#ifndef __PID_C
#define __PID_C

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "pid.h"

/*******************************************************************************
* Function Name: PID_Compute
********************************************************************************
* Summary:
*  Execute a pass through the PID process to create a duty cycle output.
*
* Parameters: Current destination and encoder value.
* Return: PWM output, from -800 to 800.
*******************************************************************************/
int32_t PID_UpdateValues(int32_t setpoint, int32_t position) {

    /* PID values are all scaled down by a factor of 1000 below.  Use values
       that are pre-scaled up to avoid floating point math. */
    
    /* (hardcoded PID values here, for now) */
    uint16_t Kp = 10000; //5000;
    uint16_t Ki = 100;
    uint16_t Kd = 0;

    /* 2023-07-07 stable values
    uint16_t Kp = 5000;
    uint16_t Ki = 100;
    uint16_t Kd = 0;
    */
        
    /* Constants for use in this algorithm */
    uint16_t max_output = 400;
    uint16_t max_iterm = 250; //150; //250;
    uint16_t pid_scale = 1000;

    /* 2023-07-07 stable values
    uint16_t max_output = 400;
    uint16_t max_iterm = 250;
    uint16_t pid_scale = 1000;
    */
    
    /* Temporary values */
    volatile int32_t error;
    volatile int32_t last_error;
    volatile int32_t Qmax_iterm;
    volatile int32_t Qmax_output;
    volatile int32_t dposition;
    volatile int32_t output;
    static int32_t last_position;
    
    /* Scale the values */
    Qmax_iterm = max_iterm * pid_scale;
    Qmax_output = max_output * pid_scale;
    
    /* Compute all the working error variables */
    error = setpoint - position;    
    
    /* If the iterm_delay is nonzero, count it down to 0ms before using the iterm in the calculation */
    if (iterm_delay > 0) {
        
        iterm_delay = (iterm_delay - ITERM_DELAY_INTERVAL);        
        iterm = 0;
        
    } else {      
    
        /* Outside of the delay period, calculate the iterm normally */
        iterm += (Ki * error);
    }
    
    /* Clip the I term at a max value for just that term (windup guard) */
    if (iterm > Qmax_iterm) {
        iterm = Qmax_iterm;
    } else if (iterm < -Qmax_iterm) {
        iterm = -Qmax_iterm;
    }
    
    /* Calculate the error term */
    dposition = (position - last_position);

    /* Compute PID Output */
    //output = (Kp * error) + iterm - (Kd * dposition);
    output = (Kp * error) + iterm + (Kd * (error - last_error));
    
    /* Clip the output */
    if (output > Qmax_output) {
        output = Qmax_output;
    } else if (output < -Qmax_output) {
        output = -Qmax_output;
    }

    /* Scale the output back down */
    output = output / pid_scale;
    
    /* Remember some variables for next time */
    last_position = position;
    last_error = error;
    
    return output;    
}

#endif /*__PID_C */
