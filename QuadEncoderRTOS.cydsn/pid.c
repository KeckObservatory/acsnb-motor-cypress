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

    //volatile uint32_t trap = 0;
    
    /* Hardcoded PID values here, for now */
    uint16_t Kp = 8000; //3000; //8000;
    uint16_t Ki = 100;
    uint16_t Kd = 0;
    
    /* Constants for use in this algorithm */
    uint16_t max_output = 400; //600; //800;
    uint16_t max_iterm = 250;
    uint16_t pid_scale = 1000;
    
    
    /* Temporary values */
    volatile int32_t error;
    //static int32_t iterm = 0;
    volatile int32_t Qmax_iterm;
    volatile int32_t Qmax_output;
    volatile int32_t dposition;
    volatile int32_t output;
    static int32_t lastposition;
    
    /* Scale the values */
    Qmax_iterm = max_iterm * pid_scale;
    Qmax_output = max_output * pid_scale;
    
    /* Compute all the working error variables */
    error = setpoint - position;
    iterm += (Ki * error);
    
    //if ((iterm == 12144) || (iterm == -12144)) {
    //    trap++;        
    //}
    
    /* Clip the I term at a max value for just that term (windup guard) */
    if (iterm > Qmax_iterm) {
        iterm = Qmax_iterm;
    } else if (iterm < -Qmax_iterm) {
        iterm = -Qmax_iterm;
    }
    
    /* Calculate the error term */
    dposition = (position - lastposition);

    /* Compute PID Output */
    output = (Kp * error) + iterm - (Kd * dposition);
    
    /* Clip the output */
    if (output > Qmax_output) {
        output = Qmax_output;
    } else if (output < -Qmax_output) {
        output = -Qmax_output;
    }

    /* Scale the output back down */
    output = output / pid_scale;
    
    /* Remember some variables for next time */
    lastposition = position;
    
    return output;    
}

#endif /*__PID_C */
