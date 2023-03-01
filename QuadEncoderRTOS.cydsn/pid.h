#ifndef __PID_H
#define __PID_H

#include <stdint.h>

#ifdef __PID_C
#define EXTERN /* nothing */
#else
#define EXTERN extern
#endif /* __PID_C */

EXTERN int32_t iterm;
EXTERN int32_t PID_UpdateValues(int32_t setpoint, int32_t encoder);


#endif /*__PID_H */
