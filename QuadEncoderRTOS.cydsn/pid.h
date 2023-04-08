#ifndef __PID_H
#define __PID_H

#include <stdint.h>

#ifdef __PID_C
#define EXTERN /* nothing */
#else
#define EXTERN extern
#endif /* __PID_C */

#define ITERM_DELAY_DEFAULT 60000 // miroseconds to delay before engaging the iterm
#define ITERM_DELAY_INTERVAL 400  // microseconds between calls to PID loop, based on the BRMS rate group

EXTERN int32_t iterm;
EXTERN int32_t iterm_delay;
EXTERN int32_t PID_UpdateValues(int32_t setpoint, int32_t encoder);

#endif /*__PID_H */
