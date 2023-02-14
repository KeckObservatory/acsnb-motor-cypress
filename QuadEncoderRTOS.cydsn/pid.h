#ifndef __PID_H
#define __PID_H

#include <stdint.h>

extern int32_t PID_UpdateValues(int32_t setpoint, int32_t encoder);

#endif /*__PID_H */
