/*******************************************************************************
* File Name: main.c
* Author: Paul Richards, W.M. Keck Observatory
*   (Adapted from original non-RTOS version from Kona Scientific, JP Fumo)
*
* Description:
*  This file provides a SPI interface for the Quadrature Encoder for the ACS project.
*  The I2C device provides readback of motor current consumption.
*
* History:
* 10/11/18 PMR  Rev: A  Implement PWM functions for PDI control
* 07/31/18 PMR  Rev: NC Initial Release after port from Kona Scientific code
*******************************************************************************/
#include <FreeRTOS.h>
#include <semphr.h>
#include <task.h>
#include <I2C_I2C.h>
#include <stdbool.h>
#include <math.h>
#include "INA219.h"

/* --------------------------------------------------------------------- 
 * CONSTANTS
 * --------------------------------------------------------------------- */

/* Interrupt priorities (not to be confused with RTOS task priorities) */
#define NESTED_ISR                          (1u)
#define DEFAULT_PRIORITY                    (3u)
#define HIGHER_PRIORITY                     (2u)

/* Interrupt prototypes */
CY_ISR_PROTO(HomeIsrHandler);
CY_ISR_PROTO(RSTIsrHandler);
CY_ISR_PROTO(SPI_IsrHandler);
CY_ISR_PROTO(SPI_SS_IsrHandler);


/* --------------------------------------------------------------------- 
 * PWM Defines
 * --------------------------------------------------------------------- */
#define PWM_15KHZ_PERIOD 1600
#define PWM_PCT_TO_COMPARE(x) trunc((float) x * (PWM_15KHZ_PERIOD/100))
#define PWM_IDLE 50.0

/* TI INA219 Zero-Drift, Bidirectional Current/Power Monitor With I2C Interface */
#define INA219_I2C_ADDR                        (0x40)
#define INA219_CAL_VALUE                       (8192)

/* --------------------------------------------------------------------- 
 * PID Defines
 * --------------------------------------------------------------------- */
uint32 lastTime;
float Output;
float ITerm, lastPosition;
float refKp, refKi, refKd;
float kp, ki, kd;
uint32 refSampleTime = 5; // Default to 5ms
float outMin, outMax;
bool inAuto = false;
 
#define PID_MANUAL 0
#define PID_AUTOMATIC 1

/* --------------------------------------------------------------------- 
 * INA219 REGISTERS
 * --------------------------------------------------------------------- */
#define INA219_REG_CONFIG                      (0x00)
#define INA219_REG_SHUNTVOLTAGE                (0x01)
#define INA219_REG_BUSVOLTAGE                  (0x02)
#define INA219_REG_POWER                       (0x03)
#define INA219_REG_CURRENT                     (0x04)
#define INA219_REG_CALIBRATION                 (0x05)

/* ****************************
 * INA219 CONFIG REGISTER (R/W)
 * **************************** */
#define INA219_CONFIG_RESET                    (0x8000)  // Reset Bit

#define INA219_CONFIG_BVOLTAGERANGE_MASK       (0x2000)  // Bus Voltage Range Mask
#define INA219_CONFIG_BVOLTAGERANGE_16V        (0x0000)  // 0-16V Range
#define INA219_CONFIG_BVOLTAGERANGE_32V        (0x2000)  // 0-32V Range

#define INA219_CONFIG_GAIN_MASK                (0x1800)  // Gain Mask
#define INA219_CONFIG_GAIN_1_40MV              (0x0000)  // Gain 1, 40mV Range
#define INA219_CONFIG_GAIN_2_80MV              (0x0800)  // Gain 2, 80mV Range
#define INA219_CONFIG_GAIN_4_160MV             (0x1000)  // Gain 4, 160mV Range
#define INA219_CONFIG_GAIN_8_320MV             (0x1800)  // Gain 8, 320mV Range

#define INA219_CONFIG_BADCRES_MASK             (0x0780)  // Bus ADC Resolution Mask
#define INA219_CONFIG_BADCRES_9BIT             (0x0080)  // 9-bit bus res = 0..511
#define INA219_CONFIG_BADCRES_10BIT            (0x0100)  // 10-bit bus res = 0..1023
#define INA219_CONFIG_BADCRES_11BIT            (0x0200)  // 11-bit bus res = 0..2047
#define INA219_CONFIG_BADCRES_12BIT            (0x0400)  // 12-bit bus res = 0..4097

#define INA219_CONFIG_SADCRES_MASK             (0x0078)  // Shunt ADC Resolution and Averaging Mask
#define INA219_CONFIG_SADCRES_9BIT_1S_84US     (0x0000)  // 1 x 9-bit shunt sample
#define INA219_CONFIG_SADCRES_10BIT_1S_148US   (0x0008)  // 1 x 10-bit shunt sample
#define INA219_CONFIG_SADCRES_11BIT_1S_276US   (0x0010)  // 1 x 11-bit shunt sample
#define INA219_CONFIG_SADCRES_12BIT_1S_532US   (0x0018)  // 1 x 12-bit shunt sample
#define INA219_CONFIG_SADCRES_12BIT_2S_1060US  (0x0048)	 // 2 x 12-bit shunt samples averaged together
#define INA219_CONFIG_SADCRES_12BIT_4S_2130US  (0x0050)  // 4 x 12-bit shunt samples averaged together
#define INA219_CONFIG_SADCRES_12BIT_8S_4260US  (0x0058)  // 8 x 12-bit shunt samples averaged together
#define INA219_CONFIG_SADCRES_12BIT_16S_8510US (0x0060)  // 16 x 12-bit shunt samples averaged together
#define INA219_CONFIG_SADCRES_12BIT_32S_17MS   (0x0068)  // 32 x 12-bit shunt samples averaged together
#define INA219_CONFIG_SADCRES_12BIT_64S_34MS   (0x0070)  // 64 x 12-bit shunt samples averaged together
#define INA219_CONFIG_SADCRES_12BIT_128S_69MS  (0x0078)  // 128 x 12-bit shunt samples averaged together

#define INA219_CONFIG_MODE_MASK                (0x0007)  // Operating Mode Mask
#define INA219_CONFIG_MODE_POWERDOWN           (0x0000)
#define INA219_CONFIG_MODE_SVOLT_TRIGGERED     (0x0001)
#define INA219_CONFIG_MODE_BVOLT_TRIGGERED     (0x0002)
#define INA219_CONFIG_MODE_SANDBVOLT_TRIGGERED (0x0003)
#define INA219_CONFIG_MODE_ADCOFF              (0x0004)
#define INA219_CONFIG_MODE_SVOLT_CONTINUOUS    (0x0005)
#define INA219_CONFIG_MODE_BVOLT_CONTINUOUS    (0x0006)
#define INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS (0x0007)	


/* --------------------------------------------------------------------- 
 * RTOS INTERFACES
 * --------------------------------------------------------------------- */
/* Define macros for delaying a task by an amount, returning control back to the OS.
   We have to sleep in OS ticks, so compute how many that is and sleep that long.
   Minimum sleep time is 1ms. */
#define Sleep(MSToSleep)    vTaskDelay( ( MSToSleep * 1000 ) / configTICK_RATE_HZ )


/* --------------------------------------------------------------------- 
 * GLOBALS
 * --------------------------------------------------------------------- */
volatile uint16 Current;
volatile float PWM;
uint8 CurrentI2Cinbuf[20];

/* Task stack information */
volatile UBaseType_t uxHighWaterMark_PID;
volatile UBaseType_t uxHighWaterMark_Current;
volatile UBaseType_t uxHighWaterMark_Comm;

/* Message buffer lock */
SemaphoreHandle_t Lock;

/* --------------------------------------------------------------------- 
 * SPI MESSAGING
 * --------------------------------------------------------------------- */
#define COMM_SIGNATURE_BYTE 0xA5
#define COMM_SIGNATURE_WORD 0xA5A5A5A5

/* Inbound message from BBB, 21 bytes long */
typedef struct {
    
    uint32 signature;  /* Comm signature for reliable messaging */ 
    uint32 setpoint;   /* Setpoint (desired actuator position, 24 bits), high byte used as PID enable */
    float Kp;          /* PID constants to be used in calculation */
    float Ki;
    float Kd;
    int8 jog;          /* Jog value, to manually move the motor; valid range 0 to 100 */    
    //TODO uint16 checksum;
    
} __attribute__ ((__packed__)) rxMessage_t;

/* Wrap the message with an array of bytes */
union {
    rxMessage_t msg;
    uint8 buf[sizeof(rxMessage_t)];
} rxMessage;

/* This is not a formal RTOS locking mechanism, because you cannot take locks inside an 
   interrupt handler.  However, we can set a flag to indicate the rxMessage is at risk
   of changing and make sure the PID task does not act on changing data. */
bool rxMessageLocked;

/* Message back to the BBB, watch out for alignment here by packing the structure */
typedef struct  {
    
    uint32 signature; /* Comm signature for reliable messaging */
    uint16 pad1;      /* Pad byte for backwards compatibility during testing */
    uint16 current;   /* Motor current consumption (mA) */
    uint32 position;  /* Actual actuator position */ 
    float pwm;        /* PWM value to drive the motor at, derived from onboard PID */     
    //TODO uint16 checksum;  /* Message checksum */
    
} __attribute__ ((__packed__)) txMessage_t ;

/* Wrap the message with an array of bytes */
union {    
    txMessage_t msg;
    rxMessage_t dummy; /* Make sure this buffer is at LEAST as large as the rxMessage! */
    uint8 buf[sizeof(txMessage_t)];        
} txMessage;


/* State machine definition for transmit side of messaging */
typedef enum {
    
    txmsClear,    /* Transmit message buffer is empty and needs loading */
    txmsLoaded    /* Transmit message buffer has been loaded by the message thread for sending */
    
} txMessageStates_t;

txMessageStates_t txMessageState;


/*******************************************************************************
* Function Name: setupFreeRTOS
********************************************************************************
* Summary:
*  Hooks the tick and service handlers for the RTOS at runtime.
*
* Parameters: None
* Return: None
*******************************************************************************/

extern void xPortPendSVHandler(void);
extern void xPortSysTickHandler(void);
extern void vPortSVCHandler(void);

void setupFreeRTOS(void) {
#define CORTEX_INTERRUPT_BASE (16)
    
    /* Handler for Cortex Supervisor Call (SVC, formerly SWI) - address 11 */
    CyIntSetSysVector( CORTEX_INTERRUPT_BASE + SVCall_IRQn, (cyisraddress)vPortSVCHandler );
    
    /* Handler for Cortex PendSV Call - address 14 */
    CyIntSetSysVector( CORTEX_INTERRUPT_BASE + PendSV_IRQn, (cyisraddress)xPortPendSVHandler );    
    
    /* Handler for Cortex SYSTICK - address 15 */
    CyIntSetSysVector( CORTEX_INTERRUPT_BASE + SysTick_IRQn, (cyisraddress)xPortSysTickHandler );
}


/*******************************************************************************
* Function Name: Current_Read_Task
********************************************************************************
* Summary:
*  RTOS task to read the motor current consumption.
*
* Parameters: None
* Return: None
*******************************************************************************/
void Current_Read_Task(void *arg) {
    
//#ifdef zero
    volatile uint32 err;
    uint8 byteMSB;
    uint8 byteLSB;
//#endif

    volatile uint16 CurrentTemp;
    uint8 foo = 0;
    
    /* Initial high water mark reading */
    uxHighWaterMark_Current = uxTaskGetStackHighWaterMark( NULL );
    
    //TODO Init_INA(INA219_I2C_ADDR);
    
    while (1) {
        LED_Write(foo);
        PROBE_Write(foo);

        if (foo) { foo = 0; } else { foo = 1; }
        
        //TODO CurrentTemp = getCurrent_raw(INA219_I2C_ADDR);
        
        
//#ifdef zero        
        err = I2C_I2CMasterSendStart(INA219_I2C_ADDR, 0x00, 10); // send start condition for the INA219
        if (err == I2C_I2C_MSTR_NO_ERROR) {
            
            I2C_I2CMasterWriteByte(INA219_REG_CALIBRATION, 10);
            
            byteMSB = (uint8)((INA219_CAL_VALUE & 0xFF00) >> 8);
            I2C_I2CMasterWriteByte(byteMSB, 10);
            
            byteLSB = (uint8)(INA219_CAL_VALUE & 0x00FF);
            I2C_I2CMasterWriteByte(byteLSB, 10);
            I2C_I2CMasterSendStop(10);
        }
        
        err = I2C_I2CMasterSendStart(INA219_I2C_ADDR, 0x00, 10); // send start condition for the INA219
        if (err == I2C_I2C_MSTR_NO_ERROR) {
            
            I2C_I2CMasterWriteByte(INA219_REG_CURRENT, 10);
            I2C_I2CMasterSendStop(10);
            
            /* Read back the value for the current usage */
            err = I2C_I2CMasterReadBuf(INA219_I2C_ADDR, CurrentI2Cinbuf, 5, 1);
            
            /* Reassemble the current value into a 16 bit value */
            if (err == I2C_I2C_MSTR_NO_ERROR) {
                CurrentTemp = (uint16)(CurrentI2Cinbuf[0] << 8) + CurrentI2Cinbuf[1];
            } else {
                CurrentTemp = 0;     
            }
        } else {
            CurrentTemp = 0;   
        }
//#endif


        /* Perform the assignment as one operation, to make it as atomic as possible (may need more work here) */
        Current = CurrentTemp;

        /* Read the current at 2Hz */        
        Sleep(500);
        
        /* Get our task stack usage high water mark */    
        uxHighWaterMark_Current = uxTaskGetStackHighWaterMark( NULL );
    }
   
}

/*******************************************************************************
* Function Name: Comm_Task
********************************************************************************
* Summary:
*  RTOS task to perform the SPI communications.
*
* Parameters: None
* Return: None
*******************************************************************************/
void Comm_Task(void *arg) {
 
    /* Initial high water mark reading */
    uxHighWaterMark_Comm = uxTaskGetStackHighWaterMark( NULL );  
    
    while (1) {
        
        /* If the SPI is moving data out right now, skip any touching of the message buffer */
        if (!SPI_1_SpiIsBusBusy()) {
        
            /* In certain states, this thread is responsible for loading the outbound messaging */
            switch (txMessageState) {
             
                /* Buffer is clear and ready for loading */
                case txmsClear:

                    /* Take the lock, set the fields, and release the lock.  If we can't get the lock in 4ms, 
                       we have missed this messaging cycle and just skip it */
                    if( xSemaphoreTake( Lock, ( TickType_t ) 4 ) == pdTRUE ) {
                    
                        /* Set fields individually */
                        txMessage.msg.signature = COMM_SIGNATURE_WORD;
                        txMessage.msg.position = Counter_1_ReadCounter();
                        txMessage.msg.pwm = PWM;
                        txMessage.msg.current = Current;            

                        /* Copy the readied buffer out to the FIFO */
                        SPI_1_SpiUartPutArray(txMessage.buf, sizeof(txMessage.buf)); 

                        /* Indicate it's loaded for use */
                        txMessageState = txmsLoaded;
                    
                        /* Release the lock */
                        xSemaphoreGive( Lock );
                    }            
                
                    break;
            
                /* A message was already readied for transmission, nothing to do here */
                case txmsLoaded:
                    
                    break;
            }
            
        }
        
        /* Quick sleep, a whole messaging sequence will take maybe 1.5ms */
        Sleep(1);
        
        /* Get our task stack usage high water mark */    
        uxHighWaterMark_Comm = uxTaskGetStackHighWaterMark( NULL );
    }
}
    



/*******************************************************************************
* Function Name: PWM_Set
********************************************************************************
* Summary:
*  Sets the duty cycle of the PWM at the output pin.
*
* Parameters: Duty cycle, in percent.
* Return: None
*******************************************************************************/
void PWM_Set(float dutycycle) {
    
    float drive = dutycycle;    
    
    /* Clip to the max/min of the drive */
    if (drive > 100) {
        drive = 100;
    } else if (drive < 0) {
        drive = 0;
    }
    
    PWM_1_WriteCompare(PWM_PCT_TO_COMPARE(drive));    
}


/*******************************************************************************
* Function Name: PID_Initialize
********************************************************************************
* Summary:
*  Setup and reset the PID terms.
*
* Parameters: None
* Return: None
*******************************************************************************/
void PID_Initialize(void) {
    
    uint32 position;
    
    /* Get up-to-date current position */
    position = Counter_1_ReadCounter();     

    lastPosition = position;
    ITerm = Output;
    
    if (ITerm > outMax) {
        ITerm = outMax;
    } else if (ITerm < outMin) {
        ITerm = outMin;
    }
}    


/*******************************************************************************
* Function Name: PID_SetTunings
********************************************************************************
* Summary:
*  Setup the p, i, and d gain values and scale to the sample time.
*
* Parameters: Sample time in ms, and Kp, Ki, Kd gains.
* Return: None
*******************************************************************************/
void PID_SetTunings(uint32 newSampleTime, float newKp, float newKi, float newKd) {
    
    if (newSampleTime > 0) {
    
        float sampleTimeInSec = ((float) newSampleTime) / 1000;
    
        kp = newKp;
        ki = newKi * sampleTimeInSec;
        kd = newKd / sampleTimeInSec;        
    }
}


/*******************************************************************************
* Function Name: PID_Compute
********************************************************************************
* Summary:
*  Execute a pass through the PID process to create a duty cycle output.
*
* Parameters: Current time and current destination.
* Return: PWM output, in percentage.
*******************************************************************************/
float PID_Compute(uint32 now, uint32 setpoint) {
    
    int32 error, dPosition;
    uint32 timeChange;
    uint32 position;
        
    if(!inAuto) 
        return 0;
    
    /* Get most up-to-date current position */
    position = Counter_1_ReadCounter();     

    /* How much time has elapsed since the last pass? */
    timeChange = (now - lastTime);
    
    /* Only do the PID calc if at LEAST 5ms has elapsed */
    if (timeChange >= refSampleTime) {

        /* Adjust the gains to the most recent sampling time.  Do it continuously to make sure the gains are amplified 
           in case the cycle runs longer than the normal 5ms. */
        PID_SetTunings(timeChange, refKp, refKi, refKd);
        
        /* Compute all the working error variables */
        error = setpoint - position;
        ITerm += (ki * error);
        
        /* Clip the I term at the output max (windup guard) */
        if (ITerm > outMax) {
            ITerm = outMax;
        } else if (ITerm < outMin) {
            ITerm = outMin;
        }
        
        /* Calculate the error term */
        dPosition = (position - lastPosition);

        /* Compute PID Output */
        Output = (kp * error) + ITerm - (kd * dPosition);
        
        /* Clip the output */
        if (Output> outMax) {
            Output = outMax;
        } else if (Output < outMin) {
            Output = outMin;
        }

        /* Remember some variables for next time */
        lastPosition = position;
        lastTime = now;        
    }
    
    return Output;    
}
 


 
/*******************************************************************************
* Function Name: PID_SetOutputLimits
********************************************************************************
* Summary:
*  Define the output limits of the PID process.
*
* Parameters: Min and Max output values.
* Return: None
*******************************************************************************/
void PID_SetOutputLimits(float Min, float Max) {
    
    if(Min > Max) 
        return;

    /* Update the global min/max */
    outMin = Min;
    outMax = Max;

    /* Clip the output to the new max */
    if(Output > outMax) {
        Output = outMax;
    } else if (Output < outMin) {
        Output = outMin;
    }

    /* Clip the I term to the new max */
    if (ITerm > outMax) {
        ITerm = outMax;
    } else if (ITerm < outMin) {
        ITerm = outMin;
    }
}
 
/*******************************************************************************
* Function Name: PID_SetMode
********************************************************************************
* Summary:
*  Sets the PID into automatic or manual mode.
*
* Parameters: Mode, either PID_AUTOMATIC or PID_MANUAL.
* Return: None
*******************************************************************************/
void PID_SetMode(uint32 Mode) {
    
    bool newAuto = (Mode == PID_AUTOMATIC);
    
    if (newAuto && !inAuto) {  
        /*we just went from manual to auto*/
        PID_Initialize();
    }
    
    inAuto = newAuto;
} 
    
/*******************************************************************************
* Function Name: PID_SetDrive
********************************************************************************
* Summary:
*  Convert the output of PID into a duty cycle for use on the PWM.
*
* Parameters: Percentage output to drive the PWM.
* Return: None
*******************************************************************************/
void PID_SetDrive(float percent) {
    
    /* Valid percentage range coming out of the PID algorithm is -100.0 to +100.0 
       which needs to be translated into a duty cycle value of 0.0 to 100.0 */
    float dutycycle = (percent + 100) / 2;    
    
    /* The duty cycle can now be written to the PWM device itself */
    PWM_Set(dutycycle);  
}




/*******************************************************************************
* Function Name: PID_Task
********************************************************************************
* Summary:
*  RTOS task to perform the PID calculations.
*
* Parameters: None
* Return: None
*******************************************************************************/
void PID_Task(void *arg) {
    
    /* Sleep this thread 5ms at a time */
    uint32 sleeptime = 5;    
    uint32 now;
    uint32 desired;
    bool enabled, was_enabled;
    float percent; 
    
    /* Initial high water mark reading */
    uxHighWaterMark_PID = uxTaskGetStackHighWaterMark( NULL );
    
    /* Initial values */

    /* These are the 'reference' values passed down from the server, not to be used in-the-raw because the actual gain 
       values are time interval adjusted */
    refKp = 0;
    refKi = 0;
    refKd = 0;
    
    /* Setup the PID subsystem */
    PID_Initialize();
    PID_SetTunings(sleeptime, refKp, refKi, refKd);
    PID_SetOutputLimits(-100.0, 100.0);
    PID_SetMode(PID_MANUAL);

    now = 0;
    desired = 0;  
    
    /* Start off disabled */
    was_enabled = false;
    enabled = false;
    
    while (1) {

        /* If the interrupt handler for the end of SPI transaction is NOT moving data into the rxMessageBuffer 
           right now, go ahead and use the data that's in there.  If it's getting updated we can wait until next
           cycle, or longer, to use it since the values shouldn't change all that often or all that quickly. */
        if (!rxMessageLocked) {
            
            /* If the rxMessage is not corrupt, use it to update PID gains and setpoint */
            if (rxMessage.msg.signature == COMM_SIGNATURE_WORD) {
            
                /* If the PID constants are different, update the reference copies. */
                if ((refKp != rxMessage.msg.Kp) || (refKi != rxMessage.msg.Ki) || (refKd != rxMessage.msg.Kd)) {
                
                    /* Update the 'reference' values passed down from the server, not to be used in-the-raw because the actual gain 
                       values are time interval adjusted */
                    refKp = rxMessage.msg.Kp;
                    refKi = rxMessage.msg.Ki;
                    refKd = rxMessage.msg.Kd;                                        
                }
                   
                /* The enable flag and desired position are currently stacked together in one uint32 */
                enabled = ((rxMessage.msg.setpoint & 0xFF000000) > 0);
                desired = (rxMessage.msg.setpoint & 0x00FFFFFF);
                
                /* If the server is asking us to jog, do that instead of PID */
                if (!enabled) {                    
                    PWM_Set(rxMessage.msg.jog + 50);
                }
            }
            
            /* Handle mode switching */
            if (!was_enabled && enabled) {
                PID_SetMode(PID_AUTOMATIC);
            } else if (!enabled && was_enabled) {
                PID_SetMode(PID_MANUAL);                
            } else {
                // No mode change happened   
            }
            
        }            
            
        /* Run the PID algorithm */
        now = xTaskGetTickCount();
        percent = PID_Compute(now, desired);
        
        /* Send the pwm back up to the BBB */
        if (enabled) {
            
            /* Use the global PWM value to communicate back the percentage of drive to the server */
            PWM = percent;
            
            /* Put the PID drive percentage out on the wire */
            PID_SetDrive(percent);
            
        } else {
            /* If disabled, just return 0 */
            PWM = 0;
        }            
        
        /* Run the loop every 5ms, which is 200Hz update rate */
        Sleep(sleeptime);

        /* Get our task stack usage high water mark */    
        uxHighWaterMark_PID = uxTaskGetStackHighWaterMark( NULL );        
    }
   
}



/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
*  Setup tasks, interrupts, and get the RTOS running.
*
* Parameters: None
* Return: NEVER!
*******************************************************************************/
int main(void) {
    
    uint8 s;
    setupFreeRTOS();
        
    /* Create LED task, which will control the intensity of the LEDs */
    xTaskCreate(
        PID_Task,       /* Task function */
        "PID",          /* Task name (string) */
        64,             /* Task stack, allocated from heap (measured 8/10 to be 32 bytes) */
        0,              /* No param passed to task function */
        2,              /* Medium priority */
        0 );            /* Not using the task handle */    
    
    xTaskCreate(
        Comm_Task,       /* Task function */
        "Communications", /* Task name (string) */
        100,            /* Task stack, allocated from heap (measured 8/10 to be 76 bytes)  */
        0,              /* No param passed to task function */
        3,              /* High priority */
        0 );            /* Not using the task handle */    

    xTaskCreate(
        Current_Read_Task, /* Task function */
        "Read Current", /* Task name (string) */
        64,             /* Task stack, allocated from heap (measured 8/10 to be 30 bytes) */
        0,              /* No param passed to task function */
        1,              /* Low priority */
        0 );            /* Not using the task handle */    
    
    /********************************************************************** 
    * Message buffer mutex
    **********************************************************************/
    Lock = xSemaphoreCreateMutex();

    /* If we can't create the lock, flash the light and hold here */
    if( Lock == NULL ) {
        s = 1;
        
        while(1) {
            s = !s;
            LED_Write(s); 
            CyDelay(1000u);                    
        }
    }
    
    
    /********************************************************************** 
    * Interrupts
    **********************************************************************/
    
    /* Sets up the Index and Reset interrupt and enables them */
    isr_home_StartEx(HomeIsrHandler);
    isr_rst_encoder_StartEx(RSTIsrHandler);
    
    /* Setup the SPI slave select interrupt ISR */
    isr_spi_ss_StartEx(SPI_SS_IsrHandler);
    
    /* Changes initial priority for the interrupts */
    isr_home_SetPriority(DEFAULT_PRIORITY);
    isr_rst_encoder_SetPriority(HIGHER_PRIORITY);
    isr_spi_ss_SetPriority(DEFAULT_PRIORITY);

    /* Enable the global interrupt */
    CyGlobalIntEnable;
    
    /*********************************************************************** 
    * Start the various subsystems.
    ***********************************************************************/
    I2C_Start();
    CyDelay(100u);
    SPI_1_Start();
    
    /* Setup the PWM at a base frequency of 15KHz, 50% duty cycle.  Clock_1 is set to
       12MHz, so the desired period to get 15KHz is a count of 800. */
    PWM_1_Start();
    PWM_1_WritePeriod(PWM_15KHZ_PERIOD);
    PWM_Set(PWM_IDLE);   
   
    /* Start counting the quadrature encoding */
    Counter_1_Start();    
    Counter_1_WriteCounter(0x00800000);  // Set the encoder initially to mid range
 
    /* Set some initial values */
    rxMessageLocked = false;
    
    /***********************************************************************
    * Start the RTOS task scheduler
    ***********************************************************************/
    vTaskStartScheduler();
   
    /***********************************************************************
    *  We should never reach this
    ***********************************************************************/
    return 1;       
}


/*******************************************************************************
* Function Name: RSTIsrHandler
********************************************************************************
* Summary:
*  The interrupt handler for resetting the encoder count interrupts.
*  Clears a pending Interrupt.
*  Clears a pin Interrupt.
*
* Parameters: None
* Return: None
*******************************************************************************/
CY_ISR(RSTIsrHandler) {
    
    /* Clear pending Interrupt */
    isr_rst_encoder_ClearPending(); 
    
    /* Clear pin Interrupt */
    Reset_Encoder_ClearInterrupt();
    
    /* Clear the 24b Encoder (Absolute Position Counter) */
    Counter_1_WriteCounter(0);
}

/*******************************************************************************
* Function Name: HomeIsrHandler
********************************************************************************
* Summary:
*  The interrupt handler for resetting the encoder count interrupts.
*  Clears a pending Interrupt.
*  Clears a pin Interrupt.
*
* Parameters: None
* Return: None
*******************************************************************************/
CY_ISR(HomeIsrHandler) {
    
    /* Clear pending Interrupt */
    isr_home_ClearPending(); 
    
    /* Clear pin Interrupt */
    HOME_IN_ClearInterrupt();

    /* Clear the 24b Encoder (Absolute Position Counter) */
    Counter_1_WriteCounter(0);
}


/*******************************************************************************
* Function Name: SPI_SS_IsrHandler
********************************************************************************
* Summary:
*  The interrupt handler for knowing when the SPI master deasserts the slave select.
*  Clears a pin Interrupt.
*
* Parameters: None
* Return: None
*******************************************************************************/
CY_ISR(SPI_SS_IsrHandler) {
    
    uint32 i;
    bool txclear;
    
    /* Clear SPI slave select pin Interrupt */
    spi_ss_ClearInterrupt();
    
    /* Make sure the slave select is actually de-asserted before proceeding */
    if (!spi_ss_Read()) 
        return;
       
    /* Determine if the TX buffer is clear */
    txclear = (SPI_1_SpiUartGetTxBufferSize() == 0);        

    /* It's possible the slave select has fired and returned without transmitting any data (glitched) so
       check the messaging state before resetting the buffers */
    switch (txMessageState) {
     
        /* FIFO is not yet loaded and slave select deasserted, this is a glitch */ 
        case txmsClear:
            break;
     
        /* A message was readied for transmission */
        case txmsLoaded:
        
            if (txclear) {
             
                /* FIFO is clear and slave select deasserted, it's time to reset */ 
                
                /* Clear the message buffer, the comm thread will fill it in again to make sure a fresh message is heading out */
                bzero(txMessage.buf, sizeof(txMessage.buf)); 

                /* Message from the master is completely clocked in by now */     
                rxMessageLocked = true;
                for (i = 0; i < sizeof(rxMessage.buf); i++) {
                    rxMessage.buf[i] = (uint8) SPI_1_SpiUartReadRxData();            
                }
                rxMessageLocked = false;
                
                /* Clear out all the bytes in the receive side so they can be filled in again */
                SPI_1_SpiUartClearRxBuffer();   
                
                /* Set the state to indicate to the messaging thread that it's time to load the TX buffer again */
                txMessageState = txmsClear;
                
            } else {

                /* FIFO is NOT clear and slave select deasserted, this is bad because the master didn't clock all the bytes! */ 

                /* Clear the partial message out */
                SPI_1_SpiUartClearTxBuffer();
                
                /* Clear the message buffer, the comm thread will fill it in again to make sure a fresh message is heading out */
                bzero(txMessage.buf, sizeof(txMessage.buf)); 

                /* Invalidate the rx message */
                rxMessage.msg.signature = 0;
                
                /* Clear out all the bytes in the receive side so they can be filled in again */
                SPI_1_SpiUartClearRxBuffer();   
                
                /* Set the state to indicate to the messaging thread that it's time to load the TX buffer again */
                txMessageState = txmsClear;                
            }
        
            break;
            
    } // End of switch block

}


