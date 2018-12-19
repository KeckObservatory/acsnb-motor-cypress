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
* 12/18/18 PMR  Rev: B  Implement checksummed messaging and max PWM limiting
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

/* Debugging - undefine this for a production system that needs to watchdog */
#define DEBUG_PROBE_ATTACHED 1

/* --------------------------------------------------------------------- 
 * CONSTANTS
 * --------------------------------------------------------------------- */

/* Interrupt priorities (not to be confused with RTOS task priorities) */
#define NESTED_ISR                             (1u)
#define DEFAULT_PRIORITY                       (3u)
#define HIGHER_PRIORITY                        (2u)

/* Interrupt prototypes */
CY_ISR_PROTO(HomeIsrHandler);
CY_ISR_PROTO(RSTIsrHandler);
CY_ISR_PROTO(SPI_IsrHandler);
CY_ISR_PROTO(SPI_SS_IsrHandler);

/* --------------------------------------------------------------------- 
 * WDT Defines
 *
 * ILO clock is 32KHz (approx. 31ms/count)
 * We will allow the CPU to stall for 2 full seconds before forcibly
 * resetting.  That comes to 64,000 counts for that 2 seconds.
 * --------------------------------------------------------------------- */
#define WDT_COUNT1_REFRESH()                   CySysWdtResetCounters(CY_SYS_WDT_COUNTER1_RESET)
#define WDT_COUNT1_MATCH_RESET                 (0xFA00u)

/* --------------------------------------------------------------------- 
 * PWM Defines
 * --------------------------------------------------------------------- */
#define PWM_15KHZ_PERIOD 1600
#define PWM_PCT_TO_COMPARE(x) trunc((float) x * (PWM_15KHZ_PERIOD/100))
#define PWM_IDLE 50.0

/* TI INA219 Zero-Drift, Bidirectional Current/Power Monitor With I2C Interface */
#define INA219_I2C_ADDR                        (0x40)
#define INA219_CAL_VALUE                       (8192)

int8 PWM_Jog;
int8 PWM_Max_Fwd, PWM_Max_Rev;

/* Neutral PWM jog is a 50% duty cycle */
#define PWM_JOG_NEUTRAL                        (50)

/* PWM maximum current value clipped to +/- X% duty cycle around the center (50 by default, full power possible) */
#define PWM_MAX_MAGNITUDE                      (50) 

/* --------------------------------------------------------------------- 
 * PID Defines
 * --------------------------------------------------------------------- */
bool PID_Enabled, PID_Was_Enabled;
uint32 PID_Setpoint;

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
/* Coarse reporting of state back to the node box software */
typedef enum {    
    csUNDEFINED   = 0,  /* Invalid state */
    csUnconfig    = 1,  /* Unconfigured, freshly rebooted */
    csReady       = 2,  /* Configuration complete and ready for commands */
    csMAX    
} ConfigStates_t;

#define configStateValid(s) ( ((ConfigStates_t) s > csUNDEFINED) && ((ConfigStates_t) s < csMAX) )

volatile ConfigStates_t ConfigState;
volatile uint8 ConfigSequence;
volatile uint16 ChecksumErrors;
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
/* V2 protocol 

   1) Messages have opcodes (configuration, position, get status)
   2) Messages are validated by a checksum instead of an arbitrary signature pattern.
   3) Transfer the max message size every time, regardless of all bytes used or not.

*/
    
/* Set this to match the size of the status response message, 18 bytes */    
#define MAX_MESSAGE_SIZE 30
    
/* Remember the last time a message came in so we can timeout moves if the node box stops 
   talking.  5000 ticks is 1 second of not talking. */
#define MAX_LAST_MESSAGE_TIME_TICKS 1000
uint32 LastMessageTimeTick;

/* Opcodes that can come from the node box software */
typedef enum {
    opUNDEFINED = 0,
    opConfig    = 1,
    opStatus    = 2,
    opError     = 3,
    opMAX       
} rxMessage_opcodes_t;    

/* Sanity check for opcodes */
#define rxMessageOpcodeValid(op) ( ((rxMessage_opcodes_t) op > opUNDEFINED) && ((rxMessage_opcodes_t) op < opMAX) )

typedef struct { 
    uint8 checksum;        
    uint8 size;       /* Size of the message bytes, including opcode and size and checksum */
    uint8 opcode;     /* Operation (generic overlay for previewing opcode value) */
} __attribute__ ((__packed__)) rxMessage_overlay_t;

/* Configuration message, 17 bytes */
typedef struct {
    uint8 checksum;        
    uint8 size;       /* Size of the message bytes, including opcode and size and checksum */
    uint8 opcode;     /* Operation: 01 == config */        
    uint8 sequence;   /* Configuration message sequence number */
    float Kp;         /* PID constants to be used in calculation */
    float Ki;
    float Kd;
    uint8 limit;      /* PWM duty cycle (speed) limit, +/- percent */
} __attribute__ ((__packed__)) rxMessage_config_t;

/* Status message, contains desired position and such values, 12 bytes */
typedef struct {
    uint8  checksum;        
    uint8  size;       /* Size of the message bytes, including opcode and size and checksum */
    uint8  opcode;     /* Operation: 02 == status */
    uint8  enable;     /* Enable/disable PID algorithm */
    uint32 setpoint;   /* Setpoint (desired actuator position) */
    int8   jog;        /* Jog value, to manually move the motor; valid range -100 to 100 */    
} __attribute__ ((__packed__)) rxMessage_status_t;
   


/* Wrap the message with an array of bytes */
union {
    uint8               buf[MAX_MESSAGE_SIZE];
    rxMessage_overlay_t overlay;
    rxMessage_config_t  config;
    rxMessage_status_t  status;  
} rxMessage;

/* Message back to the BBB, watch out for alignment here by packing the structure (should be 18 bytes) */
typedef struct  {  
    uint8  checksum;   /* Message checksum */    
    uint8  size;       /* Size of the message bytes, including opcode and size and checksum */
    uint8  opcode;     /* Echo back of the opcode this response is for, operation: 03 == status */
    uint8  state;      /* Enum value for current device configuration state */ 
    uint8  sequence;   /* Echo back the config sequence number currently set */
    uint16 checksum_errors; /* Count of checksum errors */
    uint16 current;    /* Motor current consumption (mA) */
    uint32 position;   /* Actual actuator position */ 
    float pwm;         /* PWM value the motor is moving at */     
} __attribute__ ((__packed__)) txMessage_t;

/* Wrap the message with an array of bytes */
union {    
    uint8       buf[MAX_MESSAGE_SIZE];
    txMessage_t msg;
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
 
    rxMessage_opcodes_t opcode;
    uint8 size;
    uint8 i;
    uint8 checksum;
    
    /* Initial high water mark reading */
    uxHighWaterMark_Comm = uxTaskGetStackHighWaterMark( NULL );  
    
    while (1) {
        
        /* If the SPI is moving data out right now, skip any touching of the message buffer */
        if (!SPI_1_SpiIsBusBusy()) {
        
            /* In certain states, this thread is responsible for loading the outbound messaging */
            switch (txMessageState) {
             
                /* Output buffer is clear and ready for loading, rxMessage is (probably) good and needs processing */
                case txmsClear:

                    /* Take the lock, set the fields, and release the lock.  If we can't get the lock in 4ms, 
                       we have missed this messaging cycle have to skip it.
                       2018-12-18 PMR: At the moment, this is the only task taking the lock so it will always succeed.
                                       Sometime in the future we might need the lock so I am leaving it in.
                    */
                    if( xSemaphoreTake( Lock, ( TickType_t ) 4 ) == pdTRUE ) {
                        
                        /* Get a few items out of the message before checking the sum */
                        size     = rxMessage.overlay.size;
                        opcode   = (rxMessage_opcodes_t) rxMessage.overlay.opcode;
                        
                        /* Make sure the size makes sense.  If we have to reset the size it's probably a corrupt message anyway. */
                        if (size > sizeof(txMessage.buf))
                            size = sizeof(txMessage.buf);                            
                    
                        /* Calculate the checksum by summing the bytes of the entire message, it should resolve to 0 if error-free */
                        for (i = 0, checksum = 0; i < size; i++)
                            checksum += rxMessage.buf[i]; 

                        /* Checksum fault, don't try to process the messgage */
                        if ((checksum & 0xFF) != 0) {
                            
                            txMessage.msg.opcode = opcode;
                            txMessage.msg.size = sizeof(txMessage_t);
                            ChecksumErrors++;
                            
                        /* Message looks fine, process it */
                        } else {

                            /* Message opcode must be valid before trying to process the message contents */
                            if ( rxMessageOpcodeValid(opcode) ) {                            
                            
                                switch (opcode) {
                                
                                    case opConfig:
                                        /* Special message to establish settings on the device such as PID gains */                                    
                                    
                                        /* Update the 'reference' values passed down from the server, not to be used in-the-raw because the actual gain 
                                           values are time interval adjusted */
                                        refKp = rxMessage.config.Kp;
                                        refKi = rxMessage.config.Ki;
                                        refKd = rxMessage.config.Kd;                                        
                                           
                                        /* Setup the PWM limits and stop a jog if one was in progress */
                                        PWM_Jog = PWM_JOG_NEUTRAL;
                                        PWM_Max_Fwd = PWM_JOG_NEUTRAL + rxMessage.config.limit;
                                        PWM_Max_Rev = PWM_JOG_NEUTRAL - rxMessage.config.limit;                                    
                                    
                                        /* We have received a config message, so signal to the PID thread that processing is allowed */
                                        ConfigState = csReady;  
                                        ConfigSequence = rxMessage.config.sequence;
                                        break;

                                    case opStatus:
                                        /* The normal message telling us where to go, how much to jog, enable on/off */
                                        PID_Enabled = (bool) rxMessage.status.enable;
                                        PID_Setpoint = rxMessage.status.setpoint;
                                    
                                        /* PWM jog value ranges from 0 to 100, where 0 is max-reverse current, 100 is max-forward, 50 is neutral/no movement */
                                        PWM_Jog = rxMessage.status.jog;                                    
                                        break;
                                    
                                    /* No other opcodes are valid */
                                    default:
                                        break;
                                }                           
                            }                            
                        }
                        
                        /* Fill out the common reponse the same way every time, as a status response */
                        txMessage.msg.checksum = 0;                            
                        txMessage.msg.opcode   = opStatus;
                        txMessage.msg.size     = sizeof(txMessage_t);
                        txMessage.msg.state    = (uint8) ConfigState;
                        txMessage.msg.checksum_errors = ChecksumErrors;
                        txMessage.msg.sequence = ConfigSequence;
                        txMessage.msg.position = Counter_1_ReadCounter();
                        txMessage.msg.pwm      = PWM;
                        txMessage.msg.current  = Current;                            
                        
                        /* Set the checksum in the response */
                        for (i = 0, checksum = 0; i < sizeof(txMessage_t); i++)
                            checksum += txMessage.buf[i]; 
                            
                        /* Take the 2's complement of the sum and put it back in the message */
                        txMessage.msg.checksum = ~checksum + 1;
                            
                        /* Copy the readied buffer out to the FIFO */
                        //TODO: should we clear this here, or at the end of the transmit complete interrupt?   SPI_1_SpiUartClearTxBuffer();
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

    /* Clip to the configured mix/max */
    if (drive > PWM_Max_Fwd) {
        drive = PWM_Max_Fwd;
    } else if (drive < PWM_Max_Rev) {
        drive = PWM_Max_Rev;
    }
    
    /* If somehow misconfigured to go even higher, clip to the max/min of the drive */
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
    
    /* Start off disabled */
    PID_Setpoint = 0;  
    PID_Was_Enabled = false;
    PID_Enabled = false;
    
    while (1) {
              
        /* ------------------------------------------------------------------------------------ */
        /* At the top of the PID loop, refresh the counter of the watchdog to indicate the RTOS 
           thread is still alive.  Were the RTOS to crash or the motion thread to die, the CPU 
           will be reset after 2 seconds. */
        WDT_COUNT1_REFRESH();        
        /* ------------------------------------------------------------------------------------ */

        
        /* If the server hasn't talked to us in a while (no messages on the SPI), 
           take preventative action and abandon any moves in progress.  When the uint32
           overflows, "now" will be (temporarily) less than the last timestamped message,
           so handle that too */
        now = xTaskGetTickCount();
        if ( (now > (LastMessageTimeTick + MAX_LAST_MESSAGE_TIME_TICKS)) ||
             (now < LastMessageTimeTick) ) {
            
            /* Stop all motion */
            PWM_Set(PWM_JOG_NEUTRAL);
            PID_Enabled = false;
            
            /* Clear the values that would drive motion on the next message arrival.  Assume the next message might be a config, 
               in which case we want to be neutral. */
            PWM_Jog = PWM_JOG_NEUTRAL;

        /* Only run the PID algorithm if we have been configured by the nodebox software */
        } else if (ConfigState == csReady) {
        
            /* If the server is asking us to jog, do that instead of PID */
            if (!PID_Enabled) {                    
                PWM_Set(PWM_JOG_NEUTRAL + PWM_Jog);
            }
            
            /* Handle mode switching */
            if (!PID_Was_Enabled && PID_Enabled) {
                PID_SetMode(PID_AUTOMATIC);
            } else if (!PID_Enabled && PID_Was_Enabled) {
                PID_SetMode(PID_MANUAL);                
            } else {
                // No mode change happened   
            }
            
            /* Save value for next cycle */
            PID_Was_Enabled = PID_Enabled;
            
                
            /* Run the PID algorithm */
            percent = PID_Compute(now, PID_Setpoint);
            
            /* Send the pwm back up to the BBB */
            if (PID_Enabled) {
                
                /* Use the global PWM value to communicate back the percentage of drive to the server */
                PWM = percent;
                
                /* Put the PID drive percentage out on the wire */
                PID_SetDrive(percent);
                
            } else {
                /* If disabled, just return 0 */
                PWM = 0;
            }
            
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
    
    
    
    /********************************************************************** 
    * Watchdog timer.  Implements the WDT1 automatic CPU reset.
    **********************************************************************/

    /* If you are using the JTAG debugging probe, turn off the watchdog by
       defining DEBUG_PROBE_ATTACHED to something, or it will reset the CPU
       when you hit a breakpoint. */

#ifndef DEBUG_PROBE_ATTACHED
	/* Set WDT counter 1 to generate reset on match */
	CySysWdtWriteMatch(CY_SYS_WDT_COUNTER1, WDT_COUNT1_MATCH_RESET);
	CySysWdtWriteMode(CY_SYS_WDT_COUNTER1, CY_SYS_WDT_MODE_RESET);
    CySysWdtWriteClearOnMatch(CY_SYS_WDT_COUNTER1, 1u);
	
	/* Enable WDT counter 1 */
	CySysWdtEnable(CY_SYS_WDT_COUNTER1_MASK);
	
	/* Lock WDT registers and try to disable WDT 1 */
	CySysWdtLock();
	CySysWdtDisable(CY_SYS_WDT_COUNTER1_MASK);
	CySysWdtUnlock();        
#endif    

    
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
    
    /* Default the jog value to neutral (no movement) */
    PWM_Jog = PWM_JOG_NEUTRAL;
    PWM_Max_Fwd = PWM_JOG_NEUTRAL + PWM_MAX_MAGNITUDE;
    PWM_Max_Rev = PWM_JOG_NEUTRAL - PWM_MAX_MAGNITUDE;
   
    /* Start counting the quadrature encoding */
    Counter_1_Start();    
    Counter_1_WriteCounter(0x00800000);  // Set the encoder initially to mid range
 
    /* Set some initial values */
    LastMessageTimeTick = xTaskGetTickCount();
    
    /* Start off unconfigured */
    ConfigState = csUnconfig;  
    ConfigSequence = 0;
    ChecksumErrors = 0;
   
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
    
    /* Clear SPI slave select pin Interrupt */
    spi_ss_ClearInterrupt();
    
    /* Make sure the slave select is actually de-asserted before proceeding */
    if (!spi_ss_Read()) 
        return;
   
    /* Update the last message tick timer */
    LastMessageTimeTick = xTaskGetTickCount();

    /* It's possible the slave select has fired and returned without transmitting any data (glitched) so
       check the messaging state before resetting the buffers */
    switch (txMessageState) {
     
        /* FIFO is not yet loaded and slave select deasserted, this is a glitch */ 
        case txmsClear:
            break;
     
        /* A message was readied for transmission */
        case txmsLoaded:
         
            /* FIFO is clear and slave select deasserted, it's time to reset */ 
            
            /* Clear the transmit message buffer, the comm thread will fill it in again to make sure a fresh message is heading out */
            bzero(txMessage.buf, sizeof(txMessage.buf)); 

            /* Message from the master is completely clocked in by now */     
            for (i = 0; i < sizeof(rxMessage.buf); i++) {
                rxMessage.buf[i] = (uint8) SPI_1_SpiUartReadRxData();            
            }
            
            /* Clear out all the bytes in the receive side so they can be filled in again */
            SPI_1_SpiUartClearRxBuffer();  
            
            /* Clear out any remaining bytes in the transmit buffer, in case the message transfer was cut short */
            SPI_1_SpiUartClearTxBuffer();
            
            /* Set the state to indicate to the messaging thread that it's time to load the TX buffer again */
            txMessageState = txmsClear;        
            break;
            
    } 
}


