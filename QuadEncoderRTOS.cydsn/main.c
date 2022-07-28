/*******************************************************************************
* File Name: main.c
* Author: Paul Richards, W.M. Keck Observatory
*   (Adapted from original version from Kona Scientific, JP Fumo)
*
* Description:
*  This file provides a SPI interface for the Quadrature Encoder for the ACS project.
*  The I2C device provides readback of motor current consumption.
*
* History:
* 07/27/22 PMR  Rev: 0-1-0 convert FreeRTOS to binary-rate-monotonic-scheduler (BRMS)
* 07/09/20 PMR  Rev: 0-0-7 implement zeroing the encoder value
* 07/09/19 PMR  Rev: 0-0-6 fix tuning of INA219 and inhibit encoder report during homing
* 05/09/19 PMR  Rev: 0-0-5 multiple shaper and PID fixes; let encoder go negative
* 03/22/19 PMR  Rev: 0-0-3 add PID separate I limit and simplify limiting code
* 02/07/19 PMR  Rev: 0-0-2 implement revision numbering in protocol
* 12/18/18 PMR  Rev: B  Implement checksummed messaging and max PWM limiting
* 10/11/18 PMR  Rev: A  Implement PWM functions for PDI control
* 07/31/18 PMR  Rev: NC Initial Release after port from Kona Scientific code
*******************************************************************************/
#include <I2C_I2C.h>
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>
#include "INA219.h"

/* Firmware revision as of 2022-07-27 PMR */
#define FIRMWARE_REV_0 0
#define FIRMWARE_REV_1 1
#define FIRMWARE_REV_2 0

/* Debugging - undefine this for a production system that needs to watchdog */
#define DEBUG_PROBE_ATTACHED 1

/* --------------------------------------------------------------------- 
 * CONSTANTS
 * --------------------------------------------------------------------- */

/* Interrupt priorities */
#define NESTED_ISR                             (1u)
#define HIGHER_PRIORITY                        (2u)
#define DEFAULT_PRIORITY                       (3u)

/* Interrupt prototypes */
CY_ISR_PROTO(HomeIsrHandler);
CY_ISR_PROTO(RSTIsrHandler);
CY_ISR_PROTO(SPI_IsrHandler);
CY_ISR_PROTO(SPI_SS_IsrHandler);
CY_ISR_PROTO(BRMS_Interrupt);

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
#define PWM_15KHZ_PERIOD                       (1600)
//#define PWM_PCT_TO_COMPARE(x)                  trunc((float) x * (PWM_15KHZ_PERIOD/100))
#define PWM_PCT_TO_COMPARE(x)                  trunc((float) x * 16)
#define PWM_IDLE                               (50.0)

/* TI INA219 Zero-Drift, Bidirectional Current/Power Monitor With I2C Interface */
#define INA219_I2C_ADDR                        (0x40)
#define INA219_CAL_VALUE                       (8192)

/* Neutral PWM jog is a 50% duty cycle */
#define PWM_JOG_NEUTRAL                        (50)

/* PWM maximum current value clipped to +/- X% duty cycle around the center (50 by default, full power possible) */
#define PWM_MAX_MAGNITUDE                      (50) 

/* --------------------------------------------------------------------- 
 * PID Defines
 * --------------------------------------------------------------------- */
#define PID_MANUAL                             (0)
#define PID_AUTOMATIC                          (1)
#define PID_EFFECTIVE_SETPOINT_DELTA_DEFAULT   (25)
bool inAuto = false;

volatile int8 Jog, LastJog;
bool PID_Enabled, PID_Was_Enabled;
int32 PID_Setpoint, PID_EffectiveSetpoint;
uint8 PID_EffSetDelta;
uint32 lastTime;
float Output;
float ITerm;
volatile int32 Position, LastPosition;
float kp, ki, kd; // PID values
uint32 refSampleTime = 5; // Default to 5ms
float outMax_Total, outMax_ITerm;
float pwmLimit, pwmMax, pwmMin;    
bool homingDone = true;

/* --------------------------------------------------------------------- 
 * Timekeeping defines
 * --------------------------------------------------------------------- */
volatile uint64 UptimeMicrosecondsAccumulator = 0;
volatile uint64 UptimeSeconds = 0;

/* --------------------------------------------------------------------- 
 * ENCODER PROPERTIES
 *
 * The encoder counts up and down in Counter_1, which is unsigned 24 bit
 *
 * Note: the negative boundary is defined as 1,048,576 counts of underflow
 * in the Counter_1 value.  This allows us to start at max travel, reset 
 * encoder to 0 and count negative the full travel before hitting the home
 * flag and resetting the counter to 0 again.
 *
 * (0x100000 or 1.048M is >3x the entire actuator travel)
 * --------------------------------------------------------------------- */
#define ENCODER_MAX                            (0xFFFFFF)             
#define ENCODER_NEGATIVE_BOUNDARY              (0xFFFFFF - 0x100000)  
#define ENCODER_COUNTS_PER_INDEX               (10000)

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

/* Fault codes, these are bit encoded into a uint8 */
typedef enum {
    fsNONE         = 0b00000000,    /* No faults detected */
    fsUnconfigured = 0b00000001,    /* Attempt to move an unconfigured system */
    fsEncoder      = 0b00000010,    /* One or more encoder phases not changing during a move */
    fsIndex        = 0b00000100,    /* Index marks not seen during a move */
    fsCurrentRead  = 0b00001000,    /* Unable to read current from INA219 device */
    fsChecksum     = 0b00010000,    /* Too many checksum faults */
    fsX2           = 0b00100000,    /* */
    fsX3           = 0b01000000,    /* */
    fsX4           = 0b10000000     /* */
} FaultStates_t;

volatile ConfigStates_t ConfigState;
volatile uint8 FaultState;
volatile uint8 ConfigSequence;
volatile uint16 ChecksumErrors;
volatile int16 Current;
volatile float PWM;
uint8 CurrentI2Cinbuf[20];

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
   talking.  Nominally 1 second max of not talking. */
#define MAX_LAST_MESSAGE_TIME_SECONDS 1
uint32 LastMessageTimeSeconds;

/* Opcodes that can come from the node box software */
typedef enum {
    opUNDEFINED = 0,
    opConfig    = 1,
    opStatus    = 2,
    opSetEnc    = 3,
    opMAX       
} rxMessage_opcodes_t;    

/* Sanity check for opcodes */
#define rxMessageOpcodeValid(op) ( ((rxMessage_opcodes_t) op > opUNDEFINED) && ((rxMessage_opcodes_t) op < opMAX) )

typedef struct { 
    uint8 checksum;        
    uint8 size;       /* Size of the message bytes, including opcode and size and checksum */
    uint8 opcode;     /* Operation (generic overlay for previewing opcode value) */
} __attribute__ ((__packed__)) rxMessage_overlay_t;

/* Configuration message, 19 bytes */
typedef struct {
    uint8 checksum;        
    uint8 size;       /* Size of the message bytes, including opcode and size and checksum */
    uint8 opcode;     /* Operation: 01 == config */        
    uint8 sequence;   /* Configuration message sequence number */
    float Kp;         /* PID constants to be used in calculation */
    float Ki;
    float Kd;
    uint8 limit_Total;/* Total drive output limit, +/- percentage */
    uint8 limit_ITerm;/* PID I term output limit, also +/- percentage */
    uint8 effsetdelta;/* PID effective setpoint increment delta, nominally 25 steps */
} __attribute__ ((__packed__)) rxMessage_config_t;

/* Status message, contains desired position and such values, 12 bytes */
typedef struct {
    uint8  checksum;        
    uint8  size;       /* Size of the message bytes, including opcode and size and checksum */
    uint8  opcode;     /* Operation: 02 == status */
    uint8  enable;     /* Enable/disable PID algorithm */
    int32  setpoint;   /* Setpoint (desired actuator position) */
    int8   jog;        /* Jog value, to manually move the motor; valid range -100 to 100 */ 
    uint8  clearfaults;/* Set to nonzero to clear all the current faults */
} __attribute__ ((__packed__)) rxMessage_status_t;
   
/* Clear message, contains new encoder position, 7 bytes */
typedef struct {
    uint8  checksum;        
    uint8  size;       /* Size of the message bytes, including opcode and size and checksum */
    uint8  opcode;     /* Operation: 03 == set encoders */
    int32  setpoint;   /* Setpoint (force an actuator logical position) */
} __attribute__ ((__packed__)) rxMessage_setenc_t;


/* Wrap the message with an array of bytes */
union {
    uint8               buf[MAX_MESSAGE_SIZE];
    rxMessage_overlay_t overlay;
    rxMessage_config_t  config;
    rxMessage_status_t  status;  
    rxMessage_setenc_t  setenc;
} rxMessage;

/* Message back to the BBB, watch out for alignment here by packing the structure (should be 18 bytes) */
typedef struct  {  
    uint8  checksum;   /* Message checksum */    
    uint8  version0;   /* Version byte 0 */ 
    uint8  version1;   /* Version byte 1 */
    uint8  version2;   /* Version byte 2 */
    uint8  size;       /* Size of the message bytes, including opcode and size and checksum */
    uint8  opcode;     /* Echo back of the opcode this response is for, operation: 02 == status */
    uint8  state;      /* Enum value for current device configuration state */ 
    uint8  fault;      /* Bit encoded fields for current fault status */
    uint8  sequence;   /* Echo back the config sequence number currently set */
    uint16 checksum_errors; /* Count of checksum errors */
    int16 current;     /* Motor current consumption (mA) */
    int32 position;    /* Actual actuator position, signed value */ 
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

/* --------------------------------------------------------------------- 
 * Function prototypes
 * --------------------------------------------------------------------- */
int32 GetPosition(void);
void runRateGroup1_PID(void);
void runRateGroup3_SPI(void);
void runRateGroup4_MotorCurrentRead(void);




/*******************************************************************************
* Function Name: AssertFault
********************************************************************************
* Summary:
*  Sets a flag that a particular fault has been detected.  Assert fsNONE to clear
*  all faults.
*
* Parameters: FaultStates_t of the fault detected.
* Return: None
*******************************************************************************/
void AssertFault(FaultStates_t fault) {
 
    switch (fault) {
     
        /* Clears all asserted faults */
        case fsNONE:
            FaultState = fsNONE;
            break;        
        
        /* Any other fault has its bit turned on in the fault status */
        default:
            FaultState |= fault;
            break;        
    }    
}


/*******************************************************************************
* Function Name: ClearFault
********************************************************************************
* Summary:
*  Clears a flag that a particular fault has been detected.  Assert fsNONE to clear
*  all faults.
*
* Parameters: FaultStates_t of the fault to be cleared.
* Return: None
*******************************************************************************/
void ClearFault(FaultStates_t fault) {
 
    switch (fault) {
     
        /* Clears all asserted faults */
        case fsNONE:
            FaultState = fsNONE;
            break;        
        
        /* Any other fault has its bit turned off in the fault status */
        default:
            FaultState &= ~fault;
            break;        
    }    
}


/*******************************************************************************
* Function Name: runRateGroup4_MotorCurrentRead
********************************************************************************
* Summary:
*  Task to read the motor current consumption.
*
* Parameters: None
* Return: None
*******************************************************************************/
void runRateGroup4_MotorCurrentRead(void) {
    
    static bool initialized = false;
    static float CurrentTemp;
    
    if (!initialized) {
        /* Start I2C for Current Monitor */
        I2C_Start();
        Init_INA(INA219_I2C_ADDR);
        
    initialized = true;
    }
    
    CurrentTemp = getCurrent_mA(INA219_I2C_ADDR);
            
    Current = (int16_t) CurrentTemp;
    //AssertFault(fsCurrentRead);
    
    /* Read the current at 2Hz */        
///        Sleep(500);
        
}


/*******************************************************************************
* Function Name: Comm_Task
********************************************************************************
* Summary:
*  Task to perform the SPI communications.
*
* Parameters: None
* Return: None
*******************************************************************************/
void runRateGroup3_SPI(void) {
 
    rxMessage_opcodes_t opcode;
    uint8 size;
    uint8 i;
    uint8 checksum;
    uint8 limit;    
    
    /* If the SPI is moving data out right now, do not touch the message buffer, we will
       get to it next cycle! */
    if (SPI_1_SpiIsBusBusy()) 
        return;
    
    /* In certain states, this thread is responsible for loading the outbound messaging */
    switch (txMessageState) {
     
        /* Output buffer is clear and ready for loading, rxMessage is (probably) good and needs processing */
        case txmsClear:
                
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
                        
                            /* Update the PID values passed down from the server */
                            kp = rxMessage.config.Kp;
                            ki = rxMessage.config.Ki;
                            kd = rxMessage.config.Kd;      
                        
                            /* PID effective setpoint increment delta value */                                      
                            PID_EffSetDelta = rxMessage.config.effsetdelta;
                               
                            /* Setup the output limits and stop a jog if one was in progress */
                            Jog = 0;
                        
                            /* Clip the limits to 100% duty cycle */
                            limit = rxMessage.config.limit_Total;
                            if (limit > 100)
                                limit = 100;
                        
                            outMax_Total = limit;                                        
                            
                            limit = rxMessage.config.limit_ITerm;                                        
                            if (limit > 100)
                                limit = 100;
                            
                            outMax_ITerm = limit;
                            
                            /* Convert the configured output maximum also into a PWM value, because the duty cycle
                            could be set from manual control of the PWM.  

                            outMax_Total ranges from 0 to 100% of power delivery, which means we need to 
                            convert it into a value symmetrically above and below the "neutral" center 
                            point value of 50 */    
                            pwmLimit = (outMax_Total/100 * 50);
                            pwmMax = 50 + pwmLimit;
                            pwmMin = 50 - pwmLimit;
                        
                            /* We have received a config message, so signal to the PID thread that processing is allowed */
                            ConfigState = csReady;  
                            ConfigSequence = rxMessage.config.sequence;
                        
                            /* Clear all the faults when reconfigured */
                            ClearFault(fsNONE);
                            break;

                        case opStatus:
                            /* The normal message telling us where to go, how much to jog, enable on/off */
                            PID_Enabled = (bool) rxMessage.status.enable;
                            
                            /* If we are commanded to move somewhere else, remember where we started */
                            if (PID_Setpoint != rxMessage.status.setpoint) {
                                
                                /* Remember where we started */
                                LastPosition = GetPosition();
                                
                                /* Update destination */
                                PID_Setpoint = rxMessage.status.setpoint;
                                
                                /* Initialize the effective setpoint to be equal to where we are right now,
                                it will be incremented/decremented when the PID algorithm runs next time.*/
                                PID_EffectiveSetpoint = LastPosition;                                                
                                
                                /* Reset counting of index marks */
                                Index_Counter_1_WriteCounter(0);
                            }                                        
                        
                            /* PWM jog value ranges from -50 to 50, where -50 is max-reverse current, 50 is max-forward, 0 is neutral/no movement */
                            Jog = rxMessage.status.jog;                                                    
                            break;
                        
                        case opSetEnc:
                            /* The message is telling us what to arbitrarily set the encoder values to */
                            Counter_1_WriteCounter(rxMessage.status.setpoint);   
                            LastPosition = rxMessage.status.setpoint;
                            break;                                        
                            
                        /* No other opcodes are valid */
                        default:
                            break;
                    }                           
                }                            
            }
            
            /* Get a fresh copy of the position information */
            Position = GetPosition();
            
            /* Fill out the common reponse the same way every time, as a status response */
            txMessage.msg.checksum = 0;
            txMessage.msg.version0 = FIRMWARE_REV_0;
            txMessage.msg.version1 = FIRMWARE_REV_1;
            txMessage.msg.version2 = FIRMWARE_REV_2;
            txMessage.msg.size     = sizeof(txMessage_t);
            txMessage.msg.opcode   = opStatus;
            txMessage.msg.state    = (uint8) ConfigState;
            txMessage.msg.fault    = (uint8) FaultState;
            txMessage.msg.checksum_errors = ChecksumErrors;
            txMessage.msg.sequence = ConfigSequence;
            txMessage.msg.position = Position;
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
            
            /* Clear all the faults if told to */
            if ((bool) rxMessage.status.clearfaults)
                ClearFault(fsNONE);
        
            break;
    
        /* A message was already readied for transmission, nothing to do here */
        case txmsLoaded:                
            break;
                
    } // End of message state case statement

}
    

/*******************************************************************************
* Function Name: PWM_Set
********************************************************************************
* Summary:
*  Sets the duty cycle of the PWM at the output pin.
*
* Parameters: Duty cycle, in percent.  A value of 50 is "neutral", values up 
*             to 100 is forward drive, and below 50 down to 0 is backward drive.
* Return: None
*******************************************************************************/
void PWM_Set(float dutycycle) {
    
    float drive = dutycycle;
    
    /* Clip to the max PWM drive +/- around 50 */
    if (drive > pwmMax) {
        drive = pwmMax;
    } else if (drive < pwmMin) {
        drive = pwmMin;
    }
    
    PWM_1_WriteCompare(PWM_PCT_TO_COMPARE(drive));    
}


/*******************************************************************************
* Function Name: GetPosition
********************************************************************************
* Summary:
*  Get the physical position value and compensate for negative locations.
*
* Parameters: None
* Return: int32 (signed!) position value
*******************************************************************************/
int32 GetPosition(void) {
    
    static volatile uint32 RawPosition;
    static volatile int32 result;
    
    /* Get up-to-date position from the 24 bit unsigned counter*/
    RawPosition = Counter_1_ReadCounter();   
    
    /* If the raw position is higher than some extremely high number, treat it as 
       underflow and make that into a negative value */
    if (RawPosition > ENCODER_NEGATIVE_BOUNDARY) {
        
        result = (-1) * ((int32) ENCODER_MAX - (int32) RawPosition);
        
    } else {
     
        /* Value is "positive", treat it normally */
        result = (int32) RawPosition;
    }
    
    /* Update the master copy of position information */
    Position = result;
    return result;
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
    
    /* Get up-to-date position */
    Position = GetPosition();
    LastPosition = Position;
    
    /* Initialize the effective setpoint to be equal to where we are right now,
    it will be incremented/decremented when the PID algorithm runs next time */
    PID_EffectiveSetpoint = LastPosition;
    
    /* 2019-03-13 PMR: Init to zero instead of the output value, since we are not
       switching from manual to auto frequently */
    ITerm = 0;
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
#ifdef ZERO
float PID_Compute(uint32 setpoint) {
    
    int32 error, dPosition;
        
    if(!inAuto) 
        return 0;
    
    /* Get most up-to-date current position */
    Position = GetPosition();

    /* Compute all the working error variables */
    error = setpoint - Position;
    ITerm += (ki * error);
    
    /* Clip the I term at a max value for just that term (windup guard) */
    if (ITerm > outMax_ITerm) {
        ITerm = outMax_ITerm;
    } else if (ITerm < -outMax_ITerm) {
        ITerm = -outMax_ITerm;
    }
    
    /* Calculate the error term */
    dPosition = (Position - LastPosition);

    /* Compute PID Output */
    Output = (kp * error) + ITerm - (kd * dPosition);
    
    /* Clip the output */
    if (Output> outMax_Total) {
        Output = outMax_Total;
    } else if (Output < -outMax_Total) {
        Output = -outMax_Total;
    }

    /* Remember some variables for next time */
    LastPosition = Position;
    
    return Output;    
}
#endif

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
* Function Name: runRateGroup1_PID
********************************************************************************
* Summary:
*  Task to perform the PID calculations.
*
* Parameters: None
* Return: None
*******************************************************************************/
void runRateGroup1_PID(void) {
    
    static bool initialized = false;
    
    static int32 DeltaPosition;
    static uint32 CurrentIndexCount;
    int32 error, dPosition;
    
    /* Initial values */

    if (!initialized) {
        
        /* Setup the PID subsystem */
        PID_Initialize();
        PID_SetMode(PID_MANUAL);
        
        /* Initially default to 100% output max until config tells us otherwise */
        outMax_Total = 100;
        outMax_ITerm = 100;

        /* Init the PWM limits the same way, full maximums */
        pwmLimit = 50;   // This is a + or - value
        pwmMax = PWM_JOG_NEUTRAL + pwmLimit;
        pwmMin = PWM_JOG_NEUTRAL - pwmLimit;    
        
        /* Start off disabled */
        PID_Setpoint = 0;  
        PID_EffectiveSetpoint = 0;
        PID_EffSetDelta = PID_EFFECTIVE_SETPOINT_DELTA_DEFAULT;
        PID_Was_Enabled = false;
        PID_Enabled = false;
        
        initialized = true;
    }
    
    /* ------------------------------------------------------------------------------------ */
    /* At the top of the PID loop, refresh the counter of the watchdog to indicate the task 
       is still alive.  Were the BRMS to stop working or the motion thread to die, the CPU 
       will be reset after 2 seconds. */
    WDT_COUNT1_REFRESH();        
    /* ------------------------------------------------------------------------------------ */

    
    
    
    /////////////////////////////////////////////////////
    // TESTING ONLY
    //ConfigState = csReady;
    //PID_Enabled = true;
    /////////////////////////////////////////////////////
        

    
    
    /* If the server hasn't talked to us in a while (no messages on the SPI), 
       take preventative action and abandon any moves in progress. */
    if (UptimeSeconds > (LastMessageTimeSeconds + MAX_LAST_MESSAGE_TIME_SECONDS)) {
        
        /* Stop all motion */
        PWM_Set(PWM_JOG_NEUTRAL);
        PID_Enabled = false;
        
        /* Clear the values that would drive motion on the next message arrival.  Assume the next message might be a config, 
           in which case we want to be neutral. */
        Jog = 0;

    /* Only run the PID algorithm if we have been configured by the nodebox software */
    } else if (ConfigState == csReady) {
        
        /* Enable the drive outputs for the home and index once configured, otherwise they
           can screw up the boot pins on the BeagleBoneBlack */
        HOME_OUT_SetDriveMode(HOME_OUT_DM_STRONG); 
        INDEX_OUT_SetDriveMode(INDEX_OUT_DM_STRONG); 
    
        /* If the server is asking us to jog, do that instead of PID */
        if (!PID_Enabled) {                   
            
            /* When we start homing, it looks like a big negative jog.  Setup to watch the index marks as we jog, 
               so we can assert faults if they aren't changing. */                
            if ((Jog != LastJog) && (Jog != 0)) {
                
                /* Update the 'last' jog value, so we don't fall into this reset code over and over */
                LastJog = Jog;
                
                /* Remember where we started */
                LastPosition = GetPosition();
                
                /* Reset counting of index marks */
                Index_Counter_1_WriteCounter(0);
            }                
            
            /* Drive in the direction and speed the server told us */
            PWM_Set(PWM_JOG_NEUTRAL + Jog);

            /* Watch for stuck signals while we are moving */
            if ( Jog != 0 ) {
            
                /* If we have moved a good distance away from the origin point, compare index counts versus position counts 
                   looking for discrepancy. */
                Position = GetPosition();
                DeltaPosition = labs( Position - LastPosition );
                CurrentIndexCount = Index_Counter_1_ReadCounter();
                
                /* Look for index failure: the index counter should have incremented by at least 1 by now but only if 
                   homing is complete */
                if (homingDone) {
                    if (CurrentIndexCount == 0)
                        if (DeltaPosition > 2*ENCODER_COUNTS_PER_INDEX) 
                            AssertFault(fsIndex);        
                }
                
                /* Look for encoder failure: the encoders should register some amount of movement if the index has changed,
                   but only if homing is totally done */
                if (homingDone) {
                    if ((CurrentIndexCount > 0) && (DeltaPosition < 2))                     
                        AssertFault(fsEncoder);
                }
            }
            
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
        
        /* Calculate the effective setpoint, which is defined as N (nominally 25) counts closer to the 
        actual setpoint, incremented once per cycle of this algorithm.  
    
        Consider a move of +2000 counts from position 0 to 2000: 
        
        1) The setpoint will change to 2000.
        2) The effective setpoint is initialized to the current position, plus 25 counts = 25.
        3) Calculate the PID and return.
        4) The next time PID_Compute is called, increment the effective setpoint by 25 counts = 50.
        5) Calculate the PID and return.
        6) Repeat steps 4 and 5 until the effective setpoint equals the actual setpoint.             
        */

        if (PID_Enabled) {
            
            if (PID_EffSetDelta == 0) {
                PID_EffectiveSetpoint = PID_Setpoint;
            } else if ( labs(PID_EffectiveSetpoint - PID_Setpoint) <= (2 * PID_EffSetDelta) ) {
                PID_EffectiveSetpoint = PID_Setpoint;                                                
            } else if (PID_Setpoint > PID_EffectiveSetpoint) {
                PID_EffectiveSetpoint = (PID_EffectiveSetpoint + PID_EffSetDelta);
            } else {
                PID_EffectiveSetpoint = (PID_EffectiveSetpoint - PID_EffSetDelta);
            }
            
            /* ---------------------------------------------------- */
            /* Run the PID algorithm against the effective setpoint */
            
            /* Get most up-to-date current position */
            Position = GetPosition();
            
            /* Compute all the working error variables */
            error = PID_EffectiveSetpoint - Position;
            ITerm += (ki * error);
            
            /* Clip the I term at a max value for just that term (windup guard) */
            if (ITerm > outMax_ITerm) {
                ITerm = outMax_ITerm;
            } else if (ITerm < -outMax_ITerm) {
                ITerm = -outMax_ITerm;
            }
            
            /* Calculate the error term */
            dPosition = (Position - LastPosition);

            /* Compute PID Output */
            Output = (kp * error) + ITerm - (kd * dPosition);
            
            /* Clip the output */
            if (Output> outMax_Total) {
                Output = outMax_Total;
            } else if (Output < -outMax_Total) {
                Output = -outMax_Total;
            }

            /* Remember some variables for next time */
            LastPosition = Position;
        
            /* Use the global PWM value to communicate back the percentage of drive to the server */
            PWM = Output;
            
            /* Put the PID drive percentage out on the wire */
            PID_SetDrive(Output);
            
        } else {
            /* If disabled, just return 0 */
            PWM = 0;
        }
        
    }
  
}


/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
*  Setup tasks, interrupts, and perform the background task functions.
*
* Parameters: None
* Return: NEVER!
*******************************************************************************/
int main(void) {
    
    /* DISABLE the drive outputs for the home and index immediately upon booting the 
       microprocessor   There is a race condition here: unless the actuator is on a home
       flag or index mark, a 1 will be written to each output.  Depending on which 
       Cypress device this is, it could end up going to one of the boot pins of the Beagle
       Bone Black device.  Undesirable results can result: corrupt serial output on the
       console, inabulity to boot from eMMC, or even a complete failure to power on. */
    HOME_OUT_SetDriveMode(HOME_OUT_DM_DIG_HIZ); 
    INDEX_OUT_SetDriveMode(INDEX_OUT_DM_DIG_HIZ); 

    
    /********************************************************************** 
    * Interrupts
    **********************************************************************/
    
    /* BRMS timer interrupt */
    Timer_BRMS_Start();
    isr_brms_StartEx(BRMS_Interrupt);
    isr_brms_SetPriority(HIGHER_PRIORITY);
    
    /* Sets up the Index and Reset interrupt and enables them */
    isr_home_StartEx(HomeIsrHandler);
    isr_home_SetPriority(DEFAULT_PRIORITY);

    /* Encoder interrupt */
    isr_rst_encoder_StartEx(RSTIsrHandler);
    isr_rst_encoder_SetPriority(DEFAULT_PRIORITY);
    
    /* Setup the SPI slave select interrupt ISR */
    isr_spi_ss_StartEx(SPI_SS_IsrHandler);
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
    Jog = 0;
    
    /* Set a flag that homing is not done yet, since we just booted */
    homingDone = false;
   
    /* Start counting the quadrature encoding */
    Counter_1_Start();    
    Counter_1_WriteCounter(ENCODER_MAX);  // Set the encoder initially to mid range
    LastPosition = ENCODER_MAX;
    
    /* Clear and start the index mark counter */
    Index_Counter_1_Start();
    Index_Counter_1_WriteCounter(0);
     
    /* Start off unconfigured */
    ConfigState = csUnconfig;  
    ConfigSequence = 0;
    ChecksumErrors = 0;

    /***********************************************************************
    * Run the background tasks.  Assume anything executed in here will be
    * constantly interrupted by the task scheduler.
    ***********************************************************************/
    while (1) {
        
        CyDelay(1000u); 
        
        
    }
    
    
    
    /***********************************************************************
    *  We should never reach this, if we do, we'll crash (reset).
    ***********************************************************************/
    return 1;       
}


/*******************************************************************************
* Function Name: BRMS_Interrupt
********************************************************************************
* Summary:
*  Hooks the 200us tick for the BRMS scheduler.
*
* Parameters: None
* Return: None
*******************************************************************************/
CY_ISR(BRMS_Interrupt) {
    
    static uint32 brmsTask;                 // The BRMS schedule counter

    static uint32 brmsRG1Mask = 0b00000001; // Rate group 1 mask
    static uint32 brmsRG2Mask = 0b00000010; // Rate group 2 mask
    static uint32 brmsRG3Mask = 0b00000100; // Rate group 3 mask
    static uint32 brmsRG4Mask = 0b00001000; // Rate group 4 mask
    static uint32 brmsRG5Mask = 0b00010000; // Rate group 5 mask
    
    /* Clears the timer interrupt */
    Timer_BRMS_ClearInterrupt(Timer_BRMS_INTR_MASK_CC_MATCH);

    /* Use the LED to indicate we are servicing the BRMS interrupt */
    LED_Write(1);
    
    /* 200us has elapsed since last we were called */
    UptimeMicrosecondsAccumulator += 200;

    /* When we reach 1M microseconds, a second has passed */
    if (UptimeMicrosecondsAccumulator == 1000000) {
        UptimeMicrosecondsAccumulator = 0;
        UptimeSeconds++;
    }

    /* Increment the BRMS task counter infinitely */
    brmsTask++;

    /* Determine which rate group to run.  Do this by applying the rate group masks
       sequentially until one results in a "true" value.  
    
       For example: the 1st rate group is invoked every time the brmsTask value ends 
       in 0bxxx1, and ignored when it's 0bxxx0.  
    
       The second rate group is invoked half as often as the first: when the brmsTask
       value ends in 0bxx10. 
    
       The third rate group is invoked half as often as the second: when brmsTask
       ends in 0bx100.  
    
       In this way, we have decreasing tiers of tasks that are run for at most 200us.
    
       The "background" task does not run at interrupt level.  The main() of the program
       represents everything non time critical, using whatever CPU is left over when the
       interrupt returns.
    */
    if (brmsTask & brmsRG1Mask) {
        
        /* The PID task has the highest priority for this system.  Run it every time rate 
           group 1 comes around, which results in an invocation of PID every 400us.  Equal
           to a 2.5KHz update rate. */
        runRateGroup1_PID();
        
    } else if (brmsTask & brmsRG2Mask) {
        
        /* Rate group 2 is run every 0.8ms, or 1.25KHz*/
        //runRateGroup2_TBD();        
        
    } else if (brmsTask & brmsRG3Mask) {
        
        /* Rate group 3 is run every 1.6ms, or 625Hz*/
        runRateGroup3_SPI();
        
    } else if (brmsTask & brmsRG4Mask) {
     
        /* Rate group 4 is run every 3.2ms, or 312Hz*/
        ///runRateGroup4_MotorCurrentRead();

    } else if (brmsTask & brmsRG5Mask) {
     
        /* Rate group 5 is run every 6.4ms, or 156Hz*/
        //runRateGroup5_TBD();
    }
    
    /* At the exit from the BRMS handler, turn off the LED */
    LED_Write(0);
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
    
    /* Clear the index counter */
    Index_Counter_1_WriteCounter(0);  
    
    /* When we hit the index mark, homing is complete */
    homingDone = true;    
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
    
    /* Set a flag that homing is not done yet until the next index position */
    homingDone = false;
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
    LastMessageTimeSeconds = UptimeSeconds;

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

