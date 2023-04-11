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
* 04/07/23 PMR  Rev: 0-2-1 add move timer to messaging, fix jog function
* 02/08/23 PMR  Rev: 0-2-0 rework PID algorithm based on Galil findings
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
#include "pid.h"
#include "INA219.h"

/* Firmware revision as of 2023-04-07 */
#define FIRMWARE_REV_0 0
#define FIRMWARE_REV_1 2
#define FIRMWARE_REV_2 1

/* Debugging - undefine this for a production system that needs to watchdog */
#define DEBUG_PROBE_ATTACHED 1

/* For the ACS test set, the drives are wired backwards!  Use a polarity of -1 in that case. */
//#define DRIVE_POLARITY -1
#define DRIVE_POLARITY 1

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
#define PWM_NEUTRAL                            (PWM_15KHZ_PERIOD/2)

/* TI INA219 Zero-Drift, Bidirectional Current/Power Monitor With I2C Interface */
#define INA219_I2C_ADDR                        (0x40)
#define INA219_CAL_VALUE                       (8192)

/* PWM maximum current value clipped to +/- X% duty cycle around the center (50 by default, full power possible) */
#define PWM_MAX_MAGNITUDE                      (50) 

/* --------------------------------------------------------------------- 
 * PID Defines
 * --------------------------------------------------------------------- */
#define PID_MANUAL                             (0)
#define PID_AUTOMATIC                          (1)
#define PID_EFFECTIVE_SETPOINT_DELTA_DEFAULT   (250)
#define OVERRIDE_PID_CONSTANTS                 1
#define ITERM_90PCT_FIT_CONSTANT               (13)

bool inAuto = false;

volatile int8_t Jog;
bool PID_Enabled, PID_Was_Enabled;
int32_t PID_Setpoint, PID_EffectiveSetpoint;
uint8_t PID_EffSetDelta;
uint32_t lastTime;

volatile int32_t Position, LastPosition;
volatile int32_t Output;
volatile uint16_t limitOutput;
volatile uint16_t limitIterm;

bool homingDone = true;

/* --------------------------------------------------------------------- 
 * Move timer defines
 * 
 * Wait for 30 samples of position to be the same before declaring a move
 * done.
 * --------------------------------------------------------------------- */
#define LAST_MOVE_TIME_SAMPLE_COUNT 30

/* Use UptimeMicroseconds to calculate how long a move elapsed */
uint32_t LastMoveStartTimeUsec = 0;
uint32_t LastMoveEndTimeUsec = 0;

/* Counter for how many samples elapsed since the move was complete */
uint16_t LastMoveStableCount = 0;

/* Time value in microseconds for how long the last move took */
uint32_t LastMoveTimeUsec = 0;

/* Set this flag when a new move comes down from the ACS */
bool NewCommandedMove = false;

/* --------------------------------------------------------------------- 
 * Timekeeping defines
 * --------------------------------------------------------------------- */
volatile uint64_t UptimeMicroseconds = 0;
volatile uint64_t UptimeMicrosecondsAccumulator = 0;
volatile uint64_t UptimeMilliseconds = 0;
volatile uint64_t UptimeMillisecondsAccumulator = 0;
volatile uint64_t UptimeSeconds = 0;

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

/* Fault codes, these are bit encoded into a uint8_t */
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
volatile uint8_t FaultState;
volatile uint8_t ConfigSequence;
volatile uint16_t ChecksumErrors;
volatile int16_t MotorCurrent;
uint8_t CurrentI2Cinbuf[20];

/* --------------------------------------------------------------------- 
 * SPI MESSAGING
 * --------------------------------------------------------------------- */
/* V2 protocol 

   1) Messages have opcodes (configuration, position, get status)
   2) Messages are validated by a checksum instead of an arbitrary signature pattern.
   3) Transfer the max message size every time, regardless of all bytes used or not.
*/
    
/* Set this to be at least the size of the status response message */
#define MAX_MESSAGE_SIZE 27
    
/* Remember the last time a message came in so we can timeout moves if the node box stops 
   talking.  Nominally 1 second max of not talking. */
#define MAX_LAST_MESSAGE_TIME_SECONDS 1
uint32_t LastMessageTimeSeconds;

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
    uint8_t checksum;        
    uint8_t size;       /* Size of the message bytes, including opcode and size and checksum */
    uint8_t opcode;     /* Operation (generic overlay for previewing opcode value) */
} __attribute__ ((__packed__)) rxMessage_overlay_t;

/* Configuration message, 22 bytes */
typedef struct {
    uint8_t checksum;        
    uint8_t size;        /* Size of the message bytes, including opcode and size and checksum */
    uint8_t opcode;      /* Operation: 01 == config */        
    uint8_t sequence;    /* Configuration message sequence number */
    
    uint32_t overrideKp; /* If nonzero, override the PID P term */
    uint32_t overrideKi; /* If nonzero, override the PID I term */
    uint32_t overrideKd; /* If nonzero, override the PID D term */
    
    uint16_t limitOutput;/* Drive output limit, ranges from 0 to 800 */
    uint16_t limitIterm; /* PID I term output limit, ranges from 0 to 800 */
    uint16_t effsetdelta;/* PID effective setpoint increment delta, nominally 250 steps */
} __attribute__ ((__packed__)) rxMessage_config_t;

/* Status message, contains desired position and such values, 10 bytes */
typedef struct {
    uint8_t  checksum;        
    uint8_t  size;       /* Size of the message bytes, including opcode and size and checksum */
    uint8_t  opcode;     /* Operation: 02 == status */
    uint8_t  enable;     /* Enable/disable PID algorithm */
    int32_t  setpoint;   /* Setpoint (desired actuator position) */
    int8_t   jog;        /* Jog value, to manually move the motor; valid range -100 to 100 */ 
    uint8_t  clearfaults;/* Set to nonzero to clear all the current faults */
} __attribute__ ((__packed__)) rxMessage_status_t;
   
/* Clear message, contains new encoder position, 7 bytes */
typedef struct {
    uint8_t  checksum;        
    uint8_t  size;       /* Size of the message bytes, including opcode and size and checksum */
    uint8_t  opcode;     /* Operation: 03 == set encoders */
    int32_t  setpoint;   /* Setpoint (force an actuator logical position) */
} __attribute__ ((__packed__)) rxMessage_setenc_t;


/* Wrap the message with an array of bytes */
union {
    uint8_t             buf[MAX_MESSAGE_SIZE];
    rxMessage_overlay_t overlay;
    rxMessage_config_t  config;
    rxMessage_status_t  status;  
    rxMessage_setenc_t  setenc;
} rxMessage;

/* Message back to the BBB, watch out for alignment here by packing the structure (27 bytes) */
typedef struct  {  
    uint8_t  checksum;        /* Message checksum */    
    uint8_t  version0;        /* Version byte 0 */ 
    uint8_t  version1;        /* Version byte 1 */
    uint8_t  version2;        /* Version byte 2 */
    uint8_t  size;            /* Size of the message bytes, including opcode and size and checksum */
    uint8_t  opcode;          /* Echo back of the opcode this response is for, operation: 02 == status */
    uint8_t  state;           /* Enum value for current device configuration state */ 
    uint8_t  fault;           /* Bit encoded fields for current fault status */
    uint8_t  sequence;        /* Echo back the config sequence number currently set */
    uint16_t checksum_errors; /* Count of checksum errors */
    int16_t  motor_current;   /* Motor current consumption (mA) */
    int32_t  position;        /* Actual actuator position, signed value */ 
    int16_t  pwm;             /* PWM value the motor is moving at */     
    int32_t  iterm;           /* Instantaneous PID iterm value */
    uint32_t last_move_time;  /* Amount of time for the last move, in us */ 
} __attribute__ ((__packed__)) txMessage_t;

/* Wrap the message with an array of bytes */
union {    
    uint8_t     buf[MAX_MESSAGE_SIZE];
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
int32_t GetPosition(void);
void runRateGroup1_PID(void);
void runRateGroup3_SPI(void);
void MotorCurrentRead(void);


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
* Function Name: MotorCurrentRead
********************************************************************************
* Summary:
*  Read the motor current consumption.
*
* Parameters: None
* Return: None
*******************************************************************************/
void MotorCurrentRead(void) {
    
    //TODO: Convert this to integer math
    float c;
    
    c = getCurrent_mA(INA219_I2C_ADDR);
    
    /* Assign the global MotorCurrent value */
    MotorCurrent = (int16_t) c;       
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
    uint8_t size;
    uint8_t i;
    uint8_t checksum;
    int32_t distance;
    
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
                        
                            /* Remember where we were when the init was sent */
                            LastPosition = GetPosition();
                           
                            /* Initialize the setpoints to be equal to where we are right now.*/
                            PID_Setpoint = LastPosition;
                            PID_EffectiveSetpoint = LastPosition;                                                
                            
                            /* Disable PID if it's on */
                            PID_Was_Enabled = false;
                            PID_Enabled = false;

                            /* Stop a jog if one was in progress */
                            Jog = 0;
                            
                            /* Output and Iterm limits */
                            limitOutput = rxMessage.config.limitOutput;
                            limitIterm = rxMessage.config.limitIterm;
                                
                                
                            /* Update the PID values passed down from the server */
                        
#ifndef OVERRIDE_PID_CONSTANTS
                            kp = rxMessage.config.Kp;
                            ki = rxMessage.config.Ki;
                            kd = rxMessage.config.Kd;      
                        
                            /* PID effective setpoint increment delta value */                                      
                            PID_EffSetDelta = rxMessage.config.effsetdelta;
#endif
                        
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
                                
                                /* This is a new move, start timing! */
                                NewCommandedMove = true;
                                LastMoveStartTimeUsec = UptimeMicroseconds;
                                
                                /* Remember where we started */
                                LastPosition = GetPosition();
                                
                                /* Update destination */
                                PID_Setpoint = rxMessage.status.setpoint;
                                
                                /* Initialize the effective setpoint to be equal to where we are right now,
                                it will be incremented/decremented when the PID algorithm runs next time.*/
                                PID_EffectiveSetpoint = LastPosition;                                                
                                
                                /* Reset counting of index marks */
                                Index_Counter_1_WriteCounter(0);
                                
                                
                                /* The demand has changed.  Hold off the integrator for a certain amount of time,
                                dictated by the size of the move (if it's more than 50 counts) */
                                distance = PID_Setpoint - LastPosition;                               
                                if (distance < 0) {
                                    distance *= -1;
                                }
                                
                                if (distance > 50) {                                
                                    iterm_delay = ((distance / 16) + ITERM_90PCT_FIT_CONSTANT) * 1000;
                                    //iterm_delay = distance >> 4;  // Shift by 4 is equal to div by 16
                                } else {
                                    iterm_delay = 0;
                                }
                                
                                
                                /* The demand has changed, reset the iterm delay to the max */
                                //iterm_delay = ITERM_DELAY_DEFAULT;  
                                //iterm_delay = 0;
                            }                                        
                        
                            /* PWM jog value ranges from -100 to 100, where -100 is max-reverse current, 
                            100 is max-forward, 0 is neutral/no movement */
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
            txMessage.msg.checksum        = 0;
            txMessage.msg.version0        = FIRMWARE_REV_0;
            txMessage.msg.version1        = FIRMWARE_REV_1;
            txMessage.msg.version2        = FIRMWARE_REV_2;
            txMessage.msg.size            = sizeof(txMessage_t);
            txMessage.msg.opcode          = opStatus;
            txMessage.msg.state           = (uint8_t) ConfigState;
            txMessage.msg.fault           = (uint8_t) FaultState;
            txMessage.msg.checksum_errors = ChecksumErrors;
            txMessage.msg.sequence        = ConfigSequence;
            txMessage.msg.position        = Position;
            txMessage.msg.pwm             = Output;
            txMessage.msg.iterm           = iterm;
            txMessage.msg.motor_current   = MotorCurrent;                  
            txMessage.msg.last_move_time  = LastMoveTimeUsec;
            
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
void PWM_Set(int32_t output) {    
        
    /* output varies from -800 to 800, which needs to be expressed as 0 to 1600 
       for the PWM */
    PWM_1_WriteCompare((PWM_15KHZ_PERIOD/2) + (DRIVE_POLARITY * output));    
}


/*******************************************************************************
* Function Name: GetPosition
********************************************************************************
* Summary:
*  Get the physical position value and compensate for negative locations.
*
* Parameters: None
* Return: int32_t (signed!) position value
*******************************************************************************/
int32_t GetPosition(void) {
    
    static volatile uint32_t RawPosition;
    static volatile int32_t result;
    
    /* Get up-to-date position from the 24 bit unsigned counter*/
    RawPosition = Counter_1_ReadCounter();   
    
    /* If the raw position is higher than some extremely high number, treat it as 
       underflow and make that into a negative value */
    if (RawPosition > ENCODER_NEGATIVE_BOUNDARY) {
        
        result = (-1) * ((int32_t) ENCODER_MAX - (int32_t) RawPosition);
        
    } else {
     
        /* Value is "positive", treat it normally */
        result = (int32_t) RawPosition;
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
    iterm = 0;
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
void PID_SetMode(uint32_t Mode) {
    
    bool newAuto = (Mode == PID_AUTOMATIC);
    
    if (newAuto && !inAuto) {  
        /*we just went from manual to auto*/
        PID_Initialize();
    }
    
    inAuto = newAuto;
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
     
    /////////////////////////////////////////////////////
    // TESTING ONLY
    //ConfigState = csReady;
    //PID_Enabled = true;
    /////////////////////////////////////////////////////

    /* If the server hasn't talked to us in a while (no messages on the SPI), 
       take preventative action and abandon any moves in progress. */
    if (UptimeSeconds > (LastMessageTimeSeconds + MAX_LAST_MESSAGE_TIME_SECONDS)) {
        
        /* Stop all motion */
        PWM_Set(PWM_NEUTRAL);
        PID_Enabled = false;
        
        /* Clear the values that would drive motion on the next message arrival.  Assume the 
        next message might be a config, in which case we want to be neutral. */
        Jog = 0;

    /* Only run the PID algorithm if we have been configured by the nodebox software */
    } else if (ConfigState == csReady) {
        
        /* Enable the drive outputs for the home and index once configured, otherwise they
           can screw up the boot pins on the BeagleBoneBlack */
        HOME_OUT_SetDriveMode(HOME_OUT_DM_STRONG); 
        INDEX_OUT_SetDriveMode(INDEX_OUT_DM_STRONG); 

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
            
            /* Get most up-to-date current position */
            Position = GetPosition();
            
            /* Run the PID algorithm once */
            Output = PID_UpdateValues(PID_EffectiveSetpoint, Position);
            
            /* Put the PID output value out on the wire */
            PWM_Set(Output);
            
            /* Calculate how long the last commanded move has taken */
            if (NewCommandedMove) {
                
                if (Position == PID_Setpoint) {
                    
                    /* We are at the set point, but it's not known if the motion is stable yet, we might have overshot. 
                    Therefore, count how many times we have held at this location.  When it exceeds a given threshold, 
                    use the time we first reached this position as the move's end time. */
                    if (LastMoveStableCount == LAST_MOVE_TIME_SAMPLE_COUNT) {
                        
                        /* We have arrived at the set point and have been here for 6ms (30 counts of 200us), call it 
                        good and calculate how long this move took */
                        LastMoveTimeUsec = LastMoveEndTimeUsec - LastMoveStartTimeUsec;
                        
                        /* Clear the flag for this particular move */ 
                        NewCommandedMove = false;
                        
                    } else {
                        
                        /* We have reached the destination but it is not yet proven stable. Increment the stability counter.  
                        It will be reset to zero if we deviate from this position. */                        
                        LastMoveStableCount += 1;

                        /* If the stable count is exactly 1, then this might be the start of a new stable point,
                        note the time.  This is potentially the END TIME of a move.  The start time was recorded
                        when the node box sent a new set point. */
                        if (LastMoveStableCount == 1) {
                            LastMoveEndTimeUsec = UptimeMicroseconds;
                        }                    
                    }                
                    
                } else {
                    
                    /* Reset the stable counter, because we are not at the commanded location */
                    LastMoveStableCount = 0;                
                }
            }

        } else {
            
            /* If the server is asking us to jog, do that instead of PID.  Drive in the direction 
            and speed the server told us.  Translate the jog percentage, from -100 to +100, into 
            PWM_Set values from -800 to 800 */
            Output = (Jog * 8);
            
            /* Put the new jog value on the wire, instead of a PID value */
            PWM_Set(Output);            
        }        
        
    } else {
        
        /* Config state is not ready (configured), inhibit all motion */
        PWM_Set(PWM_NEUTRAL);        
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
       microprocessor.  There is a race condition here: unless the actuator is on a home
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
    
    /* Start I2C for the Motor Current monitor */
    //Init_INA(INA219_I2C_ADDR);
    
    CyDelay(100u);
    SPI_1_Start();
    
    /* Setup the PWM at a base frequency of 15KHz, 50% duty cycle.  Clock_1 is set to
       12MHz, so the desired period to get 15KHz is a count of 800. */
    PWM_1_Start();
    PWM_1_WritePeriod(PWM_15KHZ_PERIOD);
    PWM_Set(PWM_NEUTRAL);   
    
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
    
    /***************************/
    /* Setup the PID subsystem */
    PID_Initialize();
    PID_SetMode(PID_MANUAL);
    
    /* Initially default to full output max until config tells us otherwise */
    limitOutput = 800;
    limitIterm = 800;
    
    /* Start off disabled */
    PID_Setpoint          = 0;  
    PID_EffectiveSetpoint = 0;
    PID_EffSetDelta       = 250; //PID_EFFECTIVE_SETPOINT_DELTA_DEFAULT;
    PID_Was_Enabled       = false;
    PID_Enabled           = false;
    PWM_Set(PWM_NEUTRAL);
  
    /***********************************************************************
    * Run the background tasks.  Assume anything executed in here will be
    * constantly interrupted by the task scheduler.
    ***********************************************************************/
    while (1) {

        //TODO: Enable/disable this mechanism based on some input from the SPI master,
        //      which might be the ACS test set, or the beagle bone node box.  Disable it for
        //      now to prevent multi-mastering of the I2C bus, which doesn't work.
        /*
        PROBE_Write(1);
        MotorCurrentRead();
        PROBE_Write(0);
        */
        
        /*
        if (UptimeMicroseconds > (LastUptimeMicroseconds + 500)) {
            LastUptimeMicroseconds = UptimeMicroseconds;
            
            if (toggle) {
                PROBE_Write(1);
                toggle = false;
            } else {
                PROBE_Write(0);
                toggle = true;
            }
        }
        */
        
        
        /* Use the LED as a heartbeat */
        if (UptimeSeconds % 2) {
            LED_Write(1);
        } else {
            LED_Write(0);
        } 
        
        // Delay 10ms before next loop iteration
        CyDelay(10);         
        
        
   
    /* ------------------------------------------------------------------------------------ */
    /* Every loop, refresh the counter of the watchdog to indicate the system is still alive
       and not stalled out in some interrupt somewhre.  Were the BRMS to stop working or the 
       motion thread to die, the CPU will be reset after 2 seconds. */
////    WDT_COUNT1_REFRESH();        
    /* ------------------------------------------------------------------------------------ */
        
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
    
    static uint32_t brmsTask;                 // The BRMS schedule counter

    static uint32_t brmsRG1Mask = 0b00000001; // Rate group 1 mask
    static uint32_t brmsRG2Mask = 0b00000010; // Rate group 2 mask
    static uint32_t brmsRG3Mask = 0b00000100; // Rate group 3 mask
    static uint32_t brmsRG4Mask = 0b00001000; // Rate group 4 mask
    static uint32_t brmsRG5Mask = 0b00010000; // Rate group 5 mask
    
    /* Clears the timer interrupt */
    Timer_BRMS_ClearInterrupt(Timer_BRMS_INTR_MASK_CC_MATCH);

    PROBE_Write(1);
    
    /* Use this 200us interrupt as a clock mechanism */
    UptimeMicroseconds += 200;
    UptimeMicrosecondsAccumulator += 200;
    
    /* Count milliseconds */
    if (UptimeMicrosecondsAccumulator > 1000) {
        UptimeMicrosecondsAccumulator = 0;
        UptimeMilliseconds += 1;
        UptimeMillisecondsAccumulator += 1;
    }
    
    /* Count seconds */
    if (UptimeMillisecondsAccumulator > 1000) {
        UptimeMillisecondsAccumulator = 0;
        UptimeSeconds += 1;
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
        //runRateGroup4_TBD();

    } else if (brmsTask & brmsRG5Mask) {
     
        /* Rate group 5 is run every 6.4ms, or 156Hz*/
        //runRateGroup5_TBD();
    }
    
    PROBE_Write(0);
   
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
    
    uint32_t i;
    
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
                rxMessage.buf[i] = (uint8_t) SPI_1_SpiUartReadRxData();            
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

