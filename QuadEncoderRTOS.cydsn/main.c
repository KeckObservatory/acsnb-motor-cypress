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
* 07/31/18 PMR  Rev: NC Initial Release after port from Kona Scientific code
*******************************************************************************/
#include <FreeRTOS.h>
#include <semphr.h>
#include <task.h>
#include <I2C_I2C.h>
#include "PID.h"

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


/* TI INA219 Zero-Drift, Bidirectional Current/Power Monitor With I2C Interface */
#define INA219_I2C_ADDR                        (0x40)
#define INA219_CAL_VALUE                       (8192)


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
uint8 CurrentI2Cinbuf[20];
TPIDLoop pidLoop;
TPIDConstants pidConstants;

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

/* Inbound message from BBB, currently 20 bytes long */
typedef struct {
    
    uint32 signature;  /* Comm signature for reliable messaging */ 
    uint32 setpoint;   /* Setpoint (desired actuator position, 24 bits), high byte used as PID enable */
    float P;
    float I;
    float D;
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
    
    volatile uint32 err;
    uint8 byteMSB;
    uint8 byteLSB;
    volatile uint16 CurrentTemp;
    
    /* Initial high water mark reading */
    uxHighWaterMark_Current = uxTaskGetStackHighWaterMark( NULL );
    
    while (1) {

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
                        txMessage.msg.pwm = pidLoop.CurrentOutputExtendedConfinedAsPercent;
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
* Function Name: PID_Task
********************************************************************************
* Summary:
*  RTOS task to perform the PID calculations.
*
* Parameters: None
* Return: None
*******************************************************************************/
void PID_Task(void *arg) {
    
    uint32 s = 0;
    int32 error;
    uint32 position;
    static uint32 setpoint;
    static bool pidenabled = false;
    
    /* Initial high water mark reading */
    uxHighWaterMark_PID = uxTaskGetStackHighWaterMark( NULL );
    
    /* Setup the PID data structure */
    InitializePIDLoop( &pidLoop, 1.0, 1.0, 1.0, -100.0, 100.0 );    
    InitializePIDLoopConstants( &pidLoop, pidConstants, true );
    
    /* Local copy of the setpoint; this will be updated by the rxMessage contents when they are valid */    
    setpoint = 0;
    
    while (1) {
        
        /* If the interrupt handler for the end of SPI transaction is moving data into the rxMessageBuffer right now, 
           wait for it to be done. */
        if (rxMessageLocked) {
            
            /* Hold off until the SPI is done! */
            Sleep(1);
            
        } else {
        
            /* Flash the light to indicate the task is running */
            if (++s > 100)
                s = 0;
            
            if (s > 50) {
                LED_Write(1);
            } else {
                LED_Write(0);
            }
                
            /* If the rxMessage is not corrupt, use it to update P/I/D and setpoint */
            if (rxMessage.msg.signature == COMM_SIGNATURE_WORD) {
            
                /* If the PID constants are different, update the PID loop. */
                if ((pidConstants.PGain != rxMessage.msg.P) ||
                    (pidConstants.IGain != rxMessage.msg.I) ||
                    (pidConstants.DGain != rxMessage.msg.D)) {
                
                    pidConstants.PGain = rxMessage.msg.P;
                    pidConstants.IGain = rxMessage.msg.I;
                    pidConstants.DGain = rxMessage.msg.D;
                        
                    InitializePIDLoopConstants( &pidLoop, pidConstants, true );
                    //InitializePIDLoop( &pidLoop, rxMessage.msg.P, rxMessage.msg.I, rxMessage.msg.D, -100.0, 100.0 );        
                }
                   
                pidenabled = ((rxMessage.msg.setpoint & 0xFF000000) > 0);
                setpoint = (rxMessage.msg.setpoint & 0x00FFFFFF);               
            }

            /* Run the PID algorithm */
            if (pidenabled) {
                position = Counter_1_ReadCounter();  
                error = setpoint - position;
                
                UpdatePIDLoop( &pidLoop, setpoint, position, error );   
            }

            /* Run the PID every 10ms, which is 100Hz update rate */
            Sleep(10);
        }

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
    
    // Clear the 24b Encoder (Absolute Position Counter)
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

    // Clear the 24b Encoder (Absolute Position Counter)
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
                //PROBE_Write(0);
                
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
                
                //PROBE_Write(1);              
                
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


