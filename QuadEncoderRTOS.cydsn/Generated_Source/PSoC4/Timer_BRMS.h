/*******************************************************************************
* File Name: Timer_BRMS.h
* Version 2.10
*
* Description:
*  This file provides constants and parameter values for the Timer_BRMS
*  component.
*
* Note:
*  None
*
********************************************************************************
* Copyright 2013-2015, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_TCPWM_Timer_BRMS_H)
#define CY_TCPWM_Timer_BRMS_H


#include "CyLib.h"
#include "cytypes.h"
#include "cyfitter.h"


/*******************************************************************************
* Internal Type defines
*******************************************************************************/

/* Structure to save state before go to sleep */
typedef struct
{
    uint8  enableState;
} Timer_BRMS_BACKUP_STRUCT;


/*******************************************************************************
* Variables
*******************************************************************************/
extern uint8  Timer_BRMS_initVar;


/***************************************
*   Conditional Compilation Parameters
****************************************/

#define Timer_BRMS_CY_TCPWM_V2                    (CYIPBLOCK_m0s8tcpwm_VERSION == 2u)
#define Timer_BRMS_CY_TCPWM_4000                  (CY_PSOC4_4000)

/* TCPWM Configuration */
#define Timer_BRMS_CONFIG                         (1lu)

/* Quad Mode */
/* Parameters */
#define Timer_BRMS_QUAD_ENCODING_MODES            (0lu)
#define Timer_BRMS_QUAD_AUTO_START                (1lu)

/* Signal modes */
#define Timer_BRMS_QUAD_INDEX_SIGNAL_MODE         (0lu)
#define Timer_BRMS_QUAD_PHIA_SIGNAL_MODE          (3lu)
#define Timer_BRMS_QUAD_PHIB_SIGNAL_MODE          (3lu)
#define Timer_BRMS_QUAD_STOP_SIGNAL_MODE          (0lu)

/* Signal present */
#define Timer_BRMS_QUAD_INDEX_SIGNAL_PRESENT      (0lu)
#define Timer_BRMS_QUAD_STOP_SIGNAL_PRESENT       (0lu)

/* Interrupt Mask */
#define Timer_BRMS_QUAD_INTERRUPT_MASK            (1lu)

/* Timer/Counter Mode */
/* Parameters */
#define Timer_BRMS_TC_RUN_MODE                    (0lu)
#define Timer_BRMS_TC_COUNTER_MODE                (0lu)
#define Timer_BRMS_TC_COMP_CAP_MODE               (0lu)
#define Timer_BRMS_TC_PRESCALER                   (0lu)

/* Signal modes */
#define Timer_BRMS_TC_RELOAD_SIGNAL_MODE          (0lu)
#define Timer_BRMS_TC_COUNT_SIGNAL_MODE           (1lu)
#define Timer_BRMS_TC_START_SIGNAL_MODE           (0lu)
#define Timer_BRMS_TC_STOP_SIGNAL_MODE            (0lu)
#define Timer_BRMS_TC_CAPTURE_SIGNAL_MODE         (0lu)

/* Signal present */
#define Timer_BRMS_TC_RELOAD_SIGNAL_PRESENT       (0lu)
#define Timer_BRMS_TC_COUNT_SIGNAL_PRESENT        (0lu)
#define Timer_BRMS_TC_START_SIGNAL_PRESENT        (0lu)
#define Timer_BRMS_TC_STOP_SIGNAL_PRESENT         (0lu)
#define Timer_BRMS_TC_CAPTURE_SIGNAL_PRESENT      (0lu)

/* Interrupt Mask */
#define Timer_BRMS_TC_INTERRUPT_MASK              (2lu)

/* PWM Mode */
/* Parameters */
#define Timer_BRMS_PWM_KILL_EVENT                 (0lu)
#define Timer_BRMS_PWM_STOP_EVENT                 (0lu)
#define Timer_BRMS_PWM_MODE                       (4lu)
#define Timer_BRMS_PWM_OUT_N_INVERT               (0lu)
#define Timer_BRMS_PWM_OUT_INVERT                 (0lu)
#define Timer_BRMS_PWM_ALIGN                      (0lu)
#define Timer_BRMS_PWM_RUN_MODE                   (0lu)
#define Timer_BRMS_PWM_DEAD_TIME_CYCLE            (0lu)
#define Timer_BRMS_PWM_PRESCALER                  (0lu)

/* Signal modes */
#define Timer_BRMS_PWM_RELOAD_SIGNAL_MODE         (0lu)
#define Timer_BRMS_PWM_COUNT_SIGNAL_MODE          (3lu)
#define Timer_BRMS_PWM_START_SIGNAL_MODE          (0lu)
#define Timer_BRMS_PWM_STOP_SIGNAL_MODE           (0lu)
#define Timer_BRMS_PWM_SWITCH_SIGNAL_MODE         (0lu)

/* Signal present */
#define Timer_BRMS_PWM_RELOAD_SIGNAL_PRESENT      (0lu)
#define Timer_BRMS_PWM_COUNT_SIGNAL_PRESENT       (0lu)
#define Timer_BRMS_PWM_START_SIGNAL_PRESENT       (0lu)
#define Timer_BRMS_PWM_STOP_SIGNAL_PRESENT        (0lu)
#define Timer_BRMS_PWM_SWITCH_SIGNAL_PRESENT      (0lu)

/* Interrupt Mask */
#define Timer_BRMS_PWM_INTERRUPT_MASK             (1lu)


/***************************************
*    Initial Parameter Constants
***************************************/

/* Timer/Counter Mode */
#define Timer_BRMS_TC_PERIOD_VALUE                (1lu)
#define Timer_BRMS_TC_COMPARE_VALUE               (0lu)
#define Timer_BRMS_TC_COMPARE_BUF_VALUE           (65535lu)
#define Timer_BRMS_TC_COMPARE_SWAP                (0lu)

/* PWM Mode */
#define Timer_BRMS_PWM_PERIOD_VALUE               (65535lu)
#define Timer_BRMS_PWM_PERIOD_BUF_VALUE           (65535lu)
#define Timer_BRMS_PWM_PERIOD_SWAP                (0lu)
#define Timer_BRMS_PWM_COMPARE_VALUE              (65535lu)
#define Timer_BRMS_PWM_COMPARE_BUF_VALUE          (65535lu)
#define Timer_BRMS_PWM_COMPARE_SWAP               (0lu)


/***************************************
*    Enumerated Types and Parameters
***************************************/

#define Timer_BRMS__LEFT 0
#define Timer_BRMS__RIGHT 1
#define Timer_BRMS__CENTER 2
#define Timer_BRMS__ASYMMETRIC 3

#define Timer_BRMS__X1 0
#define Timer_BRMS__X2 1
#define Timer_BRMS__X4 2

#define Timer_BRMS__PWM 4
#define Timer_BRMS__PWM_DT 5
#define Timer_BRMS__PWM_PR 6

#define Timer_BRMS__INVERSE 1
#define Timer_BRMS__DIRECT 0

#define Timer_BRMS__CAPTURE 2
#define Timer_BRMS__COMPARE 0

#define Timer_BRMS__TRIG_LEVEL 3
#define Timer_BRMS__TRIG_RISING 0
#define Timer_BRMS__TRIG_FALLING 1
#define Timer_BRMS__TRIG_BOTH 2

#define Timer_BRMS__INTR_MASK_TC 1
#define Timer_BRMS__INTR_MASK_CC_MATCH 2
#define Timer_BRMS__INTR_MASK_NONE 0
#define Timer_BRMS__INTR_MASK_TC_CC 3

#define Timer_BRMS__UNCONFIG 8
#define Timer_BRMS__TIMER 1
#define Timer_BRMS__QUAD 3
#define Timer_BRMS__PWM_SEL 7

#define Timer_BRMS__COUNT_UP 0
#define Timer_BRMS__COUNT_DOWN 1
#define Timer_BRMS__COUNT_UPDOWN0 2
#define Timer_BRMS__COUNT_UPDOWN1 3


/* Prescaler */
#define Timer_BRMS_PRESCALE_DIVBY1                ((uint32)(0u << Timer_BRMS_PRESCALER_SHIFT))
#define Timer_BRMS_PRESCALE_DIVBY2                ((uint32)(1u << Timer_BRMS_PRESCALER_SHIFT))
#define Timer_BRMS_PRESCALE_DIVBY4                ((uint32)(2u << Timer_BRMS_PRESCALER_SHIFT))
#define Timer_BRMS_PRESCALE_DIVBY8                ((uint32)(3u << Timer_BRMS_PRESCALER_SHIFT))
#define Timer_BRMS_PRESCALE_DIVBY16               ((uint32)(4u << Timer_BRMS_PRESCALER_SHIFT))
#define Timer_BRMS_PRESCALE_DIVBY32               ((uint32)(5u << Timer_BRMS_PRESCALER_SHIFT))
#define Timer_BRMS_PRESCALE_DIVBY64               ((uint32)(6u << Timer_BRMS_PRESCALER_SHIFT))
#define Timer_BRMS_PRESCALE_DIVBY128              ((uint32)(7u << Timer_BRMS_PRESCALER_SHIFT))

/* TCPWM set modes */
#define Timer_BRMS_MODE_TIMER_COMPARE             ((uint32)(Timer_BRMS__COMPARE         <<  \
                                                                  Timer_BRMS_MODE_SHIFT))
#define Timer_BRMS_MODE_TIMER_CAPTURE             ((uint32)(Timer_BRMS__CAPTURE         <<  \
                                                                  Timer_BRMS_MODE_SHIFT))
#define Timer_BRMS_MODE_QUAD                      ((uint32)(Timer_BRMS__QUAD            <<  \
                                                                  Timer_BRMS_MODE_SHIFT))
#define Timer_BRMS_MODE_PWM                       ((uint32)(Timer_BRMS__PWM             <<  \
                                                                  Timer_BRMS_MODE_SHIFT))
#define Timer_BRMS_MODE_PWM_DT                    ((uint32)(Timer_BRMS__PWM_DT          <<  \
                                                                  Timer_BRMS_MODE_SHIFT))
#define Timer_BRMS_MODE_PWM_PR                    ((uint32)(Timer_BRMS__PWM_PR          <<  \
                                                                  Timer_BRMS_MODE_SHIFT))

/* Quad Modes */
#define Timer_BRMS_MODE_X1                        ((uint32)(Timer_BRMS__X1              <<  \
                                                                  Timer_BRMS_QUAD_MODE_SHIFT))
#define Timer_BRMS_MODE_X2                        ((uint32)(Timer_BRMS__X2              <<  \
                                                                  Timer_BRMS_QUAD_MODE_SHIFT))
#define Timer_BRMS_MODE_X4                        ((uint32)(Timer_BRMS__X4              <<  \
                                                                  Timer_BRMS_QUAD_MODE_SHIFT))

/* Counter modes */
#define Timer_BRMS_COUNT_UP                       ((uint32)(Timer_BRMS__COUNT_UP        <<  \
                                                                  Timer_BRMS_UPDOWN_SHIFT))
#define Timer_BRMS_COUNT_DOWN                     ((uint32)(Timer_BRMS__COUNT_DOWN      <<  \
                                                                  Timer_BRMS_UPDOWN_SHIFT))
#define Timer_BRMS_COUNT_UPDOWN0                  ((uint32)(Timer_BRMS__COUNT_UPDOWN0   <<  \
                                                                  Timer_BRMS_UPDOWN_SHIFT))
#define Timer_BRMS_COUNT_UPDOWN1                  ((uint32)(Timer_BRMS__COUNT_UPDOWN1   <<  \
                                                                  Timer_BRMS_UPDOWN_SHIFT))

/* PWM output invert */
#define Timer_BRMS_INVERT_LINE                    ((uint32)(Timer_BRMS__INVERSE         <<  \
                                                                  Timer_BRMS_INV_OUT_SHIFT))
#define Timer_BRMS_INVERT_LINE_N                  ((uint32)(Timer_BRMS__INVERSE         <<  \
                                                                  Timer_BRMS_INV_COMPL_OUT_SHIFT))

/* Trigger modes */
#define Timer_BRMS_TRIG_RISING                    ((uint32)Timer_BRMS__TRIG_RISING)
#define Timer_BRMS_TRIG_FALLING                   ((uint32)Timer_BRMS__TRIG_FALLING)
#define Timer_BRMS_TRIG_BOTH                      ((uint32)Timer_BRMS__TRIG_BOTH)
#define Timer_BRMS_TRIG_LEVEL                     ((uint32)Timer_BRMS__TRIG_LEVEL)

/* Interrupt mask */
#define Timer_BRMS_INTR_MASK_TC                   ((uint32)Timer_BRMS__INTR_MASK_TC)
#define Timer_BRMS_INTR_MASK_CC_MATCH             ((uint32)Timer_BRMS__INTR_MASK_CC_MATCH)

/* PWM Output Controls */
#define Timer_BRMS_CC_MATCH_SET                   (0x00u)
#define Timer_BRMS_CC_MATCH_CLEAR                 (0x01u)
#define Timer_BRMS_CC_MATCH_INVERT                (0x02u)
#define Timer_BRMS_CC_MATCH_NO_CHANGE             (0x03u)
#define Timer_BRMS_OVERLOW_SET                    (0x00u)
#define Timer_BRMS_OVERLOW_CLEAR                  (0x04u)
#define Timer_BRMS_OVERLOW_INVERT                 (0x08u)
#define Timer_BRMS_OVERLOW_NO_CHANGE              (0x0Cu)
#define Timer_BRMS_UNDERFLOW_SET                  (0x00u)
#define Timer_BRMS_UNDERFLOW_CLEAR                (0x10u)
#define Timer_BRMS_UNDERFLOW_INVERT               (0x20u)
#define Timer_BRMS_UNDERFLOW_NO_CHANGE            (0x30u)

/* PWM Align */
#define Timer_BRMS_PWM_MODE_LEFT                  (Timer_BRMS_CC_MATCH_CLEAR        |   \
                                                         Timer_BRMS_OVERLOW_SET           |   \
                                                         Timer_BRMS_UNDERFLOW_NO_CHANGE)
#define Timer_BRMS_PWM_MODE_RIGHT                 (Timer_BRMS_CC_MATCH_SET          |   \
                                                         Timer_BRMS_OVERLOW_NO_CHANGE     |   \
                                                         Timer_BRMS_UNDERFLOW_CLEAR)
#define Timer_BRMS_PWM_MODE_ASYM                  (Timer_BRMS_CC_MATCH_INVERT       |   \
                                                         Timer_BRMS_OVERLOW_SET           |   \
                                                         Timer_BRMS_UNDERFLOW_CLEAR)

#if (Timer_BRMS_CY_TCPWM_V2)
    #if(Timer_BRMS_CY_TCPWM_4000)
        #define Timer_BRMS_PWM_MODE_CENTER                (Timer_BRMS_CC_MATCH_INVERT       |   \
                                                                 Timer_BRMS_OVERLOW_NO_CHANGE     |   \
                                                                 Timer_BRMS_UNDERFLOW_CLEAR)
    #else
        #define Timer_BRMS_PWM_MODE_CENTER                (Timer_BRMS_CC_MATCH_INVERT       |   \
                                                                 Timer_BRMS_OVERLOW_SET           |   \
                                                                 Timer_BRMS_UNDERFLOW_CLEAR)
    #endif /* (Timer_BRMS_CY_TCPWM_4000) */
#else
    #define Timer_BRMS_PWM_MODE_CENTER                (Timer_BRMS_CC_MATCH_INVERT       |   \
                                                             Timer_BRMS_OVERLOW_NO_CHANGE     |   \
                                                             Timer_BRMS_UNDERFLOW_CLEAR)
#endif /* (Timer_BRMS_CY_TCPWM_NEW) */

/* Command operations without condition */
#define Timer_BRMS_CMD_CAPTURE                    (0u)
#define Timer_BRMS_CMD_RELOAD                     (8u)
#define Timer_BRMS_CMD_STOP                       (16u)
#define Timer_BRMS_CMD_START                      (24u)

/* Status */
#define Timer_BRMS_STATUS_DOWN                    (1u)
#define Timer_BRMS_STATUS_RUNNING                 (2u)


/***************************************
*        Function Prototypes
****************************************/

void   Timer_BRMS_Init(void);
void   Timer_BRMS_Enable(void);
void   Timer_BRMS_Start(void);
void   Timer_BRMS_Stop(void);

void   Timer_BRMS_SetMode(uint32 mode);
void   Timer_BRMS_SetCounterMode(uint32 counterMode);
void   Timer_BRMS_SetPWMMode(uint32 modeMask);
void   Timer_BRMS_SetQDMode(uint32 qdMode);

void   Timer_BRMS_SetPrescaler(uint32 prescaler);
void   Timer_BRMS_TriggerCommand(uint32 mask, uint32 command);
void   Timer_BRMS_SetOneShot(uint32 oneShotEnable);
uint32 Timer_BRMS_ReadStatus(void);

void   Timer_BRMS_SetPWMSyncKill(uint32 syncKillEnable);
void   Timer_BRMS_SetPWMStopOnKill(uint32 stopOnKillEnable);
void   Timer_BRMS_SetPWMDeadTime(uint32 deadTime);
void   Timer_BRMS_SetPWMInvert(uint32 mask);

void   Timer_BRMS_SetInterruptMode(uint32 interruptMask);
uint32 Timer_BRMS_GetInterruptSourceMasked(void);
uint32 Timer_BRMS_GetInterruptSource(void);
void   Timer_BRMS_ClearInterrupt(uint32 interruptMask);
void   Timer_BRMS_SetInterrupt(uint32 interruptMask);

void   Timer_BRMS_WriteCounter(uint32 count);
uint32 Timer_BRMS_ReadCounter(void);

uint32 Timer_BRMS_ReadCapture(void);
uint32 Timer_BRMS_ReadCaptureBuf(void);

void   Timer_BRMS_WritePeriod(uint32 period);
uint32 Timer_BRMS_ReadPeriod(void);
void   Timer_BRMS_WritePeriodBuf(uint32 periodBuf);
uint32 Timer_BRMS_ReadPeriodBuf(void);

void   Timer_BRMS_WriteCompare(uint32 compare);
uint32 Timer_BRMS_ReadCompare(void);
void   Timer_BRMS_WriteCompareBuf(uint32 compareBuf);
uint32 Timer_BRMS_ReadCompareBuf(void);

void   Timer_BRMS_SetPeriodSwap(uint32 swapEnable);
void   Timer_BRMS_SetCompareSwap(uint32 swapEnable);

void   Timer_BRMS_SetCaptureMode(uint32 triggerMode);
void   Timer_BRMS_SetReloadMode(uint32 triggerMode);
void   Timer_BRMS_SetStartMode(uint32 triggerMode);
void   Timer_BRMS_SetStopMode(uint32 triggerMode);
void   Timer_BRMS_SetCountMode(uint32 triggerMode);

void   Timer_BRMS_SaveConfig(void);
void   Timer_BRMS_RestoreConfig(void);
void   Timer_BRMS_Sleep(void);
void   Timer_BRMS_Wakeup(void);


/***************************************
*             Registers
***************************************/

#define Timer_BRMS_BLOCK_CONTROL_REG              (*(reg32 *) Timer_BRMS_cy_m0s8_tcpwm_1__TCPWM_CTRL )
#define Timer_BRMS_BLOCK_CONTROL_PTR              ( (reg32 *) Timer_BRMS_cy_m0s8_tcpwm_1__TCPWM_CTRL )
#define Timer_BRMS_COMMAND_REG                    (*(reg32 *) Timer_BRMS_cy_m0s8_tcpwm_1__TCPWM_CMD )
#define Timer_BRMS_COMMAND_PTR                    ( (reg32 *) Timer_BRMS_cy_m0s8_tcpwm_1__TCPWM_CMD )
#define Timer_BRMS_INTRRUPT_CAUSE_REG             (*(reg32 *) Timer_BRMS_cy_m0s8_tcpwm_1__TCPWM_INTR_CAUSE )
#define Timer_BRMS_INTRRUPT_CAUSE_PTR             ( (reg32 *) Timer_BRMS_cy_m0s8_tcpwm_1__TCPWM_INTR_CAUSE )
#define Timer_BRMS_CONTROL_REG                    (*(reg32 *) Timer_BRMS_cy_m0s8_tcpwm_1__CTRL )
#define Timer_BRMS_CONTROL_PTR                    ( (reg32 *) Timer_BRMS_cy_m0s8_tcpwm_1__CTRL )
#define Timer_BRMS_STATUS_REG                     (*(reg32 *) Timer_BRMS_cy_m0s8_tcpwm_1__STATUS )
#define Timer_BRMS_STATUS_PTR                     ( (reg32 *) Timer_BRMS_cy_m0s8_tcpwm_1__STATUS )
#define Timer_BRMS_COUNTER_REG                    (*(reg32 *) Timer_BRMS_cy_m0s8_tcpwm_1__COUNTER )
#define Timer_BRMS_COUNTER_PTR                    ( (reg32 *) Timer_BRMS_cy_m0s8_tcpwm_1__COUNTER )
#define Timer_BRMS_COMP_CAP_REG                   (*(reg32 *) Timer_BRMS_cy_m0s8_tcpwm_1__CC )
#define Timer_BRMS_COMP_CAP_PTR                   ( (reg32 *) Timer_BRMS_cy_m0s8_tcpwm_1__CC )
#define Timer_BRMS_COMP_CAP_BUF_REG               (*(reg32 *) Timer_BRMS_cy_m0s8_tcpwm_1__CC_BUFF )
#define Timer_BRMS_COMP_CAP_BUF_PTR               ( (reg32 *) Timer_BRMS_cy_m0s8_tcpwm_1__CC_BUFF )
#define Timer_BRMS_PERIOD_REG                     (*(reg32 *) Timer_BRMS_cy_m0s8_tcpwm_1__PERIOD )
#define Timer_BRMS_PERIOD_PTR                     ( (reg32 *) Timer_BRMS_cy_m0s8_tcpwm_1__PERIOD )
#define Timer_BRMS_PERIOD_BUF_REG                 (*(reg32 *) Timer_BRMS_cy_m0s8_tcpwm_1__PERIOD_BUFF )
#define Timer_BRMS_PERIOD_BUF_PTR                 ( (reg32 *) Timer_BRMS_cy_m0s8_tcpwm_1__PERIOD_BUFF )
#define Timer_BRMS_TRIG_CONTROL0_REG              (*(reg32 *) Timer_BRMS_cy_m0s8_tcpwm_1__TR_CTRL0 )
#define Timer_BRMS_TRIG_CONTROL0_PTR              ( (reg32 *) Timer_BRMS_cy_m0s8_tcpwm_1__TR_CTRL0 )
#define Timer_BRMS_TRIG_CONTROL1_REG              (*(reg32 *) Timer_BRMS_cy_m0s8_tcpwm_1__TR_CTRL1 )
#define Timer_BRMS_TRIG_CONTROL1_PTR              ( (reg32 *) Timer_BRMS_cy_m0s8_tcpwm_1__TR_CTRL1 )
#define Timer_BRMS_TRIG_CONTROL2_REG              (*(reg32 *) Timer_BRMS_cy_m0s8_tcpwm_1__TR_CTRL2 )
#define Timer_BRMS_TRIG_CONTROL2_PTR              ( (reg32 *) Timer_BRMS_cy_m0s8_tcpwm_1__TR_CTRL2 )
#define Timer_BRMS_INTERRUPT_REQ_REG              (*(reg32 *) Timer_BRMS_cy_m0s8_tcpwm_1__INTR )
#define Timer_BRMS_INTERRUPT_REQ_PTR              ( (reg32 *) Timer_BRMS_cy_m0s8_tcpwm_1__INTR )
#define Timer_BRMS_INTERRUPT_SET_REG              (*(reg32 *) Timer_BRMS_cy_m0s8_tcpwm_1__INTR_SET )
#define Timer_BRMS_INTERRUPT_SET_PTR              ( (reg32 *) Timer_BRMS_cy_m0s8_tcpwm_1__INTR_SET )
#define Timer_BRMS_INTERRUPT_MASK_REG             (*(reg32 *) Timer_BRMS_cy_m0s8_tcpwm_1__INTR_MASK )
#define Timer_BRMS_INTERRUPT_MASK_PTR             ( (reg32 *) Timer_BRMS_cy_m0s8_tcpwm_1__INTR_MASK )
#define Timer_BRMS_INTERRUPT_MASKED_REG           (*(reg32 *) Timer_BRMS_cy_m0s8_tcpwm_1__INTR_MASKED )
#define Timer_BRMS_INTERRUPT_MASKED_PTR           ( (reg32 *) Timer_BRMS_cy_m0s8_tcpwm_1__INTR_MASKED )


/***************************************
*       Registers Constants
***************************************/

/* Mask */
#define Timer_BRMS_MASK                           ((uint32)Timer_BRMS_cy_m0s8_tcpwm_1__TCPWM_CTRL_MASK)

/* Shift constants for control register */
#define Timer_BRMS_RELOAD_CC_SHIFT                (0u)
#define Timer_BRMS_RELOAD_PERIOD_SHIFT            (1u)
#define Timer_BRMS_PWM_SYNC_KILL_SHIFT            (2u)
#define Timer_BRMS_PWM_STOP_KILL_SHIFT            (3u)
#define Timer_BRMS_PRESCALER_SHIFT                (8u)
#define Timer_BRMS_UPDOWN_SHIFT                   (16u)
#define Timer_BRMS_ONESHOT_SHIFT                  (18u)
#define Timer_BRMS_QUAD_MODE_SHIFT                (20u)
#define Timer_BRMS_INV_OUT_SHIFT                  (20u)
#define Timer_BRMS_INV_COMPL_OUT_SHIFT            (21u)
#define Timer_BRMS_MODE_SHIFT                     (24u)

/* Mask constants for control register */
#define Timer_BRMS_RELOAD_CC_MASK                 ((uint32)(Timer_BRMS_1BIT_MASK        <<  \
                                                                            Timer_BRMS_RELOAD_CC_SHIFT))
#define Timer_BRMS_RELOAD_PERIOD_MASK             ((uint32)(Timer_BRMS_1BIT_MASK        <<  \
                                                                            Timer_BRMS_RELOAD_PERIOD_SHIFT))
#define Timer_BRMS_PWM_SYNC_KILL_MASK             ((uint32)(Timer_BRMS_1BIT_MASK        <<  \
                                                                            Timer_BRMS_PWM_SYNC_KILL_SHIFT))
#define Timer_BRMS_PWM_STOP_KILL_MASK             ((uint32)(Timer_BRMS_1BIT_MASK        <<  \
                                                                            Timer_BRMS_PWM_STOP_KILL_SHIFT))
#define Timer_BRMS_PRESCALER_MASK                 ((uint32)(Timer_BRMS_8BIT_MASK        <<  \
                                                                            Timer_BRMS_PRESCALER_SHIFT))
#define Timer_BRMS_UPDOWN_MASK                    ((uint32)(Timer_BRMS_2BIT_MASK        <<  \
                                                                            Timer_BRMS_UPDOWN_SHIFT))
#define Timer_BRMS_ONESHOT_MASK                   ((uint32)(Timer_BRMS_1BIT_MASK        <<  \
                                                                            Timer_BRMS_ONESHOT_SHIFT))
#define Timer_BRMS_QUAD_MODE_MASK                 ((uint32)(Timer_BRMS_3BIT_MASK        <<  \
                                                                            Timer_BRMS_QUAD_MODE_SHIFT))
#define Timer_BRMS_INV_OUT_MASK                   ((uint32)(Timer_BRMS_2BIT_MASK        <<  \
                                                                            Timer_BRMS_INV_OUT_SHIFT))
#define Timer_BRMS_MODE_MASK                      ((uint32)(Timer_BRMS_3BIT_MASK        <<  \
                                                                            Timer_BRMS_MODE_SHIFT))

/* Shift constants for trigger control register 1 */
#define Timer_BRMS_CAPTURE_SHIFT                  (0u)
#define Timer_BRMS_COUNT_SHIFT                    (2u)
#define Timer_BRMS_RELOAD_SHIFT                   (4u)
#define Timer_BRMS_STOP_SHIFT                     (6u)
#define Timer_BRMS_START_SHIFT                    (8u)

/* Mask constants for trigger control register 1 */
#define Timer_BRMS_CAPTURE_MASK                   ((uint32)(Timer_BRMS_2BIT_MASK        <<  \
                                                                  Timer_BRMS_CAPTURE_SHIFT))
#define Timer_BRMS_COUNT_MASK                     ((uint32)(Timer_BRMS_2BIT_MASK        <<  \
                                                                  Timer_BRMS_COUNT_SHIFT))
#define Timer_BRMS_RELOAD_MASK                    ((uint32)(Timer_BRMS_2BIT_MASK        <<  \
                                                                  Timer_BRMS_RELOAD_SHIFT))
#define Timer_BRMS_STOP_MASK                      ((uint32)(Timer_BRMS_2BIT_MASK        <<  \
                                                                  Timer_BRMS_STOP_SHIFT))
#define Timer_BRMS_START_MASK                     ((uint32)(Timer_BRMS_2BIT_MASK        <<  \
                                                                  Timer_BRMS_START_SHIFT))

/* MASK */
#define Timer_BRMS_1BIT_MASK                      ((uint32)0x01u)
#define Timer_BRMS_2BIT_MASK                      ((uint32)0x03u)
#define Timer_BRMS_3BIT_MASK                      ((uint32)0x07u)
#define Timer_BRMS_6BIT_MASK                      ((uint32)0x3Fu)
#define Timer_BRMS_8BIT_MASK                      ((uint32)0xFFu)
#define Timer_BRMS_16BIT_MASK                     ((uint32)0xFFFFu)

/* Shift constant for status register */
#define Timer_BRMS_RUNNING_STATUS_SHIFT           (30u)


/***************************************
*    Initial Constants
***************************************/

#define Timer_BRMS_CTRL_QUAD_BASE_CONFIG                                                          \
        (((uint32)(Timer_BRMS_QUAD_ENCODING_MODES     << Timer_BRMS_QUAD_MODE_SHIFT))       |\
         ((uint32)(Timer_BRMS_CONFIG                  << Timer_BRMS_MODE_SHIFT)))

#define Timer_BRMS_CTRL_PWM_BASE_CONFIG                                                           \
        (((uint32)(Timer_BRMS_PWM_STOP_EVENT          << Timer_BRMS_PWM_STOP_KILL_SHIFT))   |\
         ((uint32)(Timer_BRMS_PWM_OUT_INVERT          << Timer_BRMS_INV_OUT_SHIFT))         |\
         ((uint32)(Timer_BRMS_PWM_OUT_N_INVERT        << Timer_BRMS_INV_COMPL_OUT_SHIFT))   |\
         ((uint32)(Timer_BRMS_PWM_MODE                << Timer_BRMS_MODE_SHIFT)))

#define Timer_BRMS_CTRL_PWM_RUN_MODE                                                              \
            ((uint32)(Timer_BRMS_PWM_RUN_MODE         << Timer_BRMS_ONESHOT_SHIFT))
            
#define Timer_BRMS_CTRL_PWM_ALIGN                                                                 \
            ((uint32)(Timer_BRMS_PWM_ALIGN            << Timer_BRMS_UPDOWN_SHIFT))

#define Timer_BRMS_CTRL_PWM_KILL_EVENT                                                            \
             ((uint32)(Timer_BRMS_PWM_KILL_EVENT      << Timer_BRMS_PWM_SYNC_KILL_SHIFT))

#define Timer_BRMS_CTRL_PWM_DEAD_TIME_CYCLE                                                       \
            ((uint32)(Timer_BRMS_PWM_DEAD_TIME_CYCLE  << Timer_BRMS_PRESCALER_SHIFT))

#define Timer_BRMS_CTRL_PWM_PRESCALER                                                             \
            ((uint32)(Timer_BRMS_PWM_PRESCALER        << Timer_BRMS_PRESCALER_SHIFT))

#define Timer_BRMS_CTRL_TIMER_BASE_CONFIG                                                         \
        (((uint32)(Timer_BRMS_TC_PRESCALER            << Timer_BRMS_PRESCALER_SHIFT))       |\
         ((uint32)(Timer_BRMS_TC_COUNTER_MODE         << Timer_BRMS_UPDOWN_SHIFT))          |\
         ((uint32)(Timer_BRMS_TC_RUN_MODE             << Timer_BRMS_ONESHOT_SHIFT))         |\
         ((uint32)(Timer_BRMS_TC_COMP_CAP_MODE        << Timer_BRMS_MODE_SHIFT)))
        
#define Timer_BRMS_QUAD_SIGNALS_MODES                                                             \
        (((uint32)(Timer_BRMS_QUAD_PHIA_SIGNAL_MODE   << Timer_BRMS_COUNT_SHIFT))           |\
         ((uint32)(Timer_BRMS_QUAD_INDEX_SIGNAL_MODE  << Timer_BRMS_RELOAD_SHIFT))          |\
         ((uint32)(Timer_BRMS_QUAD_STOP_SIGNAL_MODE   << Timer_BRMS_STOP_SHIFT))            |\
         ((uint32)(Timer_BRMS_QUAD_PHIB_SIGNAL_MODE   << Timer_BRMS_START_SHIFT)))

#define Timer_BRMS_PWM_SIGNALS_MODES                                                              \
        (((uint32)(Timer_BRMS_PWM_SWITCH_SIGNAL_MODE  << Timer_BRMS_CAPTURE_SHIFT))         |\
         ((uint32)(Timer_BRMS_PWM_COUNT_SIGNAL_MODE   << Timer_BRMS_COUNT_SHIFT))           |\
         ((uint32)(Timer_BRMS_PWM_RELOAD_SIGNAL_MODE  << Timer_BRMS_RELOAD_SHIFT))          |\
         ((uint32)(Timer_BRMS_PWM_STOP_SIGNAL_MODE    << Timer_BRMS_STOP_SHIFT))            |\
         ((uint32)(Timer_BRMS_PWM_START_SIGNAL_MODE   << Timer_BRMS_START_SHIFT)))

#define Timer_BRMS_TIMER_SIGNALS_MODES                                                            \
        (((uint32)(Timer_BRMS_TC_CAPTURE_SIGNAL_MODE  << Timer_BRMS_CAPTURE_SHIFT))         |\
         ((uint32)(Timer_BRMS_TC_COUNT_SIGNAL_MODE    << Timer_BRMS_COUNT_SHIFT))           |\
         ((uint32)(Timer_BRMS_TC_RELOAD_SIGNAL_MODE   << Timer_BRMS_RELOAD_SHIFT))          |\
         ((uint32)(Timer_BRMS_TC_STOP_SIGNAL_MODE     << Timer_BRMS_STOP_SHIFT))            |\
         ((uint32)(Timer_BRMS_TC_START_SIGNAL_MODE    << Timer_BRMS_START_SHIFT)))
        
#define Timer_BRMS_TIMER_UPDOWN_CNT_USED                                                          \
                ((Timer_BRMS__COUNT_UPDOWN0 == Timer_BRMS_TC_COUNTER_MODE)                  ||\
                 (Timer_BRMS__COUNT_UPDOWN1 == Timer_BRMS_TC_COUNTER_MODE))

#define Timer_BRMS_PWM_UPDOWN_CNT_USED                                                            \
                ((Timer_BRMS__CENTER == Timer_BRMS_PWM_ALIGN)                               ||\
                 (Timer_BRMS__ASYMMETRIC == Timer_BRMS_PWM_ALIGN))               
        
#define Timer_BRMS_PWM_PR_INIT_VALUE              (1u)
#define Timer_BRMS_QUAD_PERIOD_INIT_VALUE         (0x8000u)



#endif /* End CY_TCPWM_Timer_BRMS_H */

/* [] END OF FILE */
