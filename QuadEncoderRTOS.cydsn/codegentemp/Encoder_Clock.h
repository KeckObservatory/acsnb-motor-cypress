/*******************************************************************************
* File Name: Encoder_Clock.h
* Version 2.20
*
*  Description:
*   Provides the function and constant definitions for the clock component.
*
*  Note:
*
********************************************************************************
* Copyright 2008-2012, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_CLOCK_Encoder_Clock_H)
#define CY_CLOCK_Encoder_Clock_H

#include <cytypes.h>
#include <cyfitter.h>


/***************************************
*        Function Prototypes
***************************************/
#if defined CYREG_PERI_DIV_CMD

void Encoder_Clock_StartEx(uint32 alignClkDiv);
#define Encoder_Clock_Start() \
    Encoder_Clock_StartEx(Encoder_Clock__PA_DIV_ID)

#else

void Encoder_Clock_Start(void);

#endif/* CYREG_PERI_DIV_CMD */

void Encoder_Clock_Stop(void);

void Encoder_Clock_SetFractionalDividerRegister(uint16 clkDivider, uint8 clkFractional);

uint16 Encoder_Clock_GetDividerRegister(void);
uint8  Encoder_Clock_GetFractionalDividerRegister(void);

#define Encoder_Clock_Enable()                         Encoder_Clock_Start()
#define Encoder_Clock_Disable()                        Encoder_Clock_Stop()
#define Encoder_Clock_SetDividerRegister(clkDivider, reset)  \
    Encoder_Clock_SetFractionalDividerRegister((clkDivider), 0u)
#define Encoder_Clock_SetDivider(clkDivider)           Encoder_Clock_SetDividerRegister((clkDivider), 1u)
#define Encoder_Clock_SetDividerValue(clkDivider)      Encoder_Clock_SetDividerRegister((clkDivider) - 1u, 1u)


/***************************************
*             Registers
***************************************/
#if defined CYREG_PERI_DIV_CMD

#define Encoder_Clock_DIV_ID     Encoder_Clock__DIV_ID

#define Encoder_Clock_CMD_REG    (*(reg32 *)CYREG_PERI_DIV_CMD)
#define Encoder_Clock_CTRL_REG   (*(reg32 *)Encoder_Clock__CTRL_REGISTER)
#define Encoder_Clock_DIV_REG    (*(reg32 *)Encoder_Clock__DIV_REGISTER)

#define Encoder_Clock_CMD_DIV_SHIFT          (0u)
#define Encoder_Clock_CMD_PA_DIV_SHIFT       (8u)
#define Encoder_Clock_CMD_DISABLE_SHIFT      (30u)
#define Encoder_Clock_CMD_ENABLE_SHIFT       (31u)

#define Encoder_Clock_CMD_DISABLE_MASK       ((uint32)((uint32)1u << Encoder_Clock_CMD_DISABLE_SHIFT))
#define Encoder_Clock_CMD_ENABLE_MASK        ((uint32)((uint32)1u << Encoder_Clock_CMD_ENABLE_SHIFT))

#define Encoder_Clock_DIV_FRAC_MASK  (0x000000F8u)
#define Encoder_Clock_DIV_FRAC_SHIFT (3u)
#define Encoder_Clock_DIV_INT_MASK   (0xFFFFFF00u)
#define Encoder_Clock_DIV_INT_SHIFT  (8u)

#else 

#define Encoder_Clock_DIV_REG        (*(reg32 *)Encoder_Clock__REGISTER)
#define Encoder_Clock_ENABLE_REG     Encoder_Clock_DIV_REG
#define Encoder_Clock_DIV_FRAC_MASK  Encoder_Clock__FRAC_MASK
#define Encoder_Clock_DIV_FRAC_SHIFT (16u)
#define Encoder_Clock_DIV_INT_MASK   Encoder_Clock__DIVIDER_MASK
#define Encoder_Clock_DIV_INT_SHIFT  (0u)

#endif/* CYREG_PERI_DIV_CMD */

#endif /* !defined(CY_CLOCK_Encoder_Clock_H) */

/* [] END OF FILE */
