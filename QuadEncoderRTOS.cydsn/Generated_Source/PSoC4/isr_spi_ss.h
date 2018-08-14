/*******************************************************************************
* File Name: isr_spi_ss.h
* Version 1.70
*
*  Description:
*   Provides the function definitions for the Interrupt Controller.
*
*
********************************************************************************
* Copyright 2008-2015, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/
#if !defined(CY_ISR_isr_spi_ss_H)
#define CY_ISR_isr_spi_ss_H


#include <cytypes.h>
#include <cyfitter.h>

/* Interrupt Controller API. */
void isr_spi_ss_Start(void);
void isr_spi_ss_StartEx(cyisraddress address);
void isr_spi_ss_Stop(void);

CY_ISR_PROTO(isr_spi_ss_Interrupt);

void isr_spi_ss_SetVector(cyisraddress address);
cyisraddress isr_spi_ss_GetVector(void);

void isr_spi_ss_SetPriority(uint8 priority);
uint8 isr_spi_ss_GetPriority(void);

void isr_spi_ss_Enable(void);
uint8 isr_spi_ss_GetState(void);
void isr_spi_ss_Disable(void);

void isr_spi_ss_SetPending(void);
void isr_spi_ss_ClearPending(void);


/* Interrupt Controller Constants */

/* Address of the INTC.VECT[x] register that contains the Address of the isr_spi_ss ISR. */
#define isr_spi_ss_INTC_VECTOR            ((reg32 *) isr_spi_ss__INTC_VECT)

/* Address of the isr_spi_ss ISR priority. */
#define isr_spi_ss_INTC_PRIOR             ((reg32 *) isr_spi_ss__INTC_PRIOR_REG)

/* Priority of the isr_spi_ss interrupt. */
#define isr_spi_ss_INTC_PRIOR_NUMBER      isr_spi_ss__INTC_PRIOR_NUM

/* Address of the INTC.SET_EN[x] byte to bit enable isr_spi_ss interrupt. */
#define isr_spi_ss_INTC_SET_EN            ((reg32 *) isr_spi_ss__INTC_SET_EN_REG)

/* Address of the INTC.CLR_EN[x] register to bit clear the isr_spi_ss interrupt. */
#define isr_spi_ss_INTC_CLR_EN            ((reg32 *) isr_spi_ss__INTC_CLR_EN_REG)

/* Address of the INTC.SET_PD[x] register to set the isr_spi_ss interrupt state to pending. */
#define isr_spi_ss_INTC_SET_PD            ((reg32 *) isr_spi_ss__INTC_SET_PD_REG)

/* Address of the INTC.CLR_PD[x] register to clear the isr_spi_ss interrupt. */
#define isr_spi_ss_INTC_CLR_PD            ((reg32 *) isr_spi_ss__INTC_CLR_PD_REG)



#endif /* CY_ISR_isr_spi_ss_H */


/* [] END OF FILE */
