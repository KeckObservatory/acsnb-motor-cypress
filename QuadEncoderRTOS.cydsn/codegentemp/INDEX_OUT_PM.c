/*******************************************************************************
* File Name: INDEX_OUT.c  
* Version 2.20
*
* Description:
*  This file contains APIs to set up the Pins component for low power modes.
*
* Note:
*
********************************************************************************
* Copyright 2015, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/

#include "cytypes.h"
#include "INDEX_OUT.h"

static INDEX_OUT_BACKUP_STRUCT  INDEX_OUT_backup = {0u, 0u, 0u};


/*******************************************************************************
* Function Name: INDEX_OUT_Sleep
****************************************************************************//**
*
* \brief Stores the pin configuration and prepares the pin for entering chip 
*  deep-sleep/hibernate modes. This function applies only to SIO and USBIO pins.
*  It should not be called for GPIO or GPIO_OVT pins.
*
* <b>Note</b> This function is available in PSoC 4 only.
*
* \return 
*  None 
*  
* \sideeffect
*  For SIO pins, this function configures the pin input threshold to CMOS and
*  drive level to Vddio. This is needed for SIO pins when in device 
*  deep-sleep/hibernate modes.
*
* \funcusage
*  \snippet INDEX_OUT_SUT.c usage_INDEX_OUT_Sleep_Wakeup
*******************************************************************************/
void INDEX_OUT_Sleep(void)
{
    #if defined(INDEX_OUT__PC)
        INDEX_OUT_backup.pcState = INDEX_OUT_PC;
    #else
        #if (CY_PSOC4_4200L)
            /* Save the regulator state and put the PHY into suspend mode */
            INDEX_OUT_backup.usbState = INDEX_OUT_CR1_REG;
            INDEX_OUT_USB_POWER_REG |= INDEX_OUT_USBIO_ENTER_SLEEP;
            INDEX_OUT_CR1_REG &= INDEX_OUT_USBIO_CR1_OFF;
        #endif
    #endif
    #if defined(CYIPBLOCK_m0s8ioss_VERSION) && defined(INDEX_OUT__SIO)
        INDEX_OUT_backup.sioState = INDEX_OUT_SIO_REG;
        /* SIO requires unregulated output buffer and single ended input buffer */
        INDEX_OUT_SIO_REG &= (uint32)(~INDEX_OUT_SIO_LPM_MASK);
    #endif  
}


/*******************************************************************************
* Function Name: INDEX_OUT_Wakeup
****************************************************************************//**
*
* \brief Restores the pin configuration that was saved during Pin_Sleep(). This 
* function applies only to SIO and USBIO pins. It should not be called for
* GPIO or GPIO_OVT pins.
*
* For USBIO pins, the wakeup is only triggered for falling edge interrupts.
*
* <b>Note</b> This function is available in PSoC 4 only.
*
* \return 
*  None
*  
* \funcusage
*  Refer to INDEX_OUT_Sleep() for an example usage.
*******************************************************************************/
void INDEX_OUT_Wakeup(void)
{
    #if defined(INDEX_OUT__PC)
        INDEX_OUT_PC = INDEX_OUT_backup.pcState;
    #else
        #if (CY_PSOC4_4200L)
            /* Restore the regulator state and come out of suspend mode */
            INDEX_OUT_USB_POWER_REG &= INDEX_OUT_USBIO_EXIT_SLEEP_PH1;
            INDEX_OUT_CR1_REG = INDEX_OUT_backup.usbState;
            INDEX_OUT_USB_POWER_REG &= INDEX_OUT_USBIO_EXIT_SLEEP_PH2;
        #endif
    #endif
    #if defined(CYIPBLOCK_m0s8ioss_VERSION) && defined(INDEX_OUT__SIO)
        INDEX_OUT_SIO_REG = INDEX_OUT_backup.sioState;
    #endif
}


/* [] END OF FILE */
