/*******************************************************************************
* File Name: Timer_BRMSx_PM.c
* Version 2.10
*
* Description:
*  This file contains the setup, control, and status commands to support
*  the component operations in the low power mode.
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

#include "Timer_BRMSx.h"

static Timer_BRMSx_BACKUP_STRUCT Timer_BRMSx_backup;


/*******************************************************************************
* Function Name: Timer_BRMSx_SaveConfig
********************************************************************************
*
* Summary:
*  All configuration registers are retention. Nothing to save here.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void Timer_BRMSx_SaveConfig(void)
{

}


/*******************************************************************************
* Function Name: Timer_BRMSx_Sleep
********************************************************************************
*
* Summary:
*  Stops the component operation and saves the user configuration.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void Timer_BRMSx_Sleep(void)
{
    if(0u != (Timer_BRMSx_BLOCK_CONTROL_REG & Timer_BRMSx_MASK))
    {
        Timer_BRMSx_backup.enableState = 1u;
    }
    else
    {
        Timer_BRMSx_backup.enableState = 0u;
    }

    Timer_BRMSx_Stop();
    Timer_BRMSx_SaveConfig();
}


/*******************************************************************************
* Function Name: Timer_BRMSx_RestoreConfig
********************************************************************************
*
* Summary:
*  All configuration registers are retention. Nothing to restore here.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void Timer_BRMSx_RestoreConfig(void)
{

}


/*******************************************************************************
* Function Name: Timer_BRMSx_Wakeup
********************************************************************************
*
* Summary:
*  Restores the user configuration and restores the enable state.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void Timer_BRMSx_Wakeup(void)
{
    Timer_BRMSx_RestoreConfig();

    if(0u != Timer_BRMSx_backup.enableState)
    {
        Timer_BRMSx_Enable();
    }
}


/* [] END OF FILE */
