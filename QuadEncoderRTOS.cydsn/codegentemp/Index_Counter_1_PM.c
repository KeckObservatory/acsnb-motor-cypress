/*******************************************************************************
* File Name: Index_Counter_1_PM.c  
* Version 3.0
*
*  Description:
*    This file provides the power management source code to API for the
*    Counter.  
*
*   Note:
*     None
*
********************************************************************************
* Copyright 2008-2012, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/

#include "Index_Counter_1.h"

static Index_Counter_1_backupStruct Index_Counter_1_backup;


/*******************************************************************************
* Function Name: Index_Counter_1_SaveConfig
********************************************************************************
* Summary:
*     Save the current user configuration
*
* Parameters:  
*  void
*
* Return: 
*  void
*
* Global variables:
*  Index_Counter_1_backup:  Variables of this global structure are modified to 
*  store the values of non retention configuration registers when Sleep() API is 
*  called.
*
*******************************************************************************/
void Index_Counter_1_SaveConfig(void) 
{
    #if (!Index_Counter_1_UsingFixedFunction)

        Index_Counter_1_backup.CounterUdb = Index_Counter_1_ReadCounter();

        #if(!Index_Counter_1_ControlRegRemoved)
            Index_Counter_1_backup.CounterControlRegister = Index_Counter_1_ReadControlRegister();
        #endif /* (!Index_Counter_1_ControlRegRemoved) */

    #endif /* (!Index_Counter_1_UsingFixedFunction) */
}


/*******************************************************************************
* Function Name: Index_Counter_1_RestoreConfig
********************************************************************************
*
* Summary:
*  Restores the current user configuration.
*
* Parameters:  
*  void
*
* Return: 
*  void
*
* Global variables:
*  Index_Counter_1_backup:  Variables of this global structure are used to 
*  restore the values of non retention registers on wakeup from sleep mode.
*
*******************************************************************************/
void Index_Counter_1_RestoreConfig(void) 
{      
    #if (!Index_Counter_1_UsingFixedFunction)

       Index_Counter_1_WriteCounter(Index_Counter_1_backup.CounterUdb);

        #if(!Index_Counter_1_ControlRegRemoved)
            Index_Counter_1_WriteControlRegister(Index_Counter_1_backup.CounterControlRegister);
        #endif /* (!Index_Counter_1_ControlRegRemoved) */

    #endif /* (!Index_Counter_1_UsingFixedFunction) */
}


/*******************************************************************************
* Function Name: Index_Counter_1_Sleep
********************************************************************************
* Summary:
*     Stop and Save the user configuration
*
* Parameters:  
*  void
*
* Return: 
*  void
*
* Global variables:
*  Index_Counter_1_backup.enableState:  Is modified depending on the enable 
*  state of the block before entering sleep mode.
*
*******************************************************************************/
void Index_Counter_1_Sleep(void) 
{
    #if(!Index_Counter_1_ControlRegRemoved)
        /* Save Counter's enable state */
        if(Index_Counter_1_CTRL_ENABLE == (Index_Counter_1_CONTROL & Index_Counter_1_CTRL_ENABLE))
        {
            /* Counter is enabled */
            Index_Counter_1_backup.CounterEnableState = 1u;
        }
        else
        {
            /* Counter is disabled */
            Index_Counter_1_backup.CounterEnableState = 0u;
        }
    #else
        Index_Counter_1_backup.CounterEnableState = 1u;
        if(Index_Counter_1_backup.CounterEnableState != 0u)
        {
            Index_Counter_1_backup.CounterEnableState = 0u;
        }
    #endif /* (!Index_Counter_1_ControlRegRemoved) */
    
    Index_Counter_1_Stop();
    Index_Counter_1_SaveConfig();
}


/*******************************************************************************
* Function Name: Index_Counter_1_Wakeup
********************************************************************************
*
* Summary:
*  Restores and enables the user configuration
*  
* Parameters:  
*  void
*
* Return: 
*  void
*
* Global variables:
*  Index_Counter_1_backup.enableState:  Is used to restore the enable state of 
*  block on wakeup from sleep mode.
*
*******************************************************************************/
void Index_Counter_1_Wakeup(void) 
{
    Index_Counter_1_RestoreConfig();
    #if(!Index_Counter_1_ControlRegRemoved)
        if(Index_Counter_1_backup.CounterEnableState == 1u)
        {
            /* Enable Counter's operation */
            Index_Counter_1_Enable();
        } /* Do nothing if Counter was disabled before */    
    #endif /* (!Index_Counter_1_ControlRegRemoved) */
    
}


/* [] END OF FILE */
