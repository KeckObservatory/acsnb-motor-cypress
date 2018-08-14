/*******************************************************************************
* File Name: Reset_Encoder.h  
* Version 2.20
*
* Description:
*  This file contains the Alias definitions for Per-Pin APIs in cypins.h. 
*  Information on using these APIs can be found in the System Reference Guide.
*
* Note:
*
********************************************************************************
* Copyright 2008-2015, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_PINS_Reset_Encoder_ALIASES_H) /* Pins Reset_Encoder_ALIASES_H */
#define CY_PINS_Reset_Encoder_ALIASES_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"


/***************************************
*              Constants        
***************************************/
#define Reset_Encoder_0			(Reset_Encoder__0__PC)
#define Reset_Encoder_0_PS		(Reset_Encoder__0__PS)
#define Reset_Encoder_0_PC		(Reset_Encoder__0__PC)
#define Reset_Encoder_0_DR		(Reset_Encoder__0__DR)
#define Reset_Encoder_0_SHIFT	(Reset_Encoder__0__SHIFT)
#define Reset_Encoder_0_INTR	((uint16)((uint16)0x0003u << (Reset_Encoder__0__SHIFT*2u)))

#define Reset_Encoder_INTR_ALL	 ((uint16)(Reset_Encoder_0_INTR))


#endif /* End Pins Reset_Encoder_ALIASES_H */


/* [] END OF FILE */
