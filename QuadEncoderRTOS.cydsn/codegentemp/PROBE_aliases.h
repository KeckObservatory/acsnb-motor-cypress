/*******************************************************************************
* File Name: PROBE.h  
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

#if !defined(CY_PINS_PROBE_ALIASES_H) /* Pins PROBE_ALIASES_H */
#define CY_PINS_PROBE_ALIASES_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"


/***************************************
*              Constants        
***************************************/
#define PROBE_0			(PROBE__0__PC)
#define PROBE_0_PS		(PROBE__0__PS)
#define PROBE_0_PC		(PROBE__0__PC)
#define PROBE_0_DR		(PROBE__0__DR)
#define PROBE_0_SHIFT	(PROBE__0__SHIFT)
#define PROBE_0_INTR	((uint16)((uint16)0x0003u << (PROBE__0__SHIFT*2u)))

#define PROBE_INTR_ALL	 ((uint16)(PROBE_0_INTR))


#endif /* End Pins PROBE_ALIASES_H */


/* [] END OF FILE */
