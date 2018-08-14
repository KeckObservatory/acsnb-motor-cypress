/*******************************************************************************
* File Name: phiB.h  
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

#if !defined(CY_PINS_phiB_ALIASES_H) /* Pins phiB_ALIASES_H */
#define CY_PINS_phiB_ALIASES_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"


/***************************************
*              Constants        
***************************************/
#define phiB_0			(phiB__0__PC)
#define phiB_0_PS		(phiB__0__PS)
#define phiB_0_PC		(phiB__0__PC)
#define phiB_0_DR		(phiB__0__DR)
#define phiB_0_SHIFT	(phiB__0__SHIFT)
#define phiB_0_INTR	((uint16)((uint16)0x0003u << (phiB__0__SHIFT*2u)))

#define phiB_INTR_ALL	 ((uint16)(phiB_0_INTR))


#endif /* End Pins phiB_ALIASES_H */


/* [] END OF FILE */
