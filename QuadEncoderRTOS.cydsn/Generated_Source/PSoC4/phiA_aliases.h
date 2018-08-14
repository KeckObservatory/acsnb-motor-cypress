/*******************************************************************************
* File Name: phiA.h  
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

#if !defined(CY_PINS_phiA_ALIASES_H) /* Pins phiA_ALIASES_H */
#define CY_PINS_phiA_ALIASES_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"


/***************************************
*              Constants        
***************************************/
#define phiA_0			(phiA__0__PC)
#define phiA_0_PS		(phiA__0__PS)
#define phiA_0_PC		(phiA__0__PC)
#define phiA_0_DR		(phiA__0__DR)
#define phiA_0_SHIFT	(phiA__0__SHIFT)
#define phiA_0_INTR	((uint16)((uint16)0x0003u << (phiA__0__SHIFT*2u)))

#define phiA_INTR_ALL	 ((uint16)(phiA_0_INTR))


#endif /* End Pins phiA_ALIASES_H */


/* [] END OF FILE */
