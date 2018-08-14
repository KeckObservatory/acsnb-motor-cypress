/*******************************************************************************
* File Name: INDEX_IN.h  
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

#if !defined(CY_PINS_INDEX_IN_ALIASES_H) /* Pins INDEX_IN_ALIASES_H */
#define CY_PINS_INDEX_IN_ALIASES_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"


/***************************************
*              Constants        
***************************************/
#define INDEX_IN_0			(INDEX_IN__0__PC)
#define INDEX_IN_0_PS		(INDEX_IN__0__PS)
#define INDEX_IN_0_PC		(INDEX_IN__0__PC)
#define INDEX_IN_0_DR		(INDEX_IN__0__DR)
#define INDEX_IN_0_SHIFT	(INDEX_IN__0__SHIFT)
#define INDEX_IN_0_INTR	((uint16)((uint16)0x0003u << (INDEX_IN__0__SHIFT*2u)))

#define INDEX_IN_INTR_ALL	 ((uint16)(INDEX_IN_0_INTR))


#endif /* End Pins INDEX_IN_ALIASES_H */


/* [] END OF FILE */
