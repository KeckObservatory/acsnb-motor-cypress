/*******************************************************************************
* File Name: HOME_OUT.h  
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

#if !defined(CY_PINS_HOME_OUT_ALIASES_H) /* Pins HOME_OUT_ALIASES_H */
#define CY_PINS_HOME_OUT_ALIASES_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"


/***************************************
*              Constants        
***************************************/
#define HOME_OUT_0			(HOME_OUT__0__PC)
#define HOME_OUT_0_PS		(HOME_OUT__0__PS)
#define HOME_OUT_0_PC		(HOME_OUT__0__PC)
#define HOME_OUT_0_DR		(HOME_OUT__0__DR)
#define HOME_OUT_0_SHIFT	(HOME_OUT__0__SHIFT)
#define HOME_OUT_0_INTR	((uint16)((uint16)0x0003u << (HOME_OUT__0__SHIFT*2u)))

#define HOME_OUT_INTR_ALL	 ((uint16)(HOME_OUT_0_INTR))


#endif /* End Pins HOME_OUT_ALIASES_H */


/* [] END OF FILE */
