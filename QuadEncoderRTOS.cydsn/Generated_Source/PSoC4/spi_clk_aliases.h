/*******************************************************************************
* File Name: spi_clk.h  
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

#if !defined(CY_PINS_spi_clk_ALIASES_H) /* Pins spi_clk_ALIASES_H */
#define CY_PINS_spi_clk_ALIASES_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"


/***************************************
*              Constants        
***************************************/
#define spi_clk_0			(spi_clk__0__PC)
#define spi_clk_0_PS		(spi_clk__0__PS)
#define spi_clk_0_PC		(spi_clk__0__PC)
#define spi_clk_0_DR		(spi_clk__0__DR)
#define spi_clk_0_SHIFT	(spi_clk__0__SHIFT)
#define spi_clk_0_INTR	((uint16)((uint16)0x0003u << (spi_clk__0__SHIFT*2u)))

#define spi_clk_INTR_ALL	 ((uint16)(spi_clk_0_INTR))


#endif /* End Pins spi_clk_ALIASES_H */


/* [] END OF FILE */
