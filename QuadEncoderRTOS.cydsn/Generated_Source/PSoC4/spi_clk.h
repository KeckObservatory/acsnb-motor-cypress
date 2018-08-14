/*******************************************************************************
* File Name: spi_clk.h  
* Version 2.20
*
* Description:
*  This file contains Pin function prototypes and register defines
*
********************************************************************************
* Copyright 2008-2015, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_PINS_spi_clk_H) /* Pins spi_clk_H */
#define CY_PINS_spi_clk_H

#include "cytypes.h"
#include "cyfitter.h"
#include "spi_clk_aliases.h"


/***************************************
*     Data Struct Definitions
***************************************/

/**
* \addtogroup group_structures
* @{
*/
    
/* Structure for sleep mode support */
typedef struct
{
    uint32 pcState; /**< State of the port control register */
    uint32 sioState; /**< State of the SIO configuration */
    uint32 usbState; /**< State of the USBIO regulator */
} spi_clk_BACKUP_STRUCT;

/** @} structures */


/***************************************
*        Function Prototypes             
***************************************/
/**
* \addtogroup group_general
* @{
*/
uint8   spi_clk_Read(void);
void    spi_clk_Write(uint8 value);
uint8   spi_clk_ReadDataReg(void);
#if defined(spi_clk__PC) || (CY_PSOC4_4200L) 
    void    spi_clk_SetDriveMode(uint8 mode);
#endif
void    spi_clk_SetInterruptMode(uint16 position, uint16 mode);
uint8   spi_clk_ClearInterrupt(void);
/** @} general */

/**
* \addtogroup group_power
* @{
*/
void spi_clk_Sleep(void); 
void spi_clk_Wakeup(void);
/** @} power */


/***************************************
*           API Constants        
***************************************/
#if defined(spi_clk__PC) || (CY_PSOC4_4200L) 
    /* Drive Modes */
    #define spi_clk_DRIVE_MODE_BITS        (3)
    #define spi_clk_DRIVE_MODE_IND_MASK    (0xFFFFFFFFu >> (32 - spi_clk_DRIVE_MODE_BITS))

    /**
    * \addtogroup group_constants
    * @{
    */
        /** \addtogroup driveMode Drive mode constants
         * \brief Constants to be passed as "mode" parameter in the spi_clk_SetDriveMode() function.
         *  @{
         */
        #define spi_clk_DM_ALG_HIZ         (0x00u) /**< \brief High Impedance Analog   */
        #define spi_clk_DM_DIG_HIZ         (0x01u) /**< \brief High Impedance Digital  */
        #define spi_clk_DM_RES_UP          (0x02u) /**< \brief Resistive Pull Up       */
        #define spi_clk_DM_RES_DWN         (0x03u) /**< \brief Resistive Pull Down     */
        #define spi_clk_DM_OD_LO           (0x04u) /**< \brief Open Drain, Drives Low  */
        #define spi_clk_DM_OD_HI           (0x05u) /**< \brief Open Drain, Drives High */
        #define spi_clk_DM_STRONG          (0x06u) /**< \brief Strong Drive            */
        #define spi_clk_DM_RES_UPDWN       (0x07u) /**< \brief Resistive Pull Up/Down  */
        /** @} driveMode */
    /** @} group_constants */
#endif

/* Digital Port Constants */
#define spi_clk_MASK               spi_clk__MASK
#define spi_clk_SHIFT              spi_clk__SHIFT
#define spi_clk_WIDTH              1u

/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup intrMode Interrupt constants
     * \brief Constants to be passed as "mode" parameter in spi_clk_SetInterruptMode() function.
     *  @{
     */
        #define spi_clk_INTR_NONE      ((uint16)(0x0000u)) /**< \brief Disabled             */
        #define spi_clk_INTR_RISING    ((uint16)(0x5555u)) /**< \brief Rising edge trigger  */
        #define spi_clk_INTR_FALLING   ((uint16)(0xaaaau)) /**< \brief Falling edge trigger */
        #define spi_clk_INTR_BOTH      ((uint16)(0xffffu)) /**< \brief Both edge trigger    */
    /** @} intrMode */
/** @} group_constants */

/* SIO LPM definition */
#if defined(spi_clk__SIO)
    #define spi_clk_SIO_LPM_MASK       (0x03u)
#endif

/* USBIO definitions */
#if !defined(spi_clk__PC) && (CY_PSOC4_4200L)
    #define spi_clk_USBIO_ENABLE               ((uint32)0x80000000u)
    #define spi_clk_USBIO_DISABLE              ((uint32)(~spi_clk_USBIO_ENABLE))
    #define spi_clk_USBIO_SUSPEND_SHIFT        CYFLD_USBDEVv2_USB_SUSPEND__OFFSET
    #define spi_clk_USBIO_SUSPEND_DEL_SHIFT    CYFLD_USBDEVv2_USB_SUSPEND_DEL__OFFSET
    #define spi_clk_USBIO_ENTER_SLEEP          ((uint32)((1u << spi_clk_USBIO_SUSPEND_SHIFT) \
                                                        | (1u << spi_clk_USBIO_SUSPEND_DEL_SHIFT)))
    #define spi_clk_USBIO_EXIT_SLEEP_PH1       ((uint32)~((uint32)(1u << spi_clk_USBIO_SUSPEND_SHIFT)))
    #define spi_clk_USBIO_EXIT_SLEEP_PH2       ((uint32)~((uint32)(1u << spi_clk_USBIO_SUSPEND_DEL_SHIFT)))
    #define spi_clk_USBIO_CR1_OFF              ((uint32)0xfffffffeu)
#endif


/***************************************
*             Registers        
***************************************/
/* Main Port Registers */
#if defined(spi_clk__PC)
    /* Port Configuration */
    #define spi_clk_PC                 (* (reg32 *) spi_clk__PC)
#endif
/* Pin State */
#define spi_clk_PS                     (* (reg32 *) spi_clk__PS)
/* Data Register */
#define spi_clk_DR                     (* (reg32 *) spi_clk__DR)
/* Input Buffer Disable Override */
#define spi_clk_INP_DIS                (* (reg32 *) spi_clk__PC2)

/* Interrupt configuration Registers */
#define spi_clk_INTCFG                 (* (reg32 *) spi_clk__INTCFG)
#define spi_clk_INTSTAT                (* (reg32 *) spi_clk__INTSTAT)

/* "Interrupt cause" register for Combined Port Interrupt (AllPortInt) in GSRef component */
#if defined (CYREG_GPIO_INTR_CAUSE)
    #define spi_clk_INTR_CAUSE         (* (reg32 *) CYREG_GPIO_INTR_CAUSE)
#endif

/* SIO register */
#if defined(spi_clk__SIO)
    #define spi_clk_SIO_REG            (* (reg32 *) spi_clk__SIO)
#endif /* (spi_clk__SIO_CFG) */

/* USBIO registers */
#if !defined(spi_clk__PC) && (CY_PSOC4_4200L)
    #define spi_clk_USB_POWER_REG       (* (reg32 *) CYREG_USBDEVv2_USB_POWER_CTRL)
    #define spi_clk_CR1_REG             (* (reg32 *) CYREG_USBDEVv2_CR1)
    #define spi_clk_USBIO_CTRL_REG      (* (reg32 *) CYREG_USBDEVv2_USB_USBIO_CTRL)
#endif    
    
    
/***************************************
* The following code is DEPRECATED and 
* must not be used in new designs.
***************************************/
/**
* \addtogroup group_deprecated
* @{
*/
#define spi_clk_DRIVE_MODE_SHIFT       (0x00u)
#define spi_clk_DRIVE_MODE_MASK        (0x07u << spi_clk_DRIVE_MODE_SHIFT)
/** @} deprecated */

#endif /* End Pins spi_clk_H */


/* [] END OF FILE */
