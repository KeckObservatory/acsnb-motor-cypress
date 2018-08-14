/*******************************************************************************
* File Name: Direction.h  
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

#if !defined(CY_PINS_Direction_H) /* Pins Direction_H */
#define CY_PINS_Direction_H

#include "cytypes.h"
#include "cyfitter.h"
#include "Direction_aliases.h"


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
} Direction_BACKUP_STRUCT;

/** @} structures */


/***************************************
*        Function Prototypes             
***************************************/
/**
* \addtogroup group_general
* @{
*/
uint8   Direction_Read(void);
void    Direction_Write(uint8 value);
uint8   Direction_ReadDataReg(void);
#if defined(Direction__PC) || (CY_PSOC4_4200L) 
    void    Direction_SetDriveMode(uint8 mode);
#endif
void    Direction_SetInterruptMode(uint16 position, uint16 mode);
uint8   Direction_ClearInterrupt(void);
/** @} general */

/**
* \addtogroup group_power
* @{
*/
void Direction_Sleep(void); 
void Direction_Wakeup(void);
/** @} power */


/***************************************
*           API Constants        
***************************************/
#if defined(Direction__PC) || (CY_PSOC4_4200L) 
    /* Drive Modes */
    #define Direction_DRIVE_MODE_BITS        (3)
    #define Direction_DRIVE_MODE_IND_MASK    (0xFFFFFFFFu >> (32 - Direction_DRIVE_MODE_BITS))

    /**
    * \addtogroup group_constants
    * @{
    */
        /** \addtogroup driveMode Drive mode constants
         * \brief Constants to be passed as "mode" parameter in the Direction_SetDriveMode() function.
         *  @{
         */
        #define Direction_DM_ALG_HIZ         (0x00u) /**< \brief High Impedance Analog   */
        #define Direction_DM_DIG_HIZ         (0x01u) /**< \brief High Impedance Digital  */
        #define Direction_DM_RES_UP          (0x02u) /**< \brief Resistive Pull Up       */
        #define Direction_DM_RES_DWN         (0x03u) /**< \brief Resistive Pull Down     */
        #define Direction_DM_OD_LO           (0x04u) /**< \brief Open Drain, Drives Low  */
        #define Direction_DM_OD_HI           (0x05u) /**< \brief Open Drain, Drives High */
        #define Direction_DM_STRONG          (0x06u) /**< \brief Strong Drive            */
        #define Direction_DM_RES_UPDWN       (0x07u) /**< \brief Resistive Pull Up/Down  */
        /** @} driveMode */
    /** @} group_constants */
#endif

/* Digital Port Constants */
#define Direction_MASK               Direction__MASK
#define Direction_SHIFT              Direction__SHIFT
#define Direction_WIDTH              1u

/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup intrMode Interrupt constants
     * \brief Constants to be passed as "mode" parameter in Direction_SetInterruptMode() function.
     *  @{
     */
        #define Direction_INTR_NONE      ((uint16)(0x0000u)) /**< \brief Disabled             */
        #define Direction_INTR_RISING    ((uint16)(0x5555u)) /**< \brief Rising edge trigger  */
        #define Direction_INTR_FALLING   ((uint16)(0xaaaau)) /**< \brief Falling edge trigger */
        #define Direction_INTR_BOTH      ((uint16)(0xffffu)) /**< \brief Both edge trigger    */
    /** @} intrMode */
/** @} group_constants */

/* SIO LPM definition */
#if defined(Direction__SIO)
    #define Direction_SIO_LPM_MASK       (0x03u)
#endif

/* USBIO definitions */
#if !defined(Direction__PC) && (CY_PSOC4_4200L)
    #define Direction_USBIO_ENABLE               ((uint32)0x80000000u)
    #define Direction_USBIO_DISABLE              ((uint32)(~Direction_USBIO_ENABLE))
    #define Direction_USBIO_SUSPEND_SHIFT        CYFLD_USBDEVv2_USB_SUSPEND__OFFSET
    #define Direction_USBIO_SUSPEND_DEL_SHIFT    CYFLD_USBDEVv2_USB_SUSPEND_DEL__OFFSET
    #define Direction_USBIO_ENTER_SLEEP          ((uint32)((1u << Direction_USBIO_SUSPEND_SHIFT) \
                                                        | (1u << Direction_USBIO_SUSPEND_DEL_SHIFT)))
    #define Direction_USBIO_EXIT_SLEEP_PH1       ((uint32)~((uint32)(1u << Direction_USBIO_SUSPEND_SHIFT)))
    #define Direction_USBIO_EXIT_SLEEP_PH2       ((uint32)~((uint32)(1u << Direction_USBIO_SUSPEND_DEL_SHIFT)))
    #define Direction_USBIO_CR1_OFF              ((uint32)0xfffffffeu)
#endif


/***************************************
*             Registers        
***************************************/
/* Main Port Registers */
#if defined(Direction__PC)
    /* Port Configuration */
    #define Direction_PC                 (* (reg32 *) Direction__PC)
#endif
/* Pin State */
#define Direction_PS                     (* (reg32 *) Direction__PS)
/* Data Register */
#define Direction_DR                     (* (reg32 *) Direction__DR)
/* Input Buffer Disable Override */
#define Direction_INP_DIS                (* (reg32 *) Direction__PC2)

/* Interrupt configuration Registers */
#define Direction_INTCFG                 (* (reg32 *) Direction__INTCFG)
#define Direction_INTSTAT                (* (reg32 *) Direction__INTSTAT)

/* "Interrupt cause" register for Combined Port Interrupt (AllPortInt) in GSRef component */
#if defined (CYREG_GPIO_INTR_CAUSE)
    #define Direction_INTR_CAUSE         (* (reg32 *) CYREG_GPIO_INTR_CAUSE)
#endif

/* SIO register */
#if defined(Direction__SIO)
    #define Direction_SIO_REG            (* (reg32 *) Direction__SIO)
#endif /* (Direction__SIO_CFG) */

/* USBIO registers */
#if !defined(Direction__PC) && (CY_PSOC4_4200L)
    #define Direction_USB_POWER_REG       (* (reg32 *) CYREG_USBDEVv2_USB_POWER_CTRL)
    #define Direction_CR1_REG             (* (reg32 *) CYREG_USBDEVv2_CR1)
    #define Direction_USBIO_CTRL_REG      (* (reg32 *) CYREG_USBDEVv2_USB_USBIO_CTRL)
#endif    
    
    
/***************************************
* The following code is DEPRECATED and 
* must not be used in new designs.
***************************************/
/**
* \addtogroup group_deprecated
* @{
*/
#define Direction_DRIVE_MODE_SHIFT       (0x00u)
#define Direction_DRIVE_MODE_MASK        (0x07u << Direction_DRIVE_MODE_SHIFT)
/** @} deprecated */

#endif /* End Pins Direction_H */


/* [] END OF FILE */
