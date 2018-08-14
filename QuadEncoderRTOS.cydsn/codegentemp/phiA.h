/*******************************************************************************
* File Name: phiA.h  
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

#if !defined(CY_PINS_phiA_H) /* Pins phiA_H */
#define CY_PINS_phiA_H

#include "cytypes.h"
#include "cyfitter.h"
#include "phiA_aliases.h"


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
} phiA_BACKUP_STRUCT;

/** @} structures */


/***************************************
*        Function Prototypes             
***************************************/
/**
* \addtogroup group_general
* @{
*/
uint8   phiA_Read(void);
void    phiA_Write(uint8 value);
uint8   phiA_ReadDataReg(void);
#if defined(phiA__PC) || (CY_PSOC4_4200L) 
    void    phiA_SetDriveMode(uint8 mode);
#endif
void    phiA_SetInterruptMode(uint16 position, uint16 mode);
uint8   phiA_ClearInterrupt(void);
/** @} general */

/**
* \addtogroup group_power
* @{
*/
void phiA_Sleep(void); 
void phiA_Wakeup(void);
/** @} power */


/***************************************
*           API Constants        
***************************************/
#if defined(phiA__PC) || (CY_PSOC4_4200L) 
    /* Drive Modes */
    #define phiA_DRIVE_MODE_BITS        (3)
    #define phiA_DRIVE_MODE_IND_MASK    (0xFFFFFFFFu >> (32 - phiA_DRIVE_MODE_BITS))

    /**
    * \addtogroup group_constants
    * @{
    */
        /** \addtogroup driveMode Drive mode constants
         * \brief Constants to be passed as "mode" parameter in the phiA_SetDriveMode() function.
         *  @{
         */
        #define phiA_DM_ALG_HIZ         (0x00u) /**< \brief High Impedance Analog   */
        #define phiA_DM_DIG_HIZ         (0x01u) /**< \brief High Impedance Digital  */
        #define phiA_DM_RES_UP          (0x02u) /**< \brief Resistive Pull Up       */
        #define phiA_DM_RES_DWN         (0x03u) /**< \brief Resistive Pull Down     */
        #define phiA_DM_OD_LO           (0x04u) /**< \brief Open Drain, Drives Low  */
        #define phiA_DM_OD_HI           (0x05u) /**< \brief Open Drain, Drives High */
        #define phiA_DM_STRONG          (0x06u) /**< \brief Strong Drive            */
        #define phiA_DM_RES_UPDWN       (0x07u) /**< \brief Resistive Pull Up/Down  */
        /** @} driveMode */
    /** @} group_constants */
#endif

/* Digital Port Constants */
#define phiA_MASK               phiA__MASK
#define phiA_SHIFT              phiA__SHIFT
#define phiA_WIDTH              1u

/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup intrMode Interrupt constants
     * \brief Constants to be passed as "mode" parameter in phiA_SetInterruptMode() function.
     *  @{
     */
        #define phiA_INTR_NONE      ((uint16)(0x0000u)) /**< \brief Disabled             */
        #define phiA_INTR_RISING    ((uint16)(0x5555u)) /**< \brief Rising edge trigger  */
        #define phiA_INTR_FALLING   ((uint16)(0xaaaau)) /**< \brief Falling edge trigger */
        #define phiA_INTR_BOTH      ((uint16)(0xffffu)) /**< \brief Both edge trigger    */
    /** @} intrMode */
/** @} group_constants */

/* SIO LPM definition */
#if defined(phiA__SIO)
    #define phiA_SIO_LPM_MASK       (0x03u)
#endif

/* USBIO definitions */
#if !defined(phiA__PC) && (CY_PSOC4_4200L)
    #define phiA_USBIO_ENABLE               ((uint32)0x80000000u)
    #define phiA_USBIO_DISABLE              ((uint32)(~phiA_USBIO_ENABLE))
    #define phiA_USBIO_SUSPEND_SHIFT        CYFLD_USBDEVv2_USB_SUSPEND__OFFSET
    #define phiA_USBIO_SUSPEND_DEL_SHIFT    CYFLD_USBDEVv2_USB_SUSPEND_DEL__OFFSET
    #define phiA_USBIO_ENTER_SLEEP          ((uint32)((1u << phiA_USBIO_SUSPEND_SHIFT) \
                                                        | (1u << phiA_USBIO_SUSPEND_DEL_SHIFT)))
    #define phiA_USBIO_EXIT_SLEEP_PH1       ((uint32)~((uint32)(1u << phiA_USBIO_SUSPEND_SHIFT)))
    #define phiA_USBIO_EXIT_SLEEP_PH2       ((uint32)~((uint32)(1u << phiA_USBIO_SUSPEND_DEL_SHIFT)))
    #define phiA_USBIO_CR1_OFF              ((uint32)0xfffffffeu)
#endif


/***************************************
*             Registers        
***************************************/
/* Main Port Registers */
#if defined(phiA__PC)
    /* Port Configuration */
    #define phiA_PC                 (* (reg32 *) phiA__PC)
#endif
/* Pin State */
#define phiA_PS                     (* (reg32 *) phiA__PS)
/* Data Register */
#define phiA_DR                     (* (reg32 *) phiA__DR)
/* Input Buffer Disable Override */
#define phiA_INP_DIS                (* (reg32 *) phiA__PC2)

/* Interrupt configuration Registers */
#define phiA_INTCFG                 (* (reg32 *) phiA__INTCFG)
#define phiA_INTSTAT                (* (reg32 *) phiA__INTSTAT)

/* "Interrupt cause" register for Combined Port Interrupt (AllPortInt) in GSRef component */
#if defined (CYREG_GPIO_INTR_CAUSE)
    #define phiA_INTR_CAUSE         (* (reg32 *) CYREG_GPIO_INTR_CAUSE)
#endif

/* SIO register */
#if defined(phiA__SIO)
    #define phiA_SIO_REG            (* (reg32 *) phiA__SIO)
#endif /* (phiA__SIO_CFG) */

/* USBIO registers */
#if !defined(phiA__PC) && (CY_PSOC4_4200L)
    #define phiA_USB_POWER_REG       (* (reg32 *) CYREG_USBDEVv2_USB_POWER_CTRL)
    #define phiA_CR1_REG             (* (reg32 *) CYREG_USBDEVv2_CR1)
    #define phiA_USBIO_CTRL_REG      (* (reg32 *) CYREG_USBDEVv2_USB_USBIO_CTRL)
#endif    
    
    
/***************************************
* The following code is DEPRECATED and 
* must not be used in new designs.
***************************************/
/**
* \addtogroup group_deprecated
* @{
*/
#define phiA_DRIVE_MODE_SHIFT       (0x00u)
#define phiA_DRIVE_MODE_MASK        (0x07u << phiA_DRIVE_MODE_SHIFT)
/** @} deprecated */

#endif /* End Pins phiA_H */


/* [] END OF FILE */
