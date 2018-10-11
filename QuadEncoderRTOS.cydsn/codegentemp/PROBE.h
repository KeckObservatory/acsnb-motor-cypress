/*******************************************************************************
* File Name: PROBE.h  
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

#if !defined(CY_PINS_PROBE_H) /* Pins PROBE_H */
#define CY_PINS_PROBE_H

#include "cytypes.h"
#include "cyfitter.h"
#include "PROBE_aliases.h"


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
} PROBE_BACKUP_STRUCT;

/** @} structures */


/***************************************
*        Function Prototypes             
***************************************/
/**
* \addtogroup group_general
* @{
*/
uint8   PROBE_Read(void);
void    PROBE_Write(uint8 value);
uint8   PROBE_ReadDataReg(void);
#if defined(PROBE__PC) || (CY_PSOC4_4200L) 
    void    PROBE_SetDriveMode(uint8 mode);
#endif
void    PROBE_SetInterruptMode(uint16 position, uint16 mode);
uint8   PROBE_ClearInterrupt(void);
/** @} general */

/**
* \addtogroup group_power
* @{
*/
void PROBE_Sleep(void); 
void PROBE_Wakeup(void);
/** @} power */


/***************************************
*           API Constants        
***************************************/
#if defined(PROBE__PC) || (CY_PSOC4_4200L) 
    /* Drive Modes */
    #define PROBE_DRIVE_MODE_BITS        (3)
    #define PROBE_DRIVE_MODE_IND_MASK    (0xFFFFFFFFu >> (32 - PROBE_DRIVE_MODE_BITS))

    /**
    * \addtogroup group_constants
    * @{
    */
        /** \addtogroup driveMode Drive mode constants
         * \brief Constants to be passed as "mode" parameter in the PROBE_SetDriveMode() function.
         *  @{
         */
        #define PROBE_DM_ALG_HIZ         (0x00u) /**< \brief High Impedance Analog   */
        #define PROBE_DM_DIG_HIZ         (0x01u) /**< \brief High Impedance Digital  */
        #define PROBE_DM_RES_UP          (0x02u) /**< \brief Resistive Pull Up       */
        #define PROBE_DM_RES_DWN         (0x03u) /**< \brief Resistive Pull Down     */
        #define PROBE_DM_OD_LO           (0x04u) /**< \brief Open Drain, Drives Low  */
        #define PROBE_DM_OD_HI           (0x05u) /**< \brief Open Drain, Drives High */
        #define PROBE_DM_STRONG          (0x06u) /**< \brief Strong Drive            */
        #define PROBE_DM_RES_UPDWN       (0x07u) /**< \brief Resistive Pull Up/Down  */
        /** @} driveMode */
    /** @} group_constants */
#endif

/* Digital Port Constants */
#define PROBE_MASK               PROBE__MASK
#define PROBE_SHIFT              PROBE__SHIFT
#define PROBE_WIDTH              1u

/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup intrMode Interrupt constants
     * \brief Constants to be passed as "mode" parameter in PROBE_SetInterruptMode() function.
     *  @{
     */
        #define PROBE_INTR_NONE      ((uint16)(0x0000u)) /**< \brief Disabled             */
        #define PROBE_INTR_RISING    ((uint16)(0x5555u)) /**< \brief Rising edge trigger  */
        #define PROBE_INTR_FALLING   ((uint16)(0xaaaau)) /**< \brief Falling edge trigger */
        #define PROBE_INTR_BOTH      ((uint16)(0xffffu)) /**< \brief Both edge trigger    */
    /** @} intrMode */
/** @} group_constants */

/* SIO LPM definition */
#if defined(PROBE__SIO)
    #define PROBE_SIO_LPM_MASK       (0x03u)
#endif

/* USBIO definitions */
#if !defined(PROBE__PC) && (CY_PSOC4_4200L)
    #define PROBE_USBIO_ENABLE               ((uint32)0x80000000u)
    #define PROBE_USBIO_DISABLE              ((uint32)(~PROBE_USBIO_ENABLE))
    #define PROBE_USBIO_SUSPEND_SHIFT        CYFLD_USBDEVv2_USB_SUSPEND__OFFSET
    #define PROBE_USBIO_SUSPEND_DEL_SHIFT    CYFLD_USBDEVv2_USB_SUSPEND_DEL__OFFSET
    #define PROBE_USBIO_ENTER_SLEEP          ((uint32)((1u << PROBE_USBIO_SUSPEND_SHIFT) \
                                                        | (1u << PROBE_USBIO_SUSPEND_DEL_SHIFT)))
    #define PROBE_USBIO_EXIT_SLEEP_PH1       ((uint32)~((uint32)(1u << PROBE_USBIO_SUSPEND_SHIFT)))
    #define PROBE_USBIO_EXIT_SLEEP_PH2       ((uint32)~((uint32)(1u << PROBE_USBIO_SUSPEND_DEL_SHIFT)))
    #define PROBE_USBIO_CR1_OFF              ((uint32)0xfffffffeu)
#endif


/***************************************
*             Registers        
***************************************/
/* Main Port Registers */
#if defined(PROBE__PC)
    /* Port Configuration */
    #define PROBE_PC                 (* (reg32 *) PROBE__PC)
#endif
/* Pin State */
#define PROBE_PS                     (* (reg32 *) PROBE__PS)
/* Data Register */
#define PROBE_DR                     (* (reg32 *) PROBE__DR)
/* Input Buffer Disable Override */
#define PROBE_INP_DIS                (* (reg32 *) PROBE__PC2)

/* Interrupt configuration Registers */
#define PROBE_INTCFG                 (* (reg32 *) PROBE__INTCFG)
#define PROBE_INTSTAT                (* (reg32 *) PROBE__INTSTAT)

/* "Interrupt cause" register for Combined Port Interrupt (AllPortInt) in GSRef component */
#if defined (CYREG_GPIO_INTR_CAUSE)
    #define PROBE_INTR_CAUSE         (* (reg32 *) CYREG_GPIO_INTR_CAUSE)
#endif

/* SIO register */
#if defined(PROBE__SIO)
    #define PROBE_SIO_REG            (* (reg32 *) PROBE__SIO)
#endif /* (PROBE__SIO_CFG) */

/* USBIO registers */
#if !defined(PROBE__PC) && (CY_PSOC4_4200L)
    #define PROBE_USB_POWER_REG       (* (reg32 *) CYREG_USBDEVv2_USB_POWER_CTRL)
    #define PROBE_CR1_REG             (* (reg32 *) CYREG_USBDEVv2_CR1)
    #define PROBE_USBIO_CTRL_REG      (* (reg32 *) CYREG_USBDEVv2_USB_USBIO_CTRL)
#endif    
    
    
/***************************************
* The following code is DEPRECATED and 
* must not be used in new designs.
***************************************/
/**
* \addtogroup group_deprecated
* @{
*/
#define PROBE_DRIVE_MODE_SHIFT       (0x00u)
#define PROBE_DRIVE_MODE_MASK        (0x07u << PROBE_DRIVE_MODE_SHIFT)
/** @} deprecated */

#endif /* End Pins PROBE_H */


/* [] END OF FILE */
