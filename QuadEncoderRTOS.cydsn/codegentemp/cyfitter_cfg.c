
/*******************************************************************************
* File Name: cyfitter_cfg.c
* 
* PSoC Creator  4.4
*
* Description:
* This file contains device initialization code.
* Except for the user defined sections in CyClockStartupError(), this file should not be modified.
* This file is automatically generated by PSoC Creator.
*
********************************************************************************
* Copyright (c) 2007-2020 Cypress Semiconductor.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
********************************************************************************/

#include <string.h>
#include "cytypes.h"
#include "cydevice_trm.h"
#include "cyfitter.h"
#include "CyLib.h"
#include "cyfitter_cfg.h"


#if defined(__GNUC__) || defined(__ARMCC_VERSION)
    #define CYPACKED 
    #define CYPACKED_ATTR __attribute__ ((packed))
    #define CYALIGNED __attribute__ ((aligned))
    #define CY_CFG_UNUSED __attribute__ ((unused))
    #ifndef CY_CFG_SECTION
        #define CY_CFG_SECTION __attribute__ ((section(".psocinit")))
    #endif
    
    #if defined(__ARMCC_VERSION)
        #define CY_CFG_MEMORY_BARRIER() __memory_changed()
    #else
        #define CY_CFG_MEMORY_BARRIER() __sync_synchronize()
    #endif
    
#elif defined(__ICCARM__)
    #include <intrinsics.h>

    #define CYPACKED __packed
    #define CYPACKED_ATTR 
    #define CYALIGNED _Pragma("data_alignment=4")
    #define CY_CFG_UNUSED _Pragma("diag_suppress=Pe177")
    #define CY_CFG_SECTION _Pragma("location=\".psocinit\"")
    
    #define CY_CFG_MEMORY_BARRIER() __DMB()
    
#else
    #error Unsupported toolchain
#endif

#ifndef CYCODE
    #define CYCODE
#endif
#ifndef CYDATA
    #define CYDATA
#endif
#ifndef CYFAR
    #define CYFAR
#endif
#ifndef CYXDATA
    #define CYXDATA
#endif


CY_CFG_UNUSED
static void CYMEMZERO(void *s, size_t n);
CY_CFG_UNUSED
static void CYMEMZERO(void *s, size_t n)
{
	(void)memset(s, 0, n);
}
CY_CFG_UNUSED
static void CYCONFIGCPY(void *dest, const void *src, size_t n);
CY_CFG_UNUSED
static void CYCONFIGCPY(void *dest, const void *src, size_t n)
{
	(void)memcpy(dest, src, n);
}
CY_CFG_UNUSED
static void CYCONFIGCPYCODE(void *dest, const void *src, size_t n);
CY_CFG_UNUSED
static void CYCONFIGCPYCODE(void *dest, const void *src, size_t n)
{
	(void)memcpy(dest, src, n);
}




/* Clock startup error codes                                                   */
#define CYCLOCKSTART_NO_ERROR    0u
#define CYCLOCKSTART_XTAL_ERROR  1u
#define CYCLOCKSTART_32KHZ_ERROR 2u
#define CYCLOCKSTART_PLL_ERROR   3u
#define CYCLOCKSTART_FLL_ERROR   4u
#define CYCLOCKSTART_WCO_ERROR   5u


#ifdef CY_NEED_CYCLOCKSTARTUPERROR
/*******************************************************************************
* Function Name: CyClockStartupError
********************************************************************************
* Summary:
*  If an error is encountered during clock configuration (crystal startup error,
*  PLL lock error, etc.), the system will end up here.  Unless reimplemented by
*  the customer, this function will stop in an infinite loop.
*
* Parameters:
*   void
*
* Return:
*   void
*
*******************************************************************************/
CY_CFG_UNUSED
static void CyClockStartupError(uint8 errorCode);
CY_CFG_UNUSED
static void CyClockStartupError(uint8 errorCode)
{
    /* To remove the compiler warning if errorCode not used.                */
    errorCode = errorCode;

    /* If we have a clock startup error (bad MHz crystal, PLL lock, etc.),  */
    /* we will end up here to allow the customer to implement something to  */
    /* deal with the clock condition.                                       */

#ifdef CY_CFG_CLOCK_STARTUP_ERROR_CALLBACK
    CY_CFG_Clock_Startup_ErrorCallback();
#else
    /*  If not using CY_CFG_CLOCK_STARTUP_ERROR_CALLBACK, place your clock startup code here. */
    /* `#START CyClockStartupError` */



    /* `#END` */

    while(1) {}
#endif /* CY_CFG_CLOCK_STARTUP_ERROR_CALLBACK */
}
#endif

#define CY_CFG_BASE_ADDR_COUNT 10u
CYPACKED typedef struct
{
	uint8 offset;
	uint8 value;
} CYPACKED_ATTR cy_cfg_addrvalue_t;



/*******************************************************************************
* Function Name: cfg_write_bytes32
********************************************************************************
* Summary:
*  This function is used for setting up the chip configuration areas that
*  contain relatively sparse data.
*
* Parameters:
*   void
*
* Return:
*   void
*
*******************************************************************************/
static void cfg_write_bytes32(const uint32 addr_table[], const cy_cfg_addrvalue_t data_table[]);
static void cfg_write_bytes32(const uint32 addr_table[], const cy_cfg_addrvalue_t data_table[])
{
	/* For 32-bit little-endian architectures */
	uint32 i, j = 0u;
	for (i = 0u; i < CY_CFG_BASE_ADDR_COUNT; i++)
	{
		uint32 baseAddr = addr_table[i];
		uint8 count = (uint8)baseAddr;
		baseAddr &= 0xFFFFFF00u;
		while (count != 0u)
		{
			CY_SET_REG8((void *)(baseAddr + data_table[j].offset), data_table[j].value);
			j++;
			count--;
		}
	}
}


/*******************************************************************************
* Function Name: ClockSetup
********************************************************************************
*
* Summary:
*   Performs the initialization of all of the clocks in the device based on the
*   settings in the Clock tab of the DWR.  This includes enabling the requested
*   clocks and setting the necessary dividers to produce the desired frequency. 
*
* Parameters:
*   void
*
* Return:
*   void
*
*******************************************************************************/
static void ClockSetup(void);
CY_CFG_SECTION
static void ClockSetup(void)
{
	/* Enable HALF_EN before trimming for the flash accelerator. */
	CY_SET_REG32((void CYXDATA *)(CYREG_CLK_SELECT), (CY_GET_REG32((void *)CYREG_CLK_SELECT) | 0x00040000u));

	/* Setup and trim IMO based on desired frequency. */
	CySysClkWriteImoFreq(48u);
	/* CYDEV_CLK_ILO_CONFIG Starting address: CYDEV_CLK_ILO_CONFIG */
	CY_SET_REG32((void *)(CYREG_CLK_ILO_CONFIG), 0x80000006u);


	/* CYDEV_CLK_SELECT00 Starting address: CYDEV_CLK_SELECT00 */
	CY_SET_REG32((void *)(CYREG_CLK_SELECT02), 0x00000030u);
	CY_SET_REG32((void *)(CYREG_CLK_SELECT03), 0x00000010u);
	CY_SET_REG32((void *)(CYREG_CLK_SELECT10), 0x00000020u);
	CY_SET_REG32((void *)(CYREG_CLK_SELECT12), 0x00000020u);

	/* CYDEV_CLK_IMO_CONFIG Starting address: CYDEV_CLK_IMO_CONFIG */
	CY_SET_REG32((void *)(CYREG_CLK_IMO_CONFIG), 0x80000000u);

	/* CYDEV_CLK_SELECT Starting address: CYDEV_CLK_SELECT */
	CY_SET_REG32((void *)(CYREG_CLK_SELECT), 0x00040000u);

	/* CYDEV_CLK_DIVIDER_A00 Starting address: CYDEV_CLK_DIVIDER_A00 */
	CY_SET_REG32((void *)(CYREG_CLK_DIVIDER_A00), 0x80000000u);

	/* CYDEV_CLK_DIVIDER_B00 Starting address: CYDEV_CLK_DIVIDER_B00 */
	CY_SET_REG32((void *)(CYREG_CLK_DIVIDER_B00), 0x80000001u);

	/* CYDEV_CLK_DIVIDER_C00 Starting address: CYDEV_CLK_DIVIDER_C00 */
	CY_SET_REG32((void *)(CYREG_CLK_DIVIDER_C00), 0x8000001Du);

	CY_SET_REG32((void *)(CYREG_WDT_CONFIG), 0x00000000u);
}


/* Analog API Functions */


/*******************************************************************************
* Function Name: AnalogSetDefault
********************************************************************************
*
* Summary:
*  Sets up the analog portions of the chip to default values based on chip
*  configuration options from the project.
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
static void AnalogSetDefault(void);
static void AnalogSetDefault(void)
{
}




/*******************************************************************************
* Function Name: cyfitter_cfg
********************************************************************************
* Summary:
*  This function is called by the start-up code for the selected device. It
*  performs all of the necessary device configuration based on the design
*  settings.  This includes settings from the Design Wide Resources (DWR) such
*  as Clocks and Pins as well as any component configuration that is necessary.
*
* Parameters:
*   void
*
* Return:
*   void
*
*******************************************************************************/
CY_CFG_SECTION
void cyfitter_cfg(void)
{
	/* Disable interrupts by default. Let user enable if/when they want. */
	CyGlobalIntDisable;

	/* Enable the clock in the interrupt controller for the routed interrupts */
	CY_SET_REG8((void *)CYREG_UDB_UDBIF_INT_CLK_CTL, 0x01u);
	{
		static const uint32 CYCODE cy_cfg_addr_table[] = {
			0x400F0002u, /* Base address: 0x400F0000 Count: 2 */
			0x400F3068u, /* Base address: 0x400F3000 Count: 104 */
			0x400F313Eu, /* Base address: 0x400F3100 Count: 62 */
			0x400F3263u, /* Base address: 0x400F3200 Count: 99 */
			0x400F334Cu, /* Base address: 0x400F3300 Count: 76 */
			0x400F400Du, /* Base address: 0x400F4000 Count: 13 */
			0x400F4102u, /* Base address: 0x400F4100 Count: 2 */
			0x400F4206u, /* Base address: 0x400F4200 Count: 6 */
			0x400F4303u, /* Base address: 0x400F4300 Count: 3 */
			0x400F6002u, /* Base address: 0x400F6000 Count: 2 */
		};

		static const cy_cfg_addrvalue_t CYCODE cy_cfg_data_table[] = {
			{0x80u, 0x67u},
			{0x81u, 0x6Fu},
			{0x01u, 0x10u},
			{0x03u, 0x04u},
			{0x06u, 0x08u},
			{0x07u, 0x20u},
			{0x09u, 0x01u},
			{0x0Bu, 0x12u},
			{0x0Du, 0x06u},
			{0x0Fu, 0x11u},
			{0x15u, 0x1Fu},
			{0x19u, 0x07u},
			{0x1Bu, 0x08u},
			{0x1Cu, 0x02u},
			{0x1Eu, 0x01u},
			{0x1Fu, 0xC0u},
			{0x20u, 0x01u},
			{0x21u, 0x18u},
			{0x22u, 0x12u},
			{0x23u, 0x03u},
			{0x26u, 0x04u},
			{0x27u, 0x80u},
			{0x2Bu, 0x40u},
			{0x30u, 0x10u},
			{0x31u, 0x1Fu},
			{0x32u, 0x04u},
			{0x33u, 0x80u},
			{0x34u, 0x03u},
			{0x35u, 0x40u},
			{0x36u, 0x08u},
			{0x37u, 0x20u},
			{0x3Eu, 0x10u},
			{0x3Fu, 0x01u},
			{0x40u, 0x40u},
			{0x41u, 0x02u},
			{0x49u, 0xFFu},
			{0x4Au, 0xFFu},
			{0x4Bu, 0xFFu},
			{0x4Du, 0xA0u},
			{0x4Fu, 0x01u},
			{0x50u, 0x18u},
			{0x52u, 0x87u},
			{0x54u, 0x07u},
			{0x58u, 0x04u},
			{0x59u, 0x04u},
			{0x5Au, 0x04u},
			{0x5Bu, 0x04u},
			{0x5Fu, 0x01u},
			{0x60u, 0x40u},
			{0x62u, 0x80u},
			{0x64u, 0x40u},
			{0x65u, 0x40u},
			{0x66u, 0x80u},
			{0x68u, 0x40u},
			{0x6Au, 0x80u},
			{0x6Cu, 0x40u},
			{0x6Du, 0x20u},
			{0x6Eu, 0x80u},
			{0x80u, 0x55u},
			{0x82u, 0xAAu},
			{0x89u, 0x33u},
			{0x8Bu, 0xCCu},
			{0x8Cu, 0x96u},
			{0x8Eu, 0x69u},
			{0x91u, 0x55u},
			{0x92u, 0xFFu},
			{0x93u, 0xAAu},
			{0x94u, 0x33u},
			{0x96u, 0xCCu},
			{0x9Du, 0x0Fu},
			{0x9Fu, 0xF0u},
			{0xA0u, 0x0Fu},
			{0xA1u, 0x96u},
			{0xA2u, 0xF0u},
			{0xA3u, 0x69u},
			{0xA4u, 0xFFu},
			{0xB0u, 0xFFu},
			{0xB5u, 0xFFu},
			{0xB9u, 0x02u},
			{0xBEu, 0x01u},
			{0xBFu, 0x01u},
			{0xC0u, 0x40u},
			{0xC1u, 0x06u},
			{0xC9u, 0xFFu},
			{0xCAu, 0xFFu},
			{0xCBu, 0xFFu},
			{0xCDu, 0xAFu},
			{0xCEu, 0x07u},
			{0xCFu, 0x01u},
			{0xD0u, 0x18u},
			{0xD4u, 0x0Fu},
			{0xD8u, 0x04u},
			{0xD9u, 0x04u},
			{0xDAu, 0x04u},
			{0xDBu, 0x04u},
			{0xDFu, 0x01u},
			{0xE0u, 0x40u},
			{0xE2u, 0x80u},
			{0xE4u, 0x40u},
			{0xE5u, 0x40u},
			{0xE6u, 0x80u},
			{0xE8u, 0x40u},
			{0xEAu, 0x80u},
			{0xECu, 0x40u},
			{0xEDu, 0x20u},
			{0xEEu, 0x80u},
			{0x01u, 0x08u},
			{0x05u, 0x42u},
			{0x0Au, 0x02u},
			{0x0Eu, 0x10u},
			{0x0Fu, 0x40u},
			{0x12u, 0x20u},
			{0x13u, 0x02u},
			{0x17u, 0x0Au},
			{0x18u, 0x04u},
			{0x19u, 0x4Au},
			{0x1Au, 0x1Au},
			{0x1Bu, 0x80u},
			{0x1Cu, 0x40u},
			{0x1Du, 0x02u},
			{0x20u, 0x08u},
			{0x21u, 0x04u},
			{0x22u, 0xA0u},
			{0x23u, 0x82u},
			{0x26u, 0x02u},
			{0x27u, 0x08u},
			{0x2Au, 0x20u},
			{0x2Bu, 0x60u},
			{0x2Fu, 0x80u},
			{0x30u, 0x20u},
			{0x31u, 0x80u},
			{0x32u, 0x08u},
			{0x35u, 0x02u},
			{0x36u, 0x80u},
			{0x39u, 0x58u},
			{0x3Au, 0x01u},
			{0x3Eu, 0x10u},
			{0x40u, 0x44u},
			{0x45u, 0x02u},
			{0x4Eu, 0x04u},
			{0x58u, 0x10u},
			{0x59u, 0x04u},
			{0x5Au, 0x80u},
			{0x5Eu, 0x06u},
			{0x5Fu, 0xA0u},
			{0x61u, 0x08u},
			{0x62u, 0x10u},
			{0x66u, 0x04u},
			{0x67u, 0x04u},
			{0x68u, 0x20u},
			{0x69u, 0x50u},
			{0x6Bu, 0x01u},
			{0x73u, 0x40u},
			{0x84u, 0x01u},
			{0x87u, 0x20u},
			{0x8Au, 0x20u},
			{0xC0u, 0x94u},
			{0xC2u, 0x31u},
			{0xC4u, 0x3Cu},
			{0xCAu, 0x87u},
			{0xCCu, 0x9Eu},
			{0xCEu, 0x2Fu},
			{0xD0u, 0x85u},
			{0xD2u, 0x20u},
			{0xD6u, 0xFEu},
			{0xD8u, 0x66u},
			{0xE0u, 0x02u},
			{0xE4u, 0x80u},
			{0x03u, 0x02u},
			{0x06u, 0x0Cu},
			{0x0Bu, 0x0Cu},
			{0x0Cu, 0x02u},
			{0x13u, 0x08u},
			{0x14u, 0x04u},
			{0x19u, 0x02u},
			{0x1Eu, 0x03u},
			{0x2Fu, 0x07u},
			{0x30u, 0x01u},
			{0x32u, 0x08u},
			{0x33u, 0x02u},
			{0x34u, 0x02u},
			{0x35u, 0x01u},
			{0x36u, 0x04u},
			{0x37u, 0x0Cu},
			{0x3Eu, 0x50u},
			{0x3Fu, 0x04u},
			{0x40u, 0x40u},
			{0x41u, 0x06u},
			{0x45u, 0x3Cu},
			{0x46u, 0x25u},
			{0x47u, 0x4Eu},
			{0x48u, 0x3Fu},
			{0x49u, 0xFFu},
			{0x4Au, 0xFFu},
			{0x4Bu, 0xFFu},
			{0x4Du, 0xAFu},
			{0x4Eu, 0x07u},
			{0x4Fu, 0x01u},
			{0x50u, 0x18u},
			{0x52u, 0x80u},
			{0x58u, 0x04u},
			{0x59u, 0x04u},
			{0x5Au, 0x04u},
			{0x5Bu, 0x04u},
			{0x5Fu, 0x01u},
			{0x60u, 0x40u},
			{0x62u, 0x80u},
			{0x64u, 0x40u},
			{0x65u, 0x40u},
			{0x66u, 0x80u},
			{0x68u, 0x40u},
			{0x6Au, 0x80u},
			{0x6Cu, 0x40u},
			{0x6Du, 0x20u},
			{0x6Eu, 0x80u},
			{0x80u, 0x02u},
			{0x83u, 0x40u},
			{0x86u, 0x01u},
			{0x89u, 0x02u},
			{0x8Bu, 0x24u},
			{0x8Eu, 0x04u},
			{0x8Fu, 0x01u},
			{0x91u, 0x0Cu},
			{0x93u, 0x22u},
			{0x99u, 0x0Eu},
			{0x9Bu, 0x10u},
			{0x9Cu, 0x04u},
			{0xA1u, 0x30u},
			{0xA2u, 0x02u},
			{0xA3u, 0x06u},
			{0xADu, 0x20u},
			{0xAFu, 0x08u},
			{0xB0u, 0x04u},
			{0xB1u, 0x40u},
			{0xB2u, 0x02u},
			{0xB3u, 0x3Eu},
			{0xB4u, 0x01u},
			{0xB7u, 0x01u},
			{0xBEu, 0x05u},
			{0xC0u, 0x31u},
			{0xC1u, 0x05u},
			{0xC5u, 0x52u},
			{0xC6u, 0xC0u},
			{0xC7u, 0x0Eu},
			{0xC8u, 0x1Fu},
			{0xC9u, 0xFFu},
			{0xCAu, 0xFFu},
			{0xCBu, 0xFFu},
			{0xCDu, 0xA0u},
			{0xCFu, 0x01u},
			{0xD0u, 0x18u},
			{0xD8u, 0x0Au},
			{0xD9u, 0x04u},
			{0xDAu, 0x04u},
			{0xDFu, 0x01u},
			{0xE0u, 0x40u},
			{0xE2u, 0x80u},
			{0xE4u, 0x40u},
			{0xE5u, 0x40u},
			{0xE6u, 0x80u},
			{0xE8u, 0x40u},
			{0xEAu, 0x50u},
			{0xEBu, 0xA8u},
			{0xECu, 0x40u},
			{0xEDu, 0x20u},
			{0xEEu, 0x50u},
			{0xEFu, 0xA8u},
			{0x00u, 0x88u},
			{0x02u, 0x80u},
			{0x04u, 0x40u},
			{0x06u, 0x20u},
			{0x07u, 0x20u},
			{0x0Au, 0x02u},
			{0x0Eu, 0x20u},
			{0x0Fu, 0x01u},
			{0x10u, 0x80u},
			{0x18u, 0x08u},
			{0x19u, 0x04u},
			{0x1Au, 0x80u},
			{0x1Cu, 0x40u},
			{0x1Eu, 0x26u},
			{0x22u, 0x89u},
			{0x24u, 0x10u},
			{0x25u, 0x80u},
			{0x26u, 0x20u},
			{0x27u, 0x02u},
			{0x2Bu, 0x41u},
			{0x2Cu, 0x01u},
			{0x30u, 0x20u},
			{0x33u, 0x01u},
			{0x36u, 0x20u},
			{0x37u, 0x02u},
			{0x39u, 0x90u},
			{0x3Bu, 0x02u},
			{0x3Cu, 0x02u},
			{0x3Du, 0x20u},
			{0x40u, 0x10u},
			{0x41u, 0x40u},
			{0x45u, 0x02u},
			{0x48u, 0x40u},
			{0x49u, 0x08u},
			{0x4Au, 0x02u},
			{0x4Bu, 0x02u},
			{0x4Eu, 0x04u},
			{0x4Fu, 0x05u},
			{0x50u, 0x80u},
			{0x51u, 0x48u},
			{0x52u, 0x10u},
			{0x56u, 0x25u},
			{0x57u, 0x41u},
			{0x64u, 0x10u},
			{0x65u, 0x08u},
			{0x74u, 0x01u},
			{0x79u, 0x08u},
			{0x80u, 0x04u},
			{0x91u, 0x18u},
			{0x92u, 0x08u},
			{0x96u, 0x84u},
			{0x97u, 0x04u},
			{0x98u, 0x01u},
			{0x9Bu, 0x01u},
			{0x9Cu, 0x10u},
			{0x9Du, 0x04u},
			{0x9Eu, 0x15u},
			{0x9Fu, 0x20u},
			{0xA0u, 0x24u},
			{0xA1u, 0x12u},
			{0xA2u, 0x02u},
			{0xA5u, 0x88u},
			{0xA6u, 0x09u},
			{0xA7u, 0x40u},
			{0xA9u, 0x10u},
			{0xC0u, 0xADu},
			{0xC2u, 0xA1u},
			{0xC4u, 0x08u},
			{0xCAu, 0x19u},
			{0xCCu, 0xA5u},
			{0xCEu, 0xADu},
			{0xD0u, 0x8Au},
			{0xD2u, 0x28u},
			{0xDEu, 0x02u},
			{0xE4u, 0x01u},
			{0xE8u, 0x01u},
			{0x02u, 0x42u},
			{0x05u, 0x08u},
			{0x07u, 0x01u},
			{0x0Eu, 0x08u},
			{0x6Eu, 0x08u},
			{0x85u, 0x08u},
			{0x87u, 0x01u},
			{0x8Au, 0x40u},
			{0xC0u, 0xF0u},
			{0xC2u, 0x80u},
			{0xDAu, 0x80u},
			{0xE0u, 0x40u},
			{0xE4u, 0x10u},
			{0xAAu, 0x02u},
			{0xE8u, 0x80u},
			{0x51u, 0x10u},
			{0x5Cu, 0x02u},
			{0x5Fu, 0x80u},
			{0x85u, 0x10u},
			{0xD4u, 0x04u},
			{0xD6u, 0x05u},
			{0x80u, 0x02u},
			{0x9Cu, 0x02u},
			{0xABu, 0x80u},
			{0x01u, 0x01u},
			{0x10u, 0x01u},
		};



		CYPACKED typedef struct {
			void CYFAR *address;
			uint16 size;
		} CYPACKED_ATTR cfg_memset_t;

		static const cfg_memset_t CYCODE cfg_memset_list[] = {
			/* address, size */
			{(void CYFAR *)(CYDEV_UDB_P0_U0_BASE), 1024u},
			{(void CYFAR *)(CYDEV_UDB_DSI0_BASE), 1024u},
		};

		uint8 CYDATA i;

		/* Zero out critical memory blocks before beginning configuration */
		for (i = 0u; i < (sizeof(cfg_memset_list)/sizeof(cfg_memset_list[0])); i++)
		{
			const cfg_memset_t CYCODE * CYDATA ms = &cfg_memset_list[i];
			CYMEMZERO(ms->address, (size_t)(uint32)(ms->size));
		}

		cfg_write_bytes32(cy_cfg_addr_table, cy_cfg_data_table);

		/* HSIOM Starting address: CYDEV_HSIOM_BASE */
		CY_SET_REG32((void *)(CYDEV_HSIOM_BASE), 0xFFFF0000u);
		CY_SET_REG32((void *)(CYREG_HSIOM_PORT_SEL1), 0x00000008u);
		CY_SET_REG32((void *)(CYREG_HSIOM_PORT_SEL2), 0x00033300u);
		CY_SET_REG32((void *)(CYREG_HSIOM_PORT_SEL3), 0x0000EE00u);
		CY_SET_REG32((void *)(CYREG_HSIOM_PORT_SEL4), 0x000000EEu);

		/* UDB_PA_0 Starting address: CYDEV_UDB_PA0_BASE */
		CY_SET_REG32((void *)(CYDEV_UDB_PA0_BASE), 0x00990000u);

		/* UDB_PA_1 Starting address: CYDEV_UDB_PA1_BASE */
		CY_SET_REG32((void *)(CYDEV_UDB_PA1_BASE), 0x00990000u);

		/* UDB_PA_2 Starting address: CYDEV_UDB_PA2_BASE */
		CY_SET_REG32((void *)(CYDEV_UDB_PA2_BASE), 0x00990000u);
		CY_SET_REG32((void *)(CYREG_UDB_PA2_CFG8), 0x01350000u);

		/* UDB_PA_3 Starting address: CYDEV_UDB_PA3_BASE */
		CY_SET_REG32((void *)(CYDEV_UDB_PA3_BASE), 0x00990000u);

		/* INT_SELECT Starting address: CYDEV_CPUSS_INTR_SELECT */
		CY_SET_REG32((void *)(CYREG_CPUSS_INTR_SELECT), 0x00000002u);

		/* INT_CONFIG Starting address: CYDEV_UDB_INT_CFG */
		CY_SET_REG32((void *)(CYREG_UDB_INT_CFG), 0x00000002u);

		/* Enable UDB array and digital routing */
		CY_SET_XTND_REG8((void *)CYREG_UDB_UDBIF_WAIT_CFG, (uint8)((CY_GET_XTND_REG8((void *)CYREG_UDB_UDBIF_WAIT_CFG) & 0xC3u) | 0x14u));
		CY_SET_XTND_REG8((void *)CYREG_UDB_UDBIF_BANK_CTL, (uint8)(CY_GET_XTND_REG8((void *)CYREG_UDB_UDBIF_BANK_CTL) | 0x16u));
	}

	/* Perform second pass device configuration. These items must be configured in specific order after the regular configuration is done. */
	/* IOPINS0_0 Starting address: CYDEV_PRT0_BASE */
	CY_SET_REG32((void *)(CYDEV_PRT0_BASE), 0x0000002Cu);
	CY_SET_REG32((void *)(CYREG_PRT0_PC), 0x00271489u);
	CY_SET_REG32((void *)(CYREG_PRT0_INTCFG), 0x00000020u);

	/* IOPINS0_1 Starting address: CYDEV_PRT1_BASE */
	CY_SET_REG32((void *)(CYDEV_PRT1_BASE), 0x00000001u);
	CY_SET_REG32((void *)(CYREG_PRT1_PC), 0x00D80006u);

	/* IOPINS0_2 Starting address: CYDEV_PRT2_BASE */
	CY_SET_REG32((void *)(CYDEV_PRT2_BASE), 0x0000001Cu);
	CY_SET_REG32((void *)(CYREG_PRT2_PC), 0x00046D80u);
	CY_SET_REG32((void *)(CYREG_PRT2_INTCFG), 0x00002000u);

	/* IOPINS0_3 Starting address: CYDEV_PRT3_BASE */
	CY_SET_REG32((void *)(CYREG_PRT3_PC), 0x00000D80u);

	/* IOPINS0_4 Starting address: CYDEV_PRT4_BASE */
	CY_SET_REG32((void *)(CYDEV_PRT4_BASE), 0x00000003u);
	CY_SET_REG32((void *)(CYREG_PRT4_PC), 0x00000024u);


	/* Setup clocks based on selections from Clock DWR */
	ClockSetup();

	/* Perform basic analog initialization to defaults */
	AnalogSetDefault();

}
