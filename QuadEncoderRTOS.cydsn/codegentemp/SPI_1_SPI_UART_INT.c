/***************************************************************************//**
* \file SPI_1_SPI_UART_INT.c
* \version 4.0
*
* \brief
*  This file provides the source code to the Interrupt Service Routine for
*  the SCB Component in SPI and UART modes.
*
* Note:
*
********************************************************************************
* \copyright
* Copyright 2013-2017, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "SPI_1_PVT.h"
#include "SPI_1_SPI_UART_PVT.h"


#if (SPI_1_SCB_IRQ_INTERNAL)
/*******************************************************************************
* Function Name: SPI_1_SPI_UART_ISR
****************************************************************************//**
*
*  Handles the Interrupt Service Routine for the SCB SPI or UART modes.
*
*******************************************************************************/
CY_ISR(SPI_1_SPI_UART_ISR)
{
#if (SPI_1_INTERNAL_RX_SW_BUFFER_CONST)
    uint32 locHead;
#endif /* (SPI_1_INTERNAL_RX_SW_BUFFER_CONST) */

#if (SPI_1_INTERNAL_TX_SW_BUFFER_CONST)
    uint32 locTail;
#endif /* (SPI_1_INTERNAL_TX_SW_BUFFER_CONST) */

#ifdef SPI_1_SPI_UART_ISR_ENTRY_CALLBACK
    SPI_1_SPI_UART_ISR_EntryCallback();
#endif /* SPI_1_SPI_UART_ISR_ENTRY_CALLBACK */

    if (NULL != SPI_1_customIntrHandler)
    {
        SPI_1_customIntrHandler();
    }

    #if(SPI_1_CHECK_SPI_WAKE_ENABLE)
    {
        /* Clear SPI wakeup source */
        SPI_1_ClearSpiExtClkInterruptSource(SPI_1_INTR_SPI_EC_WAKE_UP);
    }
    #endif

    #if (SPI_1_CHECK_RX_SW_BUFFER)
    {
        if (SPI_1_CHECK_INTR_RX_MASKED(SPI_1_INTR_RX_NOT_EMPTY))
        {
            do
            {
                /* Move local head index */
                locHead = (SPI_1_rxBufferHead + 1u);

                /* Adjust local head index */
                if (SPI_1_INTERNAL_RX_BUFFER_SIZE == locHead)
                {
                    locHead = 0u;
                }

                if (locHead == SPI_1_rxBufferTail)
                {
                    #if (SPI_1_CHECK_UART_RTS_CONTROL_FLOW)
                    {
                        /* There is no space in the software buffer - disable the
                        * RX Not Empty interrupt source. The data elements are
                        * still being received into the RX FIFO until the RTS signal
                        * stops the transmitter. After the data element is read from the
                        * buffer, the RX Not Empty interrupt source is enabled to
                        * move the next data element in the software buffer.
                        */
                        SPI_1_INTR_RX_MASK_REG &= ~SPI_1_INTR_RX_NOT_EMPTY;
                        break;
                    }
                    #else
                    {
                        /* Overflow: through away received data element */
                        (void) SPI_1_RX_FIFO_RD_REG;
                        SPI_1_rxBufferOverflow = (uint8) SPI_1_INTR_RX_OVERFLOW;
                    }
                    #endif
                }
                else
                {
                    /* Store received data */
                    SPI_1_PutWordInRxBuffer(locHead, SPI_1_RX_FIFO_RD_REG);

                    /* Move head index */
                    SPI_1_rxBufferHead = locHead;
                }
            }
            while(0u != SPI_1_GET_RX_FIFO_ENTRIES);

            SPI_1_ClearRxInterruptSource(SPI_1_INTR_RX_NOT_EMPTY);
        }
    }
    #endif


    #if (SPI_1_CHECK_TX_SW_BUFFER)
    {
        if (SPI_1_CHECK_INTR_TX_MASKED(SPI_1_INTR_TX_NOT_FULL))
        {
            do
            {
                /* Check for room in TX software buffer */
                if (SPI_1_txBufferHead != SPI_1_txBufferTail)
                {
                    /* Move local tail index */
                    locTail = (SPI_1_txBufferTail + 1u);

                    /* Adjust local tail index */
                    if (SPI_1_TX_BUFFER_SIZE == locTail)
                    {
                        locTail = 0u;
                    }

                    /* Put data into TX FIFO */
                    SPI_1_TX_FIFO_WR_REG = SPI_1_GetWordFromTxBuffer(locTail);

                    /* Move tail index */
                    SPI_1_txBufferTail = locTail;
                }
                else
                {
                    /* TX software buffer is empty: complete transfer */
                    SPI_1_DISABLE_INTR_TX(SPI_1_INTR_TX_NOT_FULL);
                    break;
                }
            }
            while (SPI_1_SPI_UART_FIFO_SIZE != SPI_1_GET_TX_FIFO_ENTRIES);

            SPI_1_ClearTxInterruptSource(SPI_1_INTR_TX_NOT_FULL);
        }
    }
    #endif

#ifdef SPI_1_SPI_UART_ISR_EXIT_CALLBACK
    SPI_1_SPI_UART_ISR_ExitCallback();
#endif /* SPI_1_SPI_UART_ISR_EXIT_CALLBACK */

}

#endif /* (SPI_1_SCB_IRQ_INTERNAL) */


/* [] END OF FILE */
