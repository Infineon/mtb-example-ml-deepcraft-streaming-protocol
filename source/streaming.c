/******************************************************************************
* File Name:   streaming.c
*
* Description: This file contains functions for streaming data over a serial
*              interface. It supports using either USB CDC (default) or UART
*              over the debug port. See comment below for how to switch to the
*              debug port UART.
*
*******************************************************************************
* Copyright 2024, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/

#include "streaming.h"

/* SUPPORT FOR USB CDC AND DEBUG UART
 * ==================================
 * This file contains functions for streaming both through the USB CDC and
 * through the debug UART; however streaming through the debug UART is unstable
 * and NOT recommended. It's not clear if the problems happen on the device or
 * on the host PC. Problems we have seen include:
 * - Intermittent large data chunks buffered causing halts and high latency
 * - Dropped bytes, causing byte flips (LSB <=> MSB) and thus distorted data
 * - Triggers a bug in .NET SerialPort, causing complete crash of the Studio
 *
 * If this didn't scare you and you still want to try it, to enable streaming
 * through debug UART:
 * - Remove USBD_BASE from COMPONENT in the Makefile (will enable the lower
 *   part of this file
 * - Remove the emusb-device library using the Library Manager
 * - Remove the imports folder (remnants from the emusb-device) */


/******************************************************************************
*
* USB CDC streaming functions
*
******************************************************************************/

#ifdef COMPONENT_USBD_BASE

#include "USB.h"
#include "USB_CDC.h"


/*******************************************************************************
* Local Type Declarations
*******************************************************************************/
static const USB_DEVICE_INFO usb_deviceInfo = {
    0x058B,                       /* VendorId    */
    0x027D,                       /* ProductId    */
    "Infineon Technologies",      /* VendorName   */
    "Imagimob Streamer Example",  /* ProductName  */
    "12345678"                    /* SerialNumber */
};


/*******************************************************************************
* Local Variables
*******************************************************************************/
static USB_CDC_HANDLE usb_cdcHandle;


/*******************************************************************************
* Local Function Prototypes
*******************************************************************************/
static void streaming_usb_add_cdc(void);


/*******************************************************************************
* Function Definitions
*******************************************************************************/

/*******************************************************************************
* Function Name: streaming_init
********************************************************************************
* Summary:
*  Initializes the streaming interface. Call this once before using any other
*  function in this file.
*
*******************************************************************************/
void streaming_init()
{
    /* Initializes the USB stack */
    USBD_Init();

    /* Endpoint Initialization for CDC class */
    streaming_usb_add_cdc();

    /* Set device info used in enumeration */
    USBD_SetDeviceInfo(&usb_deviceInfo);

    /* Start the USB stack */
    USBD_Start();

    /* Wait for the usb to be configured (including connected physically */
    while ((USBD_GetState() & USB_STAT_CONFIGURED) != USB_STAT_CONFIGURED)
    {
        cyhal_system_delay_ms(50U); /* 50ms delay */
    }
}

/*******************************************************************************
* Function Name: streaming_receive
********************************************************************************
* Summary:
*  Reads bytes from the streaming interface into the given buffer if available.
*  This function may block for up to 1 ms.
**
* Parameters:
*  data: pointer to buffer where data will be stored
*  size: buffer size
*
* Return:
*  The number of bytes received; 0 if no bytes were available.
*
*******************************************************************************/
size_t streaming_receive(void* data, size_t size)
{
    /* Wait for 1 character for up to 1 ms */
    /* TODO: Enable callback on receive to set a flag, and here, if the flag is
     * set, use USBD_CDC_Receive() to obtain all available bytes at the same
     * time. Note that USBD_CDC_GetNumBytesInBuffer() seems to always return 0.
     */
    return USBD_CDC_Read(usb_cdcHandle, data, 1, 1);
}

/*******************************************************************************
* Function Name: streaming_send
********************************************************************************
* Summary:
*  Sends the given bytes to the streaming interface. This function will block
*  until transmission is complete.
*
* Parameters:
*  data: pointer to data to send
*  size: number of bytes to send
*
*******************************************************************************/
void streaming_send(const void* data, size_t size)
{
    /* Write and block until write is complete */
    USBD_CDC_Write(usb_cdcHandle, data, size, 0);
    USBD_CDC_WaitForTX(usb_cdcHandle, 0);
}

/*******************************************************************************
* Function Name: streaming_usb_add_cdc
********************************************************************************
* Summary:
*  Initializes USB CDC.
*
*******************************************************************************/
static void streaming_usb_add_cdc(void)
{
    static U8             OutBuffer[USB_FS_BULK_MAX_PACKET_SIZE];
    USB_CDC_INIT_DATA     InitData;
    USB_ADD_EP_INFO       EPBulkIn;
    USB_ADD_EP_INFO       EPBulkOut;
    USB_ADD_EP_INFO       EPIntIn;

    memset(&InitData, 0, sizeof(InitData));
    EPBulkIn.Flags          = 0;                             /* Flags not used */
    EPBulkIn.InDir          = USB_DIR_IN;                    /* IN direction (Device to Host) */
    EPBulkIn.Interval       = 0;                             /* Interval not used for Bulk endpoints */
    EPBulkIn.MaxPacketSize  = USB_FS_BULK_MAX_PACKET_SIZE;   /* Maximum packet size (64B for Bulk in full-speed) */
    EPBulkIn.TransferType   = USB_TRANSFER_TYPE_BULK;        /* Endpoint type - Bulk */
    InitData.EPIn  = USBD_AddEPEx(&EPBulkIn, NULL, 0);

    EPBulkOut.Flags         = 0;                             /* Flags not used */
    EPBulkOut.InDir         = USB_DIR_OUT;                   /* OUT direction (Host to Device) */
    EPBulkOut.Interval      = 0;                             /* Interval not used for Bulk endpoints */
    EPBulkOut.MaxPacketSize = USB_FS_BULK_MAX_PACKET_SIZE;   /* Maximum packet size (64B for Bulk in full-speed) */
    EPBulkOut.TransferType  = USB_TRANSFER_TYPE_BULK;        /* Endpoint type - Bulk */
    InitData.EPOut = USBD_AddEPEx(&EPBulkOut, OutBuffer, sizeof(OutBuffer));

    EPIntIn.Flags           = 0;                             /* Flags not used */
    EPIntIn.InDir           = USB_DIR_IN;                    /* IN direction (Device to Host) */
    EPIntIn.Interval        = 64;                            /* Interval of 8 ms (64 * 125us) */
    EPIntIn.MaxPacketSize   = USB_FS_INT_MAX_PACKET_SIZE ;   /* Maximum packet size (64 for Interrupt) */
    EPIntIn.TransferType    = USB_TRANSFER_TYPE_INT;         /* Endpoint type - Interrupt */
    InitData.EPInt = USBD_AddEPEx(&EPIntIn, NULL, 0);

    usb_cdcHandle = USBD_CDC_Add(&InitData);
}


#else

/******************************************************************************
*
* Debug UART streaming functions
*
******************************************************************************/

#include "cybsp.h"
#include "cyhal_uart.h"


/*******************************************************************************
* Macros
*******************************************************************************/
#define UART_BAUD_RATE              (1000000u)
/* NOTE: The debug UART may not support standard baud rates like 921600 due to
 * its clock configuration, so consider this before changing. It should work
 * well with 500000 and 1000000. I'm not sure why, but possibly because it's
 * based on the 32 MHz MPU clock, so there's no integer divider for e.g.
 * 921600. */
#define RX_BUF_SIZE                 (32u)
/* Should be enough for a complete protocol command. */

/*******************************************************************************
* Local Variables
*******************************************************************************/
static cyhal_uart_t  uart_obj;
static uint8_t       uart_rx_buffer[RX_BUF_SIZE];
static volatile bool uart_busy = false;


/*******************************************************************************
* Function Definitions
*******************************************************************************/

/*******************************************************************************
* Function Name: streaming_uart_event_handler
********************************************************************************
* Summary:
*  UART TX and RX completion handler.
*
*******************************************************************************/
static void streaming_uart_event_handler(void* handler_arg, cyhal_uart_event_t event)
{
    (void)handler_arg;

    /* RX or TX done; release any waiting RX or TX */
    uart_busy = false;
}

/*******************************************************************************
* Function Name: streaming_init
********************************************************************************
* Summary:
*  Initializes the streaming interface. Call this once before using any other
*  function in this file.
*
*******************************************************************************/
void streaming_init()
{
    cy_rslt_t result;

    /* UART configuration structure */
    const cyhal_uart_cfg_t uart_config =
    {
        .data_bits       = 8,
        .stop_bits       = 1,
        .parity          = CYHAL_UART_PARITY_NONE,
        .rx_buffer       = uart_rx_buffer,
        .rx_buffer_size  = RX_BUF_SIZE,
    };

    /* Initialize the UART */
    result = cyhal_uart_init(&uart_obj, CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX, NC, NC, NULL, &uart_config);
    HALT_ON_ERROR(result);
    result = cyhal_uart_set_baud(&uart_obj, UART_BAUD_RATE, NULL);
    HALT_ON_ERROR(result);

    /* Register and enable callback */
    cyhal_uart_register_callback(&uart_obj, streaming_uart_event_handler, NULL);
    cyhal_uart_event_t events = (cyhal_uart_event_t)(
        CYHAL_UART_IRQ_TX_DONE | CYHAL_UART_IRQ_TX_ERROR
        | CYHAL_UART_IRQ_RX_DONE | CYHAL_UART_IRQ_RX_ERROR);
    cyhal_uart_enable_event(&uart_obj, events, CYHAL_ISR_PRIORITY_DEFAULT, true);
}

/*******************************************************************************
* Function Name: streaming_receive
********************************************************************************
* Summary:
*  Reads bytes from the streaming interface into the given buffer if available.
*  This function may block until a preceding UART operation is complete.
**
* Parameters:
*  data: pointer to buffer where data will be stored
*  size: buffer size
*
* Return:
*  The number of bytes received; 0 if no bytes were available.
*
*******************************************************************************/
size_t streaming_receive(void* data, size_t size)
{
    /* Ensure UART available */
    while (uart_busy)
        cyhal_system_delay_ms(1);
    uart_busy = true;

    /* Do read */
    cy_rslt_t result = cyhal_uart_read(&uart_obj, data, &size);
    uart_busy = false; /* TODO Unclear if this is necessary; should probably be removed */
    if (result != CY_RSLT_SUCCESS)
        return 0;
    return size;
}

/*******************************************************************************
* Function Name: streaming_send
********************************************************************************
* Summary:
*  Sends the given bytes to the streaming interface. This function may block
*  until a preceding UART operation is complete.
*
* Parameters:
*  data: pointer to data to send
*  size: number of bytes to send
*
*******************************************************************************/
void streaming_send(const void* data, size_t size)
{
    /* Ensure UART available */
    while (uart_busy)
        cyhal_system_delay_ms(1);
    uart_busy = true;

    /* Do write */
    cyhal_uart_write_async(&uart_obj, (void*)data, size);
}

#endif
