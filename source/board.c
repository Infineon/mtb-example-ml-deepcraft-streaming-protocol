/******************************************************************************
* File Name:   board.c
*
* Description: This file provides basic board functionalities like
*         - reset
*         - debug console
*         - serial
*
*******************************************************************************
* Copyright 2024-2025, Cypress Semiconductor Corporation (an Infineon company) or
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

#include <cy_retarget_io.h>
#include <string.h>
#include <stdlib.h>
#include <cyhal.h>
#include <cybsp.h>

#include "protocol/protocol.h"
#include "board.h"

/*******************************************************************************
* Function Name: init_system
********************************************************************************
* Summary:
*   Initializes the system including device and board peripherals.
*******************************************************************************/
void board_init_system(void)
{
    cy_rslt_t result;

    /* Initialize the device and board peripherals */
    result = cybsp_init();
    if (result != CY_RSLT_SUCCESS)
    {
        CY_HALT();
    }

    /* Initialize the User LED */
    cyhal_gpio_init(CYBSP_USER_LED, CYHAL_GPIO_DIR_OUTPUT,
        CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);

    /* Enable global interrupts */
    __enable_irq();
}

/*******************************************************************************
* Function Name: enable_debug_console
********************************************************************************
* Summary:
*   Initializes the debug UART port for retarget-io.
*
* Return:
*   True if initialization is successful, otherwise false.
*******************************************************************************/
bool board_enable_debug_console(void)
{
    cy_rslt_t result;

    /* Initialize retarget-io to use the debug UART port */
    result = cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX, CY_RETARGET_IO_BAUDRATE);
    if(result != CY_RSLT_SUCCESS)
    {
        return false;
    }

    return true;
}


/*******************************************************************************
* Function Name: set_clocks
********************************************************************************
* Summary:
*   Reinitializes the clocks. FLL is dedicated for the audio done elseware.
*    PLL0 is not modified and it needs to be at 96Mhz for the usb clock.
*    PLL1 is set to 150 MHz and will drive the CPU Fast clock.
*    Peripheral speed will be half of the fast clock. I.e. 75 MHz
*    SPI clock can now be set to 75/4 = 18.75 MHz.
*
*
* Return:
*   True if initialization is successful, otherwise false.
*******************************************************************************/
bool board_set_clocks(void)
{
    cy_rslt_t result;
    cyhal_clock_t peri;
    cyhal_clock_t hf_0;
    cyhal_clock_t PLL_1;

    #define FAST_FREQ 150000000

    result = cyhal_clock_reserve(&peri, &CYHAL_CLOCK_PERI);
    if(result != CY_RSLT_SUCCESS)
    {
        return false;
    }

    result = cyhal_clock_reserve(&hf_0, &CYHAL_CLOCK_HF[0]);
    if(result != CY_RSLT_SUCCESS)
    {
        return false;
    }

    result = cyhal_clock_reserve(&PLL_1, &CYHAL_CLOCK_PLL[1]);
    if(result != CY_RSLT_SUCCESS)
    {
        return false;
    }

    /* Set the divider of peripheral clock first. */
    result = cyhal_clock_set_divider(&peri, 2);
    if(result != CY_RSLT_SUCCESS)
    {
        return false;
    }

    /* Set the frequency of PLL_1 and enable it. */
    result = cyhal_clock_set_frequency(&PLL_1, FAST_FREQ, NULL);
    if(result != CY_RSLT_SUCCESS)
    {
        return false;
    }

    result = cyhal_clock_set_enabled(&PLL_1, true, true);
    if(result != CY_RSLT_SUCCESS)
    {
        return false;
    }

    /* Set HF_0 clock to the source of PLL_1. */
    result = cyhal_clock_set_source(&hf_0, &CYHAL_CLOCK_PLL[1]);
    if(result != CY_RSLT_SUCCESS)
    {
        return false;
    }

    return true;
}


/*******************************************************************************
* Function Name: board_get_serial_uuid
********************************************************************************
* Summary:
*   Retrieves the unique serial UUID of the board.
*
* Return:
*   Pointer to the serial UUID array.
*******************************************************************************/
uint8_t* board_get_serial_uuid(void)
{
    /* Create serial UUID {290DE5CB-460B-41BF-XXXX-XXXXXXXXXXXX}. */
    /* Last part is silicon unique ID. */
    uint64_t serial64 = Cy_SysLib_GetUniqueId();
    static uint8_t serial[16] = {
            0x29, 0x0d, 0xe5, 0xcb, 0x46, 0x0b, 0x41, 0xbf,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
    };
    memcpy(serial + 8, &serial64, 8);

    return serial;
}

/*******************************************************************************
* Function Name: board_reset
********************************************************************************
* Summary:
*   Resets the board.
*
* Parameters:
*   protocol: Pointer to the protocol object.
*******************************************************************************/
void board_reset(protocol_t* protocol)
{
    UNUSED(protocol);

    NVIC_SystemReset();
}


/* [] END OF FILE */
