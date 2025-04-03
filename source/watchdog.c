/******************************************************************************
* File Name: watchdog.c
*
* Description:
*   Watchdog functionality
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

#include <cyhal.h>
#include <cybsp.h>

#include "protocol/protocol.h"
#include "watchdog.h"


/*******************************************************************************
* Function Name: watchdog_reset
********************************************************************************
* Summary:
*   Resets watchdog timer preventing the board from resetting.
*
* Parameters:
*   protocol: Pointer to the protocol object.
*******************************************************************************/
static void watchdog_reset(protocol_t* protocol)
{
    UNUSED(protocol);
    cyhal_wdt_kick(NULL);
}

/*******************************************************************************
* Function Name: watchdog_enable
********************************************************************************
* Summary:
*   Enable watchdog timer .
*
* Parameters:
*   protocol: Pointer to the protocol object.
*   timeout_msec: How often the board expects an reset message.
*      The actual reset timeout is 1.5 times this.
* Return:
*   True if initialization is successful, otherwise false.
*******************************************************************************/
bool watchdog_enable(protocol_t* protocol, int timeout_msec)
{
    cy_rslt_t result;
    static cyhal_wdt_t wdt;

    result = cyhal_wdt_init(&wdt, (uint32_t)(timeout_msec * 1.5));

    if (result != CY_RSLT_SUCCESS)
    {
        return false;
    }

    protocol_configure_watchdog(protocol, timeout_msec, watchdog_reset);

    return true;
}

/* [] END OF FILE */
