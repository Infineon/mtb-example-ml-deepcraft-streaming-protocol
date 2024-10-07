/******************************************************************************
* File Name:   clock.c
*
* Description: This file provides a simple millisecond clock.
*
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

#include "cyhal.h"
#include "clock.h"


/*******************************************************************************
* Static Variables
*******************************************************************************/
static cyhal_timer_t timer_obj;
static size_t last_t = 0;
static size_t seconds = 0;


/*******************************************************************************
* Function Definitions
*******************************************************************************/

void clock_init()
{
    cy_rslt_t result;
    /* Timer object used */
    const cyhal_timer_cfg_t timer_cfg =
    {
        .compare_value = 0,                 /* Timer compare value, not used */
        .period = 10000,                    /* Timer period set to a large enough value compared to event being measured */
        .direction = CYHAL_TIMER_DIR_UP,    /* Timer counts up */
        .is_compare = false,                /* Don't use compare mode */
        .is_continuous = true,              /* Run timer indefinitely */
        .value = 0                          /* Initial value of counter */
    };
    /* Initialize the timer object. Does not use pin output ('pin' is NC) and
     * does not use a pre-configured clock source ('clk' is NULL). */
    result = cyhal_timer_init(&timer_obj, NC, NULL);
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Apply timer configuration such as period, count direction, run mode, etc. */
    result = cyhal_timer_configure(&timer_obj, &timer_cfg);
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Set the frequency of timer to 10000 counts in a second or 10000 Hz */
    result = cyhal_timer_set_frequency(&timer_obj, 10000);
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Start the timer with the configured settings */
    result = cyhal_timer_start(&timer_obj);
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }
}

void clock_update()
{
    size_t t = cyhal_timer_read(&timer_obj);
    if (t < last_t)
    {
        seconds++;
    }
    last_t = t;
}

uint32_t clock_get_ms()
{
    return 1000 * seconds + last_t / 10;
}
