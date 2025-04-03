/******************************************************************************
* File Name:   clock.c
*
* Description: This file provides a clock.
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
#include "clock.h"

/*******************************************************************************
* Static Variables
*******************************************************************************/
static cyhal_timer_t timer_obj;
static uint32_t uptime_seconds = 0;

/*******************************************************************************
* Function Definitions
*******************************************************************************/

/*******************************************************************************
* Function Name: clock_interrupt_handler
********************************************************************************
* Summary:
*   Keeps track of total elapsed seconds.
*
* Parameters:
*   callback_arg: User defined argument. Not used.
*   event: Timer/counter interrupt trigger.
*
*******************************************************************************/
static void clock_interrupt_handler(void *callback_arg, cyhal_timer_event_t event)
{
    (void) callback_arg;
    (void) event;

    uptime_seconds++;
}

/*******************************************************************************
* Function Name: clock_get_tick
********************************************************************************
* Summary:
*   Returns the number of clock ticks elapsed since initialization.
*
* Return:
*   The number of clock ticks elapsed.
*
*******************************************************************************/
clock_tick_t clock_get_tick(void)
{
    /* Ensure it can be reread in case of a corner case. */
    volatile uint32_t* p_sec = (uint32_t*)&uptime_seconds;

    uint32_t seconds_0 = *p_sec;
    uint32_t time = cyhal_timer_read(&timer_obj);
    uint32_t seconds_1 = *p_sec;

    /* If an interrupt appears here and increments the second counter this will
    * add another erroneous isecond or 100 000 ticks!
    * Is the solution to reread and if seconds have incremented assume time = 0. */
    if ( seconds_0 == seconds_1 )
    {
        return ((clock_tick_t)CLOCK_TICK_PER_SECOND * seconds_1) + (clock_tick_t)time;
    }

    /* If the latter differ from the first, then assume time is zero. */
    return ((clock_tick_t)CLOCK_TICK_PER_SECOND * seconds_1);
}

/*******************************************************************************
* Function Name: clock_init
********************************************************************************
* Summary:
*   Initializes the clock.
*
* Return:
*   True if registration is successful, otherwise false.
*
*******************************************************************************/
bool clock_init(void)
{
    cy_rslt_t result;

    /* Timer object used */
    const cyhal_timer_cfg_t timer_cfg =
    {
        .compare_value = 0,                   /* Timer compare value, not used */
        .period = CLOCK_TICK_PER_SECOND - 1,  /* Timer period set to a large enough value compared to event being measured */
        .direction = CYHAL_TIMER_DIR_UP,      /* Timer counts up */
        .is_compare = false,                  /* Don't use compare mode */
        .is_continuous = true,                /* Run timer indefinitely */
        .value = 0                            /* Initial value of counter */
    };

    /* Initialize the timer object. Does not use pin output ('pin' is NC) and
     * does not use a pre-configured clock source ('clk' is NULL). */
    result = cyhal_timer_init(&timer_obj, NC, NULL);
    if (result != CY_RSLT_SUCCESS)
    {
        return false;
    }

    /* Apply timer configuration such as period, count direction, run mode, etc. */
    result = cyhal_timer_configure(&timer_obj, &timer_cfg);
    if (result != CY_RSLT_SUCCESS)
    {
        return false;
    }

    /* Set the frequency of timer to CLOCK_TICK_PER_SECOND counts in a second */
    result = cyhal_timer_set_frequency(&timer_obj, CLOCK_TICK_PER_SECOND);
    if (result != CY_RSLT_SUCCESS)
    {
        return false;
    }

    /* Assign the ISR to execute on timer interrupt */
    cyhal_timer_register_callback(&timer_obj, clock_interrupt_handler, NULL);

    /* Set the event on which timer interrupt occurs and enable it */
    cyhal_timer_enable_event(&timer_obj, CYHAL_TIMER_IRQ_TERMINAL_COUNT, CLOCK_INTERRUPT_PRIORITY, true);

    /* Start the timer with the configured settings */
    result = cyhal_timer_start(&timer_obj);
    if (result != CY_RSLT_SUCCESS)
    {
        return false;
    }

    return true;
}

/* [] END OF FILE */
