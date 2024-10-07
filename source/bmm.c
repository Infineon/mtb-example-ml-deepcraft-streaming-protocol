/******************************************************************************
* File Name:   bmm.c
*
* Description: This file implements the interface with the magnetometer sensor,
*               as a timer to feed the pre-processor at 50Hz.
*
* Related Document: See README.md
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

#include "bmm.h"
#include "config.h"
#include "cyhal.h"
#include "cybsp.h"
#include "mtb_bmm350.h"
/*******************************************************************************
* Macros
*******************************************************************************/
#define I2C_TIMEOUT_MS          (1U)
#define BMM350_SCAN_RATE        50
#define BMM350_TIMER_FREQUENCY  100000
#define BMM350_TIMER_PERIOD     (BMM350_TIMER_FREQUENCY/BMM350_SCAN_RATE)
#define BMM350_TIMER_PRIORITY   3

#ifdef CY_XSS_BMM350
#define BMM350_ADDRESS (MTB_BMM350_ADDRESS_DEFAULT)
#else
#define BMM350_ADDRESS (MTB_BMM350_ADDRESS_SEC)
#endif

/*******************************************************************************
* Global Variables
*******************************************************************************/
#ifdef CY_MAG_BMM350
mtb_bmm350_t dev;
#endif
/* timer used for getting data */
cyhal_timer_t bmm350_timer;

/*******************************************************************************
* Function Prototypes
*******************************************************************************/
void bmm350_timer_intr_handler(void* callback_arg, cyhal_timer_event_t event);
cy_rslt_t bmm350_timer_init(void);


/*******************************************************************************
* Function Name: mag_sensor_init
********************************************************************************
* Summary:
*    A function used to initialize the magnetometer based on the shield selected 
*    in the makefile. Starts a timer that triggers an interrupt at 50Hz.
*
* Parameters:
*   None
*
* Return:
*     The status of the initialization.
*
*
*******************************************************************************/
cy_rslt_t mag_sensor_init(void)
{
    cy_rslt_t result;
#ifdef CY_MAG_BMM350
    /* Initialize BMM350 */
    result = mtb_bmm350_init_i2c(&dev, &i2c, BMM350_ADDRESS);
    if(CY_RSLT_SUCCESS != result)
    {
        return result;
    }
    bmm_flag = false;
#endif

    /* Timer for data collection */
    result = bmm350_timer_init();
    if(CY_RSLT_SUCCESS != result)
    {
        return result;
    }

    return CY_RSLT_SUCCESS;
}


/*******************************************************************************
* Function Name: bmm350_timer_init
********************************************************************************
* Summary:
*   Sets up an interrupt that triggers at the desired frequency.
*
* Returns:
*   The status of the initialization.
*
*
*******************************************************************************/
cy_rslt_t bmm350_timer_init(void)
{
    cy_rslt_t rslt;
    const cyhal_timer_cfg_t timer_cfg =
    {
        .compare_value = 0,                 /* Timer compare value, not used */
        .period = BMM350_TIMER_PERIOD,      /* Defines the timer period */
        .direction = CYHAL_TIMER_DIR_UP,    /* Timer counts up */
        .is_compare = false,                /* Don't use compare mode */
        .is_continuous = true,              /* Run the timer indefinitely */
        .value = 0                          /* Initial value of counter */
    };

    /* Initialize the timer object. Does not use pin output ('pin' is NC) and
    * does not use a pre-configured clock source ('clk' is NULL). */
    rslt = cyhal_timer_init(&bmm350_timer, NC, NULL);
    if (CY_RSLT_SUCCESS != rslt)
    {
        return rslt;
    }

    /* Apply timer configuration such as period, count direction, run mode, etc. */
    rslt = cyhal_timer_configure(&bmm350_timer, &timer_cfg);
    if (CY_RSLT_SUCCESS != rslt)
    {
        return rslt;
    }

    /* Set the frequency of timer to 100KHz */
    rslt = cyhal_timer_set_frequency(&bmm350_timer, BMM350_TIMER_FREQUENCY);
    if (CY_RSLT_SUCCESS != rslt)
    {
        return rslt;
    }

    /* Assign the ISR to execute on timer interrupt */
    cyhal_timer_register_callback(&bmm350_timer, bmm350_timer_intr_handler, NULL);

    /* Set the event on which timer interrupt occurs and enable it */
    cyhal_timer_enable_event(&bmm350_timer, CYHAL_TIMER_IRQ_TERMINAL_COUNT, BMM350_TIMER_PRIORITY, true);
    
    /* Start the timer with the configured settings */
    rslt = cyhal_timer_start(&bmm350_timer);
    if (CY_RSLT_SUCCESS != rslt)
    {
        return rslt;
    }

    return CY_RSLT_SUCCESS;
}


/*******************************************************************************
* Function Name: bmm350_timer_intr_handler
********************************************************************************
* Summary:
*   Interrupt handler for timer. Interrupt handler will get called at 50Hz and
*   sets a flag that can be checked in main.
*
* Parameters:
*     callback_arg: not used
*     event: not used
*
*
*******************************************************************************/
void bmm350_timer_intr_handler(void *callback_arg, cyhal_timer_event_t event)
{
    (void) callback_arg;
    (void) event;

    bmm_flag = true;
}


/*******************************************************************************
* Function Name: bmm350_get_data
********************************************************************************
* Summary:
*   Reads Magnetometer data from the magnetometer and stores it in a buffer.
*
* Parameters:
*     bmm_data: Stores Magnetometer data
*
*
*******************************************************************************/
void bmm350_get_data(float *bmm_data)
{
#ifdef CY_MAG_BMM350
    /* Read data from BMM sensor */
    cy_rslt_t result;
    mtb_bmm350_data_t data1;
    result = mtb_bmm350_read(&dev,&data1);
    if (CY_RSLT_SUCCESS == result)
    {
        bmm_data[0] = data1.sensor_data.y;
        bmm_data[1] = data1.sensor_data.x;
        bmm_data[2] = data1.sensor_data.z;
    }
#endif
}
