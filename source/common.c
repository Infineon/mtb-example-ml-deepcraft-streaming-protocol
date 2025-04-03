/******************************************************************************
* File Name: common.c
*
* Description:
*   This file contains common utility functions used across the project.
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

#include <cy_result.h>
#include <cy_utils.h>
#include <cyhal.h>
#include <cybsp.h>

#include "common.h"

/*******************************************************************************
* Function Name: set_led
********************************************************************************
* Summary:
*   Sets the status LED.
*
* Parameters:
*   state: The desired state of the LED. 'true' for ON, 'false' for OFF.
*
*******************************************************************************/
void set_led(bool state)
{
    cyhal_gpio_write(CYBSP_USER_LED, state ? CYBSP_LED_STATE_OFF : CYBSP_LED_STATE_ON);
}

/*******************************************************************************
* Function Name: halt_error
********************************************************************************
* Summary:
*  Flash the given error code indefinitely. This function never returns.
*
*  Blink pattern:
*  0:  .        10: ..-      20: .--.
*  1:  -        11: -.-      21: ---.
*  2:  ..       12: .--      22: ...-
*  3:  -.       13: ---      23: -..-
*  4:  .-       14: ....     24: .-.-
*  5:  --       15: -...     25: --.-
*  6:  ...      16: .-..     26: ..--
*  7:  -..      17: --..     27: -.--
*  8:  .-.      18: ..-.     28: .---
*  9:  --.      19: -.-.     29: ----
*******************************************************************************/
void halt_error(int code)
{
    __disable_irq();

    if(code < 0)
    {
        code = -code;
    }

    do {
        int i = code + 2;
        do {
            set_led(true);
            Cy_SysLib_Delay((i & 1) ? LED_LONG_ON_TIME : LED_SHORT_ON_TIME);
            set_led(false);
            Cy_SysLib_Delay(LED_OFF_TIME);
            i >>= 1;
        } while(i > 1);

        Cy_SysLib_Delay(LED_CODE_SEPARATOR_TIME);
    } while(true);
}

/* [] END OF FILE */
