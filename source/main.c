/******************************************************************************
* File Name:   main.c
*
* Description: Implementation of Tensor Streaming Protocol firmware for PSOC 6.
*
* Related Document:
*    See README.md and https://bitbucket.org/imagimob/tensor-streaming-protocol
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

#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include <cyhal.h>
#include <stdio.h>

#include "protocol/protocol.h"
#include "usbd.h"
#include "build.h"
#include "common.h"
#include "clock.h"
#include "board.h"
#include "system.h"
#include "watchdog.h"

/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
*  This is the main function. It never returns.
*
*******************************************************************************/
int main(void)
{
    uint32_t clock_hz;

    /* Base system initialization  */
    board_init_system();

    /* Override the base PLL clocks and routing. */
    board_set_clocks();

    /* Start clock */
    if(!clock_init())
    {
        halt_error(LED_CODE_CLOCK_ERROR);
    }

    /* Initialize retarget-io to use the debug UART port */
    if(!board_enable_debug_console())
    {
        halt_error(LED_CODE_STDOUT_RETARGET_ERROR);
    }

    /* Firmware version */
    protocol_Version firmware_version = {
        .major = 1,
        .minor = 2,
        .build = BUILD_DATE,
        .revision = BUILD_TIME
    };

    /* Serial UUID */
    uint8_t* serial = board_get_serial_uuid();

    /* Create a protocol instance */
    protocol_t* protocol = protocol_create("PSOC 6 AI (CY8CKIT-06S2-AI)", serial, firmware_version);

    /* Add reset function */
    protocol->board_reset = board_reset;

    /* Debug console print */
    printf("\x1b[2J\x1b[;H");
    printf("*********** Firmware Debug Console ***********\n\n");
    printf("Board Name             : %s\r\n", protocol->board.name);
    printf("Board Serial           : %02X%02X%02X%02X-%02X%02X-%02X%02X-%02X%02X-%02X%02X%02X%02X%02X%02X\n",
        serial[0], serial[1], serial[2], serial[3],
        serial[4], serial[5],
        serial[6], serial[7],
        serial[8], serial[9],
        serial[10], serial[11], serial[12], serial[13], serial[14], serial[15]);
    printf("Firmware Version       : %" PRIu32 ".%" PRIu32 ".%" PRIu32 ".%" PRIu32 "\n\n",
        protocol->board.firmware_version.major,
        protocol->board.firmware_version.minor,
        protocol->board.firmware_version.build,
        protocol->board.firmware_version.revision);
    printf("Protocol Version       : %" PRIu32 ".%" PRIu32 ".%" PRIu32 ".%" PRIu32 "\n\n",
        protocol->board.protocol_version.major,
        protocol->board.protocol_version.minor,
        protocol->board.protocol_version.build,
        protocol->board.protocol_version.revision);

#ifdef IM_ENABLE_WATCHDOG
    /* For watchdog auto reset every 3 second...  */
    if(!watchdog_enable(protocol, 3000))
    {
        halt_error(LED_CODE_WATCHDOG_ERROR);
    }
#endif

    /* Load all device drivers */
    system_load_device_drivers(protocol);

    /* Initialize the streaming interface */
    usbd_t* usb = usbd_create(protocol);

    /* Show some of the clocks we have. I am interested in the fast_clock and the peripheral clock */
    clock_hz = cyhal_clock_get_frequency(&CYHAL_CLOCK_FLL);
    printf("@init fll clock = %" PRIu32 "\n", clock_hz );

    clock_hz = cyhal_clock_get_frequency(&CYHAL_CLOCK_PLL[0]);
    printf("@init pll_0 clock = %" PRIu32 "\n", clock_hz );

    clock_hz = cyhal_clock_get_frequency(&CYHAL_CLOCK_PLL[1]);
    printf("@init pll_1 clock = %" PRIu32 "\n", clock_hz );

    clock_hz = cyhal_clock_get_frequency(&CYHAL_CLOCK_HF[0]);
    printf("@init HF[0]_clock = %" PRIu32 "\n", clock_hz );

    clock_hz = cyhal_clock_get_frequency(&CYHAL_CLOCK_HF[1]);
    printf("@init audio_clock HF[1] = %" PRIu32 "\n", clock_hz );

    clock_hz = cyhal_clock_get_frequency(&CYHAL_CLOCK_PERI);
    printf("@init peri_clock = %" PRIu32 "\n", clock_hz );

    clock_hz = cyhal_clock_get_frequency(&CYHAL_CLOCK_FAST);
    printf("@init fast_clock = %" PRIu32 "\n", clock_hz );

    clock_hz = cyhal_clock_get_frequency(&CYHAL_CLOCK_TIMER);
    printf("@init timer_clock = %" PRIu32 "\n", clock_hz );

    clock_hz = cyhal_clock_get_frequency(&CYHAL_CLOCK_SLOW);
    printf("@init slow_clock = %" PRIu32 "\n", clock_hz );

    clock_hz = cyhal_clock_get_frequency(&CYHAL_CLOCK_ECO);
    printf("@init ECO_clock = %" PRIu32 "\n", clock_hz );


    printf("Ready accepting commands.\r\n");
    set_led(false);

    for (;;)
    {
        protocol_process_request(protocol, &usb->istream, &usb->ostream);
    }
}

/* [] END OF FILE */
