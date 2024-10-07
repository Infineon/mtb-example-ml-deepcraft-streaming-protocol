/******************************************************************************
* File Name:   protocol.c
*
* Description: This file implements the Imagimob streaming protocol.
*
* Related Document: See PROTOCOL.md
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

#include <stdio.h>
#include "clock.h"
#include "config.h"
#include "protocol.h"


/******************************************************************************
 * Macros
 *****************************************************************************/
#define RECEIVE_BUFFER_SIZE 32
#define HEARTBEAT_TIMEOUT_MS 5000


/*******************************************************************************
* Local Constants
*******************************************************************************/
static const char* TOO_LONG_COMMAND_MESSAGE = "ERROR:Too long command\r\n\0";
static const char* CONFIG_MESSAGE =
        "{\r\n"
        "    \"device_name\": \"PSoC6\",\r\n"
        "    \"protocol_version\": 1,\r\n"
        "    \"heartbeat_timeout\": 5,\r\n"
        "    \"sensors\": [\r\n"
        "        {\r\n"
        "            \"channel\": 1,\r\n"
        "            \"type\": \"microphone\",\r\n"
        "            \"datatype\": \"s16\",\r\n"
        "            \"shape\": [ 1024, 1 ],\r\n"
        "            \"rates\": [ 16000 ]\r\n"
#if IM_ENABLE_IMU
        "        },\r\n"
        "        {\r\n"
        "            \"channel\": 2,\r\n"
        "            \"type\": \"accelerometer\",\r\n"
        "            \"datatype\": \"f32\",\r\n"
        "            \"shape\": [ 1, 3 ],\r\n"
        "            \"rates\": [ 50 ]\r\n"
#endif
#if IM_ENABLE_BMM
        "        },\r\n"
        "        {\r\n"
        "            \"channel\": 3,\r\n"
        "            \"type\": \"Magnetometer\",\r\n"
        "            \"datatype\": \"f32\",\r\n"
        "            \"shape\": [ 1, 3 ],\r\n"
        "            \"rates\": [ 50 ]\r\n"
#endif
#if IM_ENABLE_RADAR
        "        },\r\n"
        "        {\r\n"
        "            \"channel\": 4,\r\n"
        "            \"type\": \"RADAR\",\r\n"
        "            \"datatype\": \"f32\",\r\n"
        "            \"shape\": [ 128, 1 ],\r\n"
        "            \"rates\": [ 100000 ]\r\n"
#endif
#if IM_ENABLE_DPS
        "        },\r\n"
        "        {\r\n"
        "            \"channel\": 5,\r\n"
        "            \"type\": \"DPS\",\r\n"
        "            \"datatype\": \"f32\",\r\n"
        "            \"shape\": [ 1, 2 ],\r\n"
        "            \"rates\": [ 50 ]\r\n"
#endif
        "        }\r\n"
        "    ]\r\n"
        "}\r\n\0";
static const char* OK_MESSAGE = "OK\r\n\0";
static const char* UNRECOGNIZED_COMMAND_MESSAGE = "ERROR:Unrecognized command\r\n\0";
static const uint8_t CRLF[2] = { '\r', '\n' };


/*******************************************************************************
* Local Variables
*******************************************************************************/
static char receive_buffer[RECEIVE_BUFFER_SIZE];
static char *receive_p = receive_buffer;
static volatile bool subscribe_audio = false;
static volatile bool subscribe_imu = false;
static volatile bool subscribe_bmm = false;
static volatile bool subscribe_dps = false;
static volatile bool subscribe_radar = false;
static uint32_t last_receive_time = 0;


/*******************************************************************************
* Function Definitions
*******************************************************************************/

/*******************************************************************************
* Function Name: protocol_init
********************************************************************************
* Summary:
*  Initializes the protocol. Call this once before using any other function in
*  this file.
*
*******************************************************************************/
void protocol_init()
{
    clock_init();
}

/*******************************************************************************
* Function Name: protocol_repl
********************************************************************************
* Summary:
*  Handles incoming characters, executes commands and sends response messages
*  in a read-eval-print loop. This function may block until a response
*  transmission is complete.
*
*******************************************************************************/
void protocol_repl()
{
    /* Update clock */
    clock_update();

    /* Read and handle data if any bytes are available */
    size_t bytes_read = streaming_receive(receive_p, RECEIVE_BUFFER_SIZE - (receive_p - receive_buffer));
    if (bytes_read)
    {
        /* Register receive time */
        last_receive_time = clock_get_ms();

        /* Echo incoming characters when not streaming data */
        /* Uncomment if desired!
        if (!subscribe_audio && !subscribe_imu)
            streaming_send(receive_p, bytes_read);
        */

        /* Advance receive pointer */
        receive_p += bytes_read;

        /* Check for \r\n at end */
        if (receive_p >= receive_buffer + 2 && *(receive_p - 2) == '\r' && *(receive_p - 1) == '\n')
        {
            /* Remove \r\n */
            *(receive_p - 2) = 0;

            /* config? */
            if (strcmp(receive_buffer, "config?") == 0)
            {
                subscribe_audio = false;
                subscribe_imu = false;
                subscribe_bmm = false;
                subscribe_radar = false;
                subscribe_dps = false;
                streaming_send(CONFIG_MESSAGE, strlen(CONFIG_MESSAGE));
            }
            /* subscribe,1,16000 */
            else if (strcmp(receive_buffer, "subscribe,1,16000") == 0)
            {
                subscribe_audio = true;
            }
            /* unsubscribe,1 */
            else if (strcmp(receive_buffer, "unsubscribe,1") == 0)
            {
                subscribe_audio = false;
                streaming_send(OK_MESSAGE, strlen(OK_MESSAGE));
            }
#if IM_ENABLE_IMU
            /* subscribe,2,50 */
            else if (strcmp(receive_buffer, "subscribe,2,50") == 0)
            {
                subscribe_imu = true;
            }
            /* unsubscribe,2 */
            else if (strcmp(receive_buffer, "unsubscribe,2") == 0)
            {
                subscribe_imu = false;
                streaming_send(OK_MESSAGE, strlen(OK_MESSAGE));
            }
#endif
#if IM_ENABLE_BMM
            /* subscribe,3,50 */
            else if (strcmp(receive_buffer, "subscribe,3,50") == 0)
            {
                subscribe_bmm = true;
            }
            /* unsubscribe,3 */
            else if (strcmp(receive_buffer, "unsubscribe,3") == 0)
            {
                subscribe_bmm = false;
                streaming_send(OK_MESSAGE, strlen(OK_MESSAGE));
            }
#endif
#if IM_ENABLE_RADAR
            /* subscribe,4,100000 */
            else if (strcmp(receive_buffer, "subscribe,4,100000") == 0)
            {
                subscribe_radar = true;
            }
            /* unsubscribe,4 */
            else if (strcmp(receive_buffer, "unsubscribe,4") == 0)
            {
                subscribe_radar = false;
                streaming_send(OK_MESSAGE, strlen(OK_MESSAGE));
            }
#endif
#if IM_ENABLE_DPS
            /* subscribe,5,50 */
            else if (strcmp(receive_buffer, "subscribe,5,50") == 0)
            {
                subscribe_dps = true;
            }
            /* unsubscribe,5 */
            else if (strcmp(receive_buffer, "unsubscribe,5") == 0)
            {
                subscribe_dps = false;
                streaming_send(OK_MESSAGE, strlen(OK_MESSAGE));
            }
#endif
            /* unsubscribe */
            else if (strcmp(receive_buffer, "unsubscribe") == 0)
            {
                subscribe_audio = false;
                subscribe_imu = false;
                subscribe_bmm = false;
                subscribe_radar = false;
                subscribe_dps = false;
                streaming_send(OK_MESSAGE, strlen(OK_MESSAGE));
            }
            /* empty command or heartbeat */
            else if (*receive_buffer == 0 || strcmp(receive_buffer, "heartbeat") == 0)
            {
                /* Nothing to do except register receive time, which was done above */
            }
            else
            {
                streaming_send(UNRECOGNIZED_COMMAND_MESSAGE, strlen(UNRECOGNIZED_COMMAND_MESSAGE));
            }

            /* Reset receive pointer; ready for new command */
            receive_p = receive_buffer;
        }

        /* Check end of buffer */
        if (receive_p == receive_buffer + RECEIVE_BUFFER_SIZE)
        {
            streaming_send(TOO_LONG_COMMAND_MESSAGE, strlen(TOO_LONG_COMMAND_MESSAGE));
            receive_p = receive_buffer;
        }
    }

    /* Check receive timeout: If no message for 5 seconds, stop streaming */
    if ((subscribe_audio || subscribe_imu || subscribe_bmm || subscribe_radar || subscribe_dps) && clock_get_ms() - last_receive_time > HEARTBEAT_TIMEOUT_MS)
    {
        subscribe_audio = false;
        subscribe_imu = false;
        subscribe_bmm = false;
        subscribe_radar = false;
        subscribe_dps = false;
    }
}

/*******************************************************************************
* Function Name: protocol_send
********************************************************************************
* Summary:
*  Sends a packet of data to the host. This function may block until the
*  transmission is complete.
*
* Parameters:
*  channel: the channel (1-9) to send the packet on
*  data: pointer to data to send
*  size: number of bytes to send
*
*******************************************************************************/
void protocol_send(uint8_t channel, const uint8_t* data, size_t size)
{
    uint8_t header[2] = { 'B', '0' + channel };
    switch (channel)
    {
    case PROTOCOL_AUDIO_CHANNEL:
        if (subscribe_audio)
        {
            streaming_send(header, 2);
            streaming_send(data, size);
            streaming_send(CRLF, 2);
        }
        break;
    case PROTOCOL_IMU_CHANNEL:
        if (subscribe_imu)
        {
            streaming_send(header, 2);
            streaming_send(data, size);
            streaming_send(CRLF, 2);
        }
        break;
    case PROTOCOL_BMM_CHANNEL:
        if (subscribe_bmm)
        {
            streaming_send(header, 2);
            streaming_send(data, size);
            streaming_send(CRLF, 2);
        }
        break;
    case PROTOCOL_RADAR_CHANNEL:
        if (subscribe_radar)
        {
            streaming_send(header, 2);
            streaming_send(data, size);
            streaming_send(CRLF, 2);
        }
        break;
    case PROTOCOL_DPS_CHANNEL:
        if (subscribe_dps)
        {
            streaming_send(header, 2);
            streaming_send(data, size);
            streaming_send(CRLF, 2);
        }
        break;
    }
}
