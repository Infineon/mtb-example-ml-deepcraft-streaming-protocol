/******************************************************************************
* File Name:   dev_bgt60trxx_settings.h
*
* Description: This file contains configuration settings for radar sensor.
*
* Related Document: See README.md
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

#ifndef XENSIV_BGT60TRXX_CONF_H
#define XENSIV_BGT60TRXX_CONF_H


struct radar_config {
    uint64_t start_freq;
    uint64_t end_freq;
    uint32_t samples_per_chirp;
    uint32_t chirps_per_frame;
    uint32_t rx_antennas;
    uint32_t tx_antennas;
    uint32_t sample_rate;
    float   chirp_repetition_time;
    float   frame_repetition_time;
    uint32_t frame_rate;
    uint32_t num_samples_per_frame;
    uint32_t fifo_int_level;
    uint32_t number_of_regs;
    uint32_t* register_list;
};

/* Simple radar configuration.. */
#define P0_XENSIV_BGT60TRXX_CONF_DEVICE (XENSIV_DEVICE_BGT60TR13C)
#define P0_XENSIV_BGT60TRXX_CONF_START_FREQ_HZ (61020099000)
#define P0_XENSIV_BGT60TRXX_CONF_END_FREQ_HZ (61479903000)
#define P0_XENSIV_BGT60TRXX_CONF_NUM_SAMPLES_PER_CHIRP (128)
#define P0_XENSIV_BGT60TRXX_CONF_NUM_CHIRPS_PER_FRAME (1)
#define P0_XENSIV_BGT60TRXX_CONF_NUM_RX_ANTENNAS (1)
#define P0_XENSIV_BGT60TRXX_CONF_NUM_TX_ANTENNAS (1)
#define P0_XENSIV_BGT60TRXX_CONF_SAMPLE_RATE (2352941)
#define P0_XENSIV_BGT60TRXX_CONF_CHIRP_REPETITION_TIME_S (6.21125e-05)
#define P0_XENSIV_BGT60TRXX_CONF_FRAME_REPETITION_TIME_S (0.00500407)
#define P0_XENSIV_BGT60TRXX_CONF_NUM_REGS (38)


uint32_t register_list_p0[] = {
    0x11e8270UL,
    0x3088210UL,
    0x9e967fdUL,
    0xb0805b4UL,
    0xd1027ffUL,
    0xf010700UL,
    0x11000000UL,
    0x13000000UL,
    0x15000000UL,
    0x17000be0UL,
    0x19000000UL,
    0x1b000000UL,
    0x1d000000UL,
    0x1f000b60UL,
    0x21103c51UL,
    0x231ff41fUL,
    0x25006f7bUL,
    0x2d000490UL,
    0x3b000480UL,
    0x49000480UL,
    0x57000480UL,
    0x5911be0eUL,
    0x5b44c40aUL,
    0x5d000000UL,
    0x5f787e1eUL,
    0x61f5208aUL,
    0x630000a4UL,
    0x65000252UL,
    0x67000080UL,
    0x69000000UL,
    0x6b000000UL,
    0x6d000000UL,
    0x6f093910UL,
    0x7f000100UL,
    0x8f000100UL,
    0x9f000100UL,
    0xad000000UL,
    0xb7000000UL,
};


/* Radar configuration for gesture detection */

#define P1_XENSIV_BGT60TRXX_CONF_DEVICE (XENSIV_DEVICE_BGT60TR13C)
#define P1_XENSIV_BGT60TRXX_CONF_START_FREQ_HZ (58500000000)
#define P1_XENSIV_BGT60TRXX_CONF_END_FREQ_HZ (62500000000)
#define P1_XENSIV_BGT60TRXX_CONF_NUM_SAMPLES_PER_CHIRP (64)
#define P1_XENSIV_BGT60TRXX_CONF_NUM_CHIRPS_PER_FRAME (32)
#define P1_XENSIV_BGT60TRXX_CONF_NUM_RX_ANTENNAS (3)
#define P1_XENSIV_BGT60TRXX_CONF_NUM_TX_ANTENNAS (1)
#define P1_XENSIV_BGT60TRXX_CONF_SAMPLE_RATE (2000000)
#define P1_XENSIV_BGT60TRXX_CONF_CHIRP_REPETITION_TIME_S (0.000299787)
#define P1_XENSIV_BGT60TRXX_CONF_FRAME_REPETITION_TIME_S (0.0300446)
#define P1_XENSIV_BGT60TRXX_CONF_NUM_REGS (38)

uint32_t register_list_p1[] = {
            0x11e8270UL,
            0x30a0210UL,
            0x9e967fdUL,
            0xb0805b4UL,
            0xd102bffUL,
            0xf010d00UL,
            0x11000000UL,
            0x13000000UL,
            0x15000000UL,
            0x17000be0UL,
            0x19000000UL,
            0x1b000000UL,
            0x1d000000UL,
            0x1f000b60UL,
            0x2113fc51UL,
            0x237ff41fUL,
            0x25000c63UL,
            0x2d000490UL,
            0x3b000480UL,
            0x49000480UL,
            0x57000480UL,
            0x5911be0eUL,
            0x5b56040aUL,
            0x5d01f000UL,
            0x5f787e1eUL,
            0x61b12902UL,
            0x6300091dUL,
            0x65000172UL,
            0x67000040UL,
            0x69000000UL,
            0x6b000000UL,
            0x6d000000UL,
            0x6f2a0b10UL,
            0x7f000100UL,
            0x8f000100UL,
            0x9f000100UL,
            0xad000000UL,
            0xb7000000UL
};


/* Radar configuration for presence detection.
 * This is the low framerate settings. */

#define P2_XENSIV_BGT60TRXX_CONF_DEVICE (XENSIV_DEVICE_BGT60TR13C)
#define P2_XENSIV_BGT60TRXX_CONF_START_FREQ_HZ (61020100000)
#define P2_XENSIV_BGT60TRXX_CONF_END_FREQ_HZ (61479904000)
#define P2_XENSIV_BGT60TRXX_CONF_NUM_SAMPLES_PER_CHIRP (128)
#define P2_XENSIV_BGT60TRXX_CONF_NUM_CHIRPS_PER_FRAME (16)
#define P2_XENSIV_BGT60TRXX_CONF_NUM_RX_ANTENNAS (1)
#define P2_XENSIV_BGT60TRXX_CONF_NUM_TX_ANTENNAS (1)
#define P2_XENSIV_BGT60TRXX_CONF_SAMPLE_RATE (2352941)
#define P2_XENSIV_BGT60TRXX_CONF_CHIRP_REPETITION_TIME_S (6.945e-05)
#define P2_XENSIV_BGT60TRXX_CONF_FRAME_REPETITION_TIME_S (0.100049)
#define P2_XENSIV_BGT60TRXX_CONF_NUM_REGS (38)


uint32_t register_list_p2[] = {
    0x11e8270UL,
    0x3088210UL,
    0x9e967fdUL,
    0xb0805b4UL,
    0xdf023ffUL,
    0xf010700UL,
    0x11000000UL,
    0x13000000UL,
    0x15000000UL,
    0x17000be0UL,
    0x19000000UL,
    0x1b000000UL,
    0x1d000000UL,
    0x1f000b60UL,
    0x21130c51UL,
    0x234ff41fUL,
    0x25006f7bUL,
    0x2d000490UL,
    0x3b000480UL,
    0x49000480UL,
    0x57000480UL,
    0x5911be0eUL,
    0x5b677c0aUL,
    0x5d00f000UL,
    0x5f787e1eUL,
    0x61f5208cUL,
    0x630000a4UL,
    0x65000252UL,
    0x67000080UL,
    0x69000000UL,
    0x6b000000UL,
    0x6d000000UL,
    0x6f092910UL,
    0x7f000100UL,
    0x8f000100UL,
    0x9f000100UL,
    0xad000000UL,
    0xb7000000UL
};

/* Radar configuration for presence detection.
 * This is the high framerate settings. */
#define P3_XENSIV_BGT60TRXX_CONF_DEVICE (XENSIV_DEVICE_BGT60TR13C)
#define P3_XENSIV_BGT60TRXX_CONF_START_FREQ_HZ (61020099000)
#define P3_XENSIV_BGT60TRXX_CONF_END_FREQ_HZ (61479903000)
#define P3_XENSIV_BGT60TRXX_CONF_NUM_SAMPLES_PER_CHIRP (128)
#define P3_XENSIV_BGT60TRXX_CONF_NUM_CHIRPS_PER_FRAME (16)
#define P3_XENSIV_BGT60TRXX_CONF_NUM_RX_ANTENNAS (1)
#define P3_XENSIV_BGT60TRXX_CONF_NUM_TX_ANTENNAS (1)
#define P3_XENSIV_BGT60TRXX_CONF_SAMPLE_RATE (2352941)
#define P3_XENSIV_BGT60TRXX_CONF_CHIRP_REPETITION_TIME_S (6.99625e-05)
#define P3_XENSIV_BGT60TRXX_CONF_FRAME_REPETITION_TIME_S (0.00999593)
#define P3_XENSIV_BGT60TRXX_CONF_NUM_REGS (38)

uint32_t register_list_p3[] = {
    0x11e8270UL,
    0x3088210UL,
    0x9e967fdUL,
    0xb0805b4UL,
    0xd1027ffUL,
    0xf010700UL,
    0x11000000UL,
    0x13000000UL,
    0x15000000UL,
    0x17000be0UL,
    0x19000000UL,
    0x1b000000UL,
    0x1d000000UL,
    0x1f000b60UL,
    0x21130c51UL,
    0x234ff41fUL,
    0x25006f7bUL,
    0x2d000490UL,
    0x3b000480UL,
    0x49000480UL,
    0x57000480UL,
    0x5911be0eUL,
    0x5b4ccc0aUL,
    0x5d00f000UL,
    0x5f787e1eUL,
    0x61f5208aUL,
    0x630000a4UL,
    0x65000252UL,
    0x67000080UL,
    0x69000000UL,
    0x6b000000UL,
    0x6d000000UL,
    0x6f093910UL,
    0x7f000100UL,
    0x8f000100UL,
    0x9f000100UL,
    0xad000000UL,
    0xb7000000UL,
};

#endif /* XENSIV_BGT60TRXX_CONF_H */

/* [] END OF FILE */
