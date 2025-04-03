/******************************************************************************
* File Name:   dev_bgt60trxx.c
*
* Description: This file implements the interface with the radar sensor.
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

#ifdef IM_ENABLE_BGT60TRXX

#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <cyhal.h>
#include <cybsp.h>
#include <cy_pdl.h>
#include <xensiv_bgt60trxx_mtb.h>

#include "protocol/protocol.h"
#include "protocol/pb_encode.h"
#include "clock.h"
#include "dev_bgt60trxx_settings.h"
#include "dev_bgt60trxx.h"

#include "cyhal_system.h"
#include "xensiv_bgt60trxx_platform.h"

#include <cy_scb_spi.h>

/*******************************************************************************
* Macros
*******************************************************************************/

#define DEV_BGT60TRXX_OPTION_PRESET     (1)
#define DEV_BGT60TRXX_OPTION_CUSTOM_SET (2)


#define XENSIV_BGT60TRXX_SPI_REG_XFER_LEN_BYTES         (4U)
#define XENSIV_BGT60TRXX_SOFT_RESET_DELAY_MS            (10U)

#define XENSIV_BGT60TRXX_SPI_WR_OP_MSK                  (0x01000000UL)
#define XENSIV_BGT60TRXX_SPI_WR_OP_POS                  (24U)
#define XENSIV_BGT60TRXX_SPI_GSR0_MSK                   (0x0F000000UL)
#define XENSIV_BGT60TRXX_SPI_GSR0_POS                   (24U)
#define XENSIV_BGT60TRXX_SPI_REGADR_MSK                 (0xFE000000UL)
#define XENSIV_BGT60TRXX_SPI_REGADR_POS                 (25U)
#define XENSIV_BGT60TRXX_SPI_DATA_MSK                   (0x00FFFFFFUL)
#define XENSIV_BGT60TRXX_SPI_DATA_POS                   (0U)
#define XENSIV_BGT60TRXX_SPI_BURST_MODE_CMD             (0xFF000000UL)  /* Write addr 7f<<1 | 0x01; */
#define XENSIV_BGT60TRXX_SPI_BURST_MODE_SADR_MSK        (0x00FE0000UL)
#define XENSIV_BGT60TRXX_SPI_BURST_MODE_SADR_POS        (17U)
#define XENSIV_BGT60TRXX_SPI_BURST_MODE_RWB_MSK         (0x00010000UL)
#define XENSIV_BGT60TRXX_SPI_BURST_MODE_RWB_POS         (16U)
#define XENSIV_BGT60TRXX_SPI_BURST_MODE_LEN_MSK         (0x0000FE00UL)
#define XENSIV_BGT60TRXX_SPI_BURST_MODE_LEN_POS         (9U)

struct xensiv_bgt60trxx_type
{
    uint32_t fifo_addr;
    uint16_t fifo_size;
    xensiv_bgt60trxx_device_t device;
};

/*******************************************************************************
* Ringbuffer array
*******************************************************************************/
/* This sizes and masks must be aligned. */
#define RING_BUFFER_SIZE 0x00010000 /* 64K, samples */
#define RING_BUFFER_MASK 0x0000FFFF
#define RING_BUFFER_MASK32 0x00007FFF /* Mask for 32 bits. */
uint16_t bgt60_tensor_ring[ RING_BUFFER_SIZE ];  /* This is the size of the internal fifo. */
uint32_t* ring32; /* This will point to 32k a 32 bits. */
int bgt60_ring_next_to_write = 0;
int bgt60_ring_next_to_read = 0;
int bgt60_ring_level=0;

/*******************************************************************************
* Preset configuration variables and structs.
*******************************************************************************/
struct radar_config radar_configs[5];
int current_config_g = 0;

/*******************************************************************************
* Types
*******************************************************************************/

typedef struct {
    xensiv_bgt60trxx_mtb_t bgt60_obj;
    uint16_t bgt60_buffer0[16384];
    uint16_t bgt60_buffer1[16384];
    uint16_t bgt60_send_buffer[RING_BUFFER_SIZE];
    bool have_data;
    int skipped_frames;
} dev_bgt60trxx_t;

int radar_sent_frames=0;

/*******************************************************************************
* Function Prototypes
*******************************************************************************/
static bool _config_hw(dev_bgt60trxx_t *radar, int preset);
static bool _init_hw(dev_bgt60trxx_t *radar, cyhal_spi_t* spi );

static bool _configure_streams(protocol_t* protocol, int device, void* arg);
static void _start_streams(protocol_t* protocol, int device, pb_ostream_t* ostream, void* arg);
static void _stop_streams(protocol_t* protocol, int device, pb_ostream_t* ostream, void* arg);
static void _poll_streams(protocol_t* protocol, int device, pb_ostream_t* ostream, void* arg);

static void _load_presets();
static void _load_custom_values( uint8_t* data_blob );

static bool _write_payload(
    protocol_t* protocol,
    int device_id,
    int stream_id,
    int frame_count,
    int total_bytes,
    pb_ostream_t* ostream,
    void* arg);

/*******************************************************************************
* Function Name: _init_hw
********************************************************************************
* Summary:
*    A function used to initialize the Radar sensor Present in
*    Ai Evaluation Kit(CY8CKIT-062S2-AI).
*
* Parameters:
*   None
*
* Return:
*     The status of the initialization.
*
*
*******************************************************************************/
static bool _init_hw(dev_bgt60trxx_t *radar, cyhal_spi_t* spi)
{
    cy_rslt_t result;

    radar->have_data = false;
    radar->skipped_frames = 0;

    /* Reduce drive strength to improve EMI */
    Cy_GPIO_SetSlewRate(CYHAL_GET_PORTADDR(CYBSP_RSPI_MOSI), CYHAL_GET_PIN(CYBSP_RSPI_MOSI), CY_GPIO_SLEW_FAST);
    Cy_GPIO_SetDriveSel(CYHAL_GET_PORTADDR(CYBSP_RSPI_MOSI), CYHAL_GET_PIN(CYBSP_RSPI_MOSI), CY_GPIO_DRIVE_1_8);
    Cy_GPIO_SetSlewRate(CYHAL_GET_PORTADDR(CYBSP_RSPI_CLK), CYHAL_GET_PIN(CYBSP_RSPI_CLK), CY_GPIO_SLEW_FAST);
    Cy_GPIO_SetDriveSel(CYHAL_GET_PORTADDR(CYBSP_RSPI_CLK), CYHAL_GET_PIN(CYBSP_RSPI_CLK), CY_GPIO_DRIVE_1_8);

    /* Initialization uses preset 0 */
    result = xensiv_bgt60trxx_mtb_init(
            &radar->bgt60_obj,
            spi,
            CYBSP_RSPI_CS,
            CYBSP_RXRES_L,
            radar_configs[0].register_list,
            radar_configs[0].number_of_regs
            );

    if (result != CY_RSLT_SUCCESS)
    {
        printf("ERROR: xensiv_bgt60trxx_mtb_init failed\n");
        return false;
    }

    result = cyhal_gpio_init(CYBSP_RSPI_IRQ, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_PULLDOWN, false);

    if (result == CY_RSLT_SUCCESS)
    {
        xensiv_bgt60trxx_set_fifo_limit(&radar->bgt60_obj.dev, radar_configs[0].fifo_int_level);
    }

    return true;
}



/*******************************************************************************
 * Platform functions implementation
 ********************************************************************************/
__STATIC_INLINE void spi_set_data_width(CySCB_Type* base, uint32_t data_width)
{
    CY_ASSERT(CY_SCB_SPI_IS_DATA_WIDTH_VALID(data_width));

    CY_REG32_CLR_SET(SCB_TX_CTRL(base),
                     SCB_TX_CTRL_DATA_WIDTH,
                     (uint32_t)data_width - 1U);
    CY_REG32_CLR_SET(SCB_RX_CTRL(base),
                     SCB_RX_CTRL_DATA_WIDTH,
                     (uint32_t)data_width - 1U);
}

/*******************************************************************************
* Function Name: _configure_streams
********************************************************************************
* Summary:
*   Called when the device is configured or re-configured.
*
* Parameters:
*   protocol: Pointer the protocol handle
*   device: The device index
*   arg: Pointer the device struct (dev_bgt60trxx_t)
*
* Return:
*   True to keep the connection open, otherwise false.
*
*******************************************************************************/
static bool _configure_streams(protocol_t* protocol, int device, void* arg)
{
    UNUSED(arg);
    int preset_index=0;

    if(protocol_clear_streams(protocol, device) != PROTOCOL_STATUS_SUCCESS) {
        protocol_set_device_status(
                protocol,
                device,
                protocol_DeviceStatus_DEVICE_STATUS_ERROR,
                "Failed to clear streams.");
        return true;
    }

    /* Check if custom option exist. */
    int status;
    pb_bytes_array_t* pb_bytes;

    status = protocol_get_option_blob(protocol, device,
                DEV_BGT60TRXX_OPTION_CUSTOM_SET,
                &pb_bytes);

    if (status != PROTOCOL_STATUS_SUCCESS){
        printf("Getting blob option failed\n");
    }

    if ( pb_bytes == NULL ){
        /*printf("Don't have any blob option.\n"); */
    }
    else{
        _load_custom_values( (uint8_t*)pb_bytes);
    }

    protocol_get_option_oneof(protocol, device, DEV_BGT60TRXX_OPTION_PRESET, &preset_index);
    current_config_g = preset_index;

    int stream = protocol_add_stream(
        protocol,
        device,
        "Radar",
        protocol_StreamDirection_STREAM_DIRECTION_OUTPUT,
        protocol_DataType_DATA_TYPE_S16,
        radar_configs[preset_index].frame_rate, /* Frequency in number of frames per second. */
                                                /* This needs to be rounded to some integer value. */
        1,      /* Max frame_count? */
        "???");

    if(stream < 0) {
        protocol_set_device_status(
                protocol,
                device,
                protocol_DeviceStatus_DEVICE_STATUS_ERROR,
                "Failed to add streams.");
        return true;
    }

    protocol_add_stream_rank(
            protocol,
            device,
            stream,
            "Antenna",
            radar_configs[preset_index].rx_antennas,
            NULL);

    protocol_add_stream_rank(
            protocol,
            device,
            stream,
            "Chirp",
            radar_configs[preset_index].chirps_per_frame,
            NULL);

    protocol_add_stream_rank(
            protocol,
            device,
            stream,
            "Sample",
            radar_configs[preset_index].samples_per_chirp,
            NULL);

    return true;
}
/******************************************************************************
* Function Name: _config_hw
********************************************************************************
* Summary:
*   Configures radar using presets.
*
* Parameters:
*   dev: Pointer to the dev_bgt60trxx_t device handle.
*   preset: One of the presets available.
*
* Return:
*   True if configuration is successful, otherwise false.
*
*******************************************************************************/
static bool _config_hw(dev_bgt60trxx_t *radar, int preset){

    int status;

/* Reinitialize the radar using the selected radar_settings. */
    status = xensiv_bgt60trxx_config(&radar->bgt60_obj.dev,
                                    radar_configs[preset].register_list,
                                    radar_configs[preset].number_of_regs);
    if ( status != XENSIV_BGT60TRXX_STATUS_OK )
    {
        printf("Radar config HW failed\n");
        return false;
    }

/* Update the fifo limit for the interrupts. */
    status = xensiv_bgt60trxx_set_fifo_limit(&radar->bgt60_obj.dev, radar_configs[preset].fifo_int_level);
    if ( status != XENSIV_BGT60TRXX_STATUS_OK )
    {
        printf("Radar config fifo limit failed\n");
        return false;
    }

    return true;
}

/*******************************************************************************
* Function Name: _start_streams
********************************************************************************
* Summary:
*   Called when streaming is started. This may also initialize the device.
*
* Parameters:
*   protocol: Pointer the protocol handle
*   device: The device index
*   ostream: Pointer to the output stream to write to
*   arg: Pointer the device struct (dev_bgt60trxx_t)
*
*******************************************************************************/
static void _start_streams(protocol_t* protocol, int device, pb_ostream_t* ostream, void* arg)
{
    dev_bgt60trxx_t* radar = (dev_bgt60trxx_t*)arg;
    int preset_index;


    UNUSED(radar);
    UNUSED(ostream);

    protocol_get_option_oneof(protocol, device, DEV_BGT60TRXX_OPTION_PRESET, &preset_index);
    current_config_g = preset_index;

    if(!_config_hw(radar, preset_index) ) {
        protocol_set_device_status(
            protocol,
            device,
            protocol_DeviceStatus_DEVICE_STATUS_ERROR,
            "Failed to configure hardware.");
    }

    /* TODO: enable radar */
    if (xensiv_bgt60trxx_soft_reset(&radar->bgt60_obj.dev, XENSIV_BGT60TRXX_RESET_FIFO) != XENSIV_BGT60TRXX_STATUS_OK){
        /* Error */
        printf("Fifo reset error error.\r\n");
    }
    /* Reset the local fifo. */
    bgt60_ring_next_to_write = 0;
    bgt60_ring_next_to_read = 0;
    bgt60_ring_level=0;

    radar->have_data = false;
    if (xensiv_bgt60trxx_start_frame(&radar->bgt60_obj.dev, true) != XENSIV_BGT60TRXX_STATUS_OK){
        /* Error */
        printf("Start frame error.\r\n");
    }
    else{
            printf("\nStreaming\n");
    }

    protocol_set_device_status(
            protocol,
            device,
            protocol_DeviceStatus_DEVICE_STATUS_ACTIVE,
            "Device is streaming");

    radar_sent_frames = 0;

}

/*******************************************************************************
* Function Name: _stop_streams
********************************************************************************
* Summary:
*  Called when streaming is stopped.
*
* Parameters:
*  protocol: Pointer the protocol handle
*  device: The device index
*  ostream: Pointer to the output stream to write to
*  arg: Pointer the device struct (dev_bgt60trxx_t)
*
*******************************************************************************/
static void _stop_streams(protocol_t* protocol, int device, pb_ostream_t* ostream, void* arg)
{

    dev_bgt60trxx_t* radar = (dev_bgt60trxx_t*)arg;

    UNUSED(radar);
    UNUSED(ostream);
    /* TODO: disable radar */
    if (xensiv_bgt60trxx_start_frame(&radar->bgt60_obj.dev, false) != XENSIV_BGT60TRXX_STATUS_OK){
        /* Error */
    }

    protocol_set_device_status(
            protocol,
            device,
            protocol_DeviceStatus_DEVICE_STATUS_READY,
            "Device stopped");

    printf("radar_sent_frames = %d\n", radar_sent_frames );
    radar_sent_frames = 0;
}

/*******************************************************************************
* Function Name: _poll_streams
********************************************************************************
* Summary:
*  Called periodically to send data messages.
*
* Parameters:
*  protocol: Pointer the protocol handle
*  device: The device index
*  ostream: Pointer to the output stream to write to
*  arg: Pointer the device struct (dev_bgt60trxx_t)
*
*******************************************************************************/

enum spi_state {
    NONE = 0,
    IDLE,
    BURST_PENDING,
    FIFO_READ_PENDING,
    FIFO_READ_DONE
};

enum spi_state sp_state = IDLE;

static void _poll_streams(protocol_t* protocol, int device, pb_ostream_t* ostream, void* arg)
{

    dev_bgt60trxx_t* radar = (dev_bgt60trxx_t*)arg;

    xensiv_bgt60trxx_t* p_device = &radar->bgt60_obj.dev;

    void* iface = p_device->iface;
    xensiv_bgt60trxx_mtb_iface_t* mtb_iface = iface;

    /* Check if SPI transfer is active. If it is active then skip everything. */
    if (0UL != (CY_SCB_SPI_TRANSFER_ACTIVE &
                Cy_SCB_SPI_GetTransferStatus(mtb_iface->spi->base, &(mtb_iface->spi->context))))
    {
        return;
    }


    int a = 0;
    int words_a_24 = 0;

    switch (sp_state) {
        case IDLE:
            a = cyhal_gpio_read(CYBSP_RSPI_IRQ); /* P11_0 */
            if ( a ) {

                /* Make Chip select */
                xensiv_bgt60trxx_platform_spi_cs_set(iface, false);

                /* Read current fifo state */
                uint32_t* head = (uint32_t*)radar->bgt60_buffer0;
                const xensiv_bgt60trxx_mtb_iface_t* mtb_iface = iface;

                *head = XENSIV_BGT60TRXX_SPI_BURST_MODE_CMD |
                        (p_device->type->fifo_addr << XENSIV_BGT60TRXX_SPI_BURST_MODE_SADR_POS) |   /* Addr 0x60.. */
                        (0 << XENSIV_BGT60TRXX_SPI_BURST_MODE_RWB_POS) |    /* Read mode    */
                        (0 << XENSIV_BGT60TRXX_SPI_BURST_MODE_LEN_POS);     /* Read until termination. */

                /* Ensure correct byte order for sending the command. */
                *head = xensiv_bgt60trxx_platform_word_reverse(*head);

                spi_set_data_width(mtb_iface->spi->base, 8U);
                Cy_SCB_SetByteMode(mtb_iface->spi->base, true);

                Cy_SCB_SPI_Transfer(mtb_iface->spi->base, (uint8_t*)radar->bgt60_buffer0,
                                                                  (uint8_t*)radar->bgt60_buffer1, /* Can be set to NULL, Recieved data is discarded. */
                                                                  4,
                                                                  &(mtb_iface->spi->context));

                sp_state = BURST_PENDING;
                return;
            }
            break;

        case BURST_PENDING:

            words_a_24 = radar_configs[current_config_g].fifo_int_level & 0xFFFFFFC0;

            spi_set_data_width(mtb_iface->spi->base, 12U);
            Cy_SCB_SetByteMode(mtb_iface->spi->base, false);

            Cy_SCB_SPI_Transfer(mtb_iface->spi->base,
                                            (uint8_t*)radar->bgt60_buffer0, /* Can be set to NULL, Tx data is irrelevant. */
                                            (uint8_t*)radar->bgt60_buffer1,
                                            words_a_24,
                                            &(mtb_iface->spi->context));

            /* At this point it might be a good idea to check if there is another interrupt from the radar pending. */
            /* If there is it should be possible to swap buffers and issue a second read of data. */
            /* In this state the radar burst mode is still active. */
            sp_state = FIFO_READ_PENDING;
            break;

        case FIFO_READ_PENDING:
            /* Clear the Chip Select. Done when the next polling allows for it. */
            xensiv_bgt60trxx_platform_spi_cs_set(iface, true);
            sp_state = FIFO_READ_DONE;
            break;

        case FIFO_READ_DONE:
            sp_state = IDLE;
            break;

        default:
            break;
    }



    if(sp_state == FIFO_READ_DONE) {

        radar->have_data = false;


/*********************************************************************************
**  Data copying is now made as a 32 bit data copying to reduce the number of
**  clock cycles needed.
*/
        ring32 = (uint32_t*)bgt60_tensor_ring; /* This will point to 32k a 32 bits. */
        int fifo_size_32 = radar_configs[current_config_g].fifo_int_level>>1;

        for ( int x = 0; x<fifo_size_32;x++ ){
                                                /* Note the operator priority here. */
            ring32[bgt60_ring_next_to_write] = ((uint32_t*)(radar->bgt60_buffer1))[x]; /* radar->full_buffer[x]; */
            /*bgt60_ring_level++; */
            bgt60_ring_level+=2;        /* Copying 2 samples at a time. */
            bgt60_ring_next_to_write++;
            bgt60_ring_next_to_write&=RING_BUFFER_MASK32;
        }

        int t=0;
        int frames=0;

        /* bgt60_ring_level holds the number of samples. */
        while ( bgt60_ring_level >= radar_configs[current_config_g].num_samples_per_frame ){
            for ( int i=0; i < radar_configs[current_config_g].num_samples_per_frame; i+=2 ){
                ((uint32_t*)(radar->bgt60_send_buffer))[t++] = ring32[bgt60_ring_next_to_read];
                bgt60_ring_level-=2;
                bgt60_ring_next_to_read++;
                bgt60_ring_next_to_read&=RING_BUFFER_MASK32;
            }
            frames++;
        }

/*********************************************************************************/
        if ( frames ){
            protocol_send_data_chunk(
                protocol,
                device,
                0,
                frames,
                radar->skipped_frames,
                ostream,
                _write_payload);

        }
        radar_sent_frames+=frames;
        radar->skipped_frames = 0;

    }
}

/*******************************************************************************
* Function Name: _write_payload
********************************************************************************
* Summary:
*  Used by protocol_send_data_chunk to write the actual data.
*
* Parameters:
*  protocol: Pointer the protocol handle
*  device_id: The device index
*  stream_id: The stream index
*  frame_count: Number of frames to write
*  total_bytes: Total number of bytes to write (= frame_count * sizeof(type) * frame_shape.flat)
*  ostream: Pointer to the output stream to write to
*  arg: Pointer the device struct (dev_bgt60trxx_t)
*
* Return:
*   True if data writing is successful, otherwise false.
*
*******************************************************************************/
static bool _write_payload(
    protocol_t* protocol,
    int device_id,
    int stream_id,
    int frame_count,
    int total_bytes,
    pb_ostream_t* ostream,
    void* arg)
{
    UNUSED(protocol);
    UNUSED(stream_id);
    UNUSED(frame_count);
    UNUSED(protocol);

    dev_bgt60trxx_t* radar = (dev_bgt60trxx_t*)arg;

    if (!pb_write(ostream, (const pb_byte_t *)radar->bgt60_send_buffer, total_bytes))
    {
        return false;
    }

    return true;
}

/*******************************************************************************
* Summary:
*   This gets a binary data_blob via the protocol and transfer this values to
*   setting index 4. This 'slot' is initially identical to the first but is
*   overwritten by custom data if set.
*******************************************************************************/

#define DEFAULT_FIFO_SETTING 6144

static void _load_custom_values( uint8_t* data_blob ){

    /* Binary file is prepended with a 2 byte size that should be skipped. */
    uint32_t* values = (uint32_t*)(data_blob+2);

/* Ensure this is updated if the binary file is generated diffrently. */
#define OFF_REGS_START                      16
#define OFF_CONF_NUM_REGS                   0
#define OFF_CONF_REGS_START                 1
#define OFF_CONF_START_FREQ_LO_HZ           2
#define OFF_CONF_START_FREQ_HI_HZ           3
#define OFF_CONF_END_FREQ_LO_HZ             4
#define OFF_CONF_END_FREQ_HI_HZ             5
#define OFF_CONF_NUM_SAMPLES_PER_CHIRP      6
#define OFF_CONF_NUM_CHIRPS_PER_FRAME       7
#define OFF_CONF_NUM_RX_ANTENNAS            8
#define OFF_CONF_NUM_TX_ANTENNAS            9
#define OFF_CONF_SAMPLE_RATE                10
#define OFF_CONF_CHIRP_REPETITION_TIME_S    11
#define OFF_CONF_FRAME_REPETITION_TIME_S    12

    /* Misaligned 64 bit data needs bytewise copying.  */
    memcpy( (char*)&radar_configs[4].start_freq, (char*)&values[OFF_CONF_START_FREQ_LO_HZ], 8 );
    memcpy( (char*)&radar_configs[4].end_freq, (char*)&values[OFF_CONF_END_FREQ_LO_HZ], 8 );

    radar_configs[4].samples_per_chirp =     values[OFF_CONF_NUM_SAMPLES_PER_CHIRP];
    radar_configs[4].chirps_per_frame =      values[OFF_CONF_NUM_CHIRPS_PER_FRAME];
    radar_configs[4].rx_antennas =           values[OFF_CONF_NUM_RX_ANTENNAS];
    radar_configs[4].tx_antennas =           values[OFF_CONF_NUM_TX_ANTENNAS];
    radar_configs[4].sample_rate =           values[OFF_CONF_SAMPLE_RATE];

    radar_configs[4].chirp_repetition_time = *(float*)&values[OFF_CONF_CHIRP_REPETITION_TIME_S];  /* 32bit float */
    radar_configs[4].frame_repetition_time = *(float*)&values[OFF_CONF_FRAME_REPETITION_TIME_S];  /* 32bit float */

    radar_configs[4].frame_rate = (int)(0.5 + 1.0/(double)radar_configs[4].frame_repetition_time);

    radar_configs[4].num_samples_per_frame =
            radar_configs[4].samples_per_chirp *
            radar_configs[4].chirps_per_frame *
            radar_configs[4].rx_antennas;

    radar_configs[4].fifo_int_level = DEFAULT_FIFO_SETTING; /* Words!This will equal 75% of the fifo. 12288 samples! */
    radar_configs[4].number_of_regs =           values[OFF_CONF_NUM_REGS];
    radar_configs[4].register_list = &values[values[OFF_CONF_REGS_START]];

    return;

}
/*******************************************************************************
* Summary:
*   This just load preset from header file constants to enable selection of
*   various radar presets.
* See dev_bgt60trxx_settings.h for information of the sources of radar settings.
*
*******************************************************************************/
static void _load_presets(){
/* Config 1 Default testing configuration. */
    radar_configs[0].start_freq =            P0_XENSIV_BGT60TRXX_CONF_START_FREQ_HZ;
    radar_configs[0].end_freq =              P0_XENSIV_BGT60TRXX_CONF_END_FREQ_HZ;
    radar_configs[0].samples_per_chirp =     P0_XENSIV_BGT60TRXX_CONF_NUM_SAMPLES_PER_CHIRP;
    radar_configs[0].chirps_per_frame =      P0_XENSIV_BGT60TRXX_CONF_NUM_CHIRPS_PER_FRAME;
    radar_configs[0].rx_antennas =           P0_XENSIV_BGT60TRXX_CONF_NUM_RX_ANTENNAS;
    radar_configs[0].tx_antennas =           P0_XENSIV_BGT60TRXX_CONF_NUM_TX_ANTENNAS;
    radar_configs[0].sample_rate =           P0_XENSIV_BGT60TRXX_CONF_SAMPLE_RATE;
    radar_configs[0].chirp_repetition_time = P0_XENSIV_BGT60TRXX_CONF_CHIRP_REPETITION_TIME_S;
    radar_configs[0].frame_repetition_time = P0_XENSIV_BGT60TRXX_CONF_FRAME_REPETITION_TIME_S;

    radar_configs[0].frame_rate = (int)(0.5 + 1.0/(double)radar_configs[0].frame_repetition_time);
    radar_configs[0].num_samples_per_frame =
            radar_configs[0].samples_per_chirp *
            radar_configs[0].chirps_per_frame *
            radar_configs[0].rx_antennas;

    radar_configs[0].fifo_int_level = DEFAULT_FIFO_SETTING; /* Words!This will equal 75% of the fifo. 12288 samples! */


    radar_configs[0].number_of_regs =           P0_XENSIV_BGT60TRXX_CONF_NUM_REGS;
    radar_configs[0].register_list = register_list_p0;

/* Config 2 Gesture detection */
    radar_configs[1].start_freq =            P1_XENSIV_BGT60TRXX_CONF_START_FREQ_HZ;
    radar_configs[1].end_freq =              P1_XENSIV_BGT60TRXX_CONF_END_FREQ_HZ;
    radar_configs[1].samples_per_chirp =     P1_XENSIV_BGT60TRXX_CONF_NUM_SAMPLES_PER_CHIRP;
    radar_configs[1].chirps_per_frame =      P1_XENSIV_BGT60TRXX_CONF_NUM_CHIRPS_PER_FRAME;
    radar_configs[1].rx_antennas =           P1_XENSIV_BGT60TRXX_CONF_NUM_RX_ANTENNAS;
    radar_configs[1].tx_antennas =           P1_XENSIV_BGT60TRXX_CONF_NUM_TX_ANTENNAS;
    radar_configs[1].sample_rate =           P1_XENSIV_BGT60TRXX_CONF_SAMPLE_RATE;
    radar_configs[1].chirp_repetition_time = P1_XENSIV_BGT60TRXX_CONF_CHIRP_REPETITION_TIME_S;
    radar_configs[1].frame_repetition_time = P1_XENSIV_BGT60TRXX_CONF_FRAME_REPETITION_TIME_S;

    radar_configs[1].frame_rate = (int)(0.5 + 1.0/(double)radar_configs[1].frame_repetition_time);
    radar_configs[1].num_samples_per_frame =
            radar_configs[1].samples_per_chirp *
            radar_configs[1].chirps_per_frame *
            radar_configs[1].rx_antennas;

    radar_configs[1].fifo_int_level = DEFAULT_FIFO_SETTING; /* Words!This will equal 75% of the fifo. 12288 samples! */


    radar_configs[1].number_of_regs =           P1_XENSIV_BGT60TRXX_CONF_NUM_REGS;
    radar_configs[1].register_list = register_list_p1;

/* Config 3 Macro movements */
    radar_configs[2].start_freq =            P2_XENSIV_BGT60TRXX_CONF_START_FREQ_HZ;
    radar_configs[2].end_freq =              P2_XENSIV_BGT60TRXX_CONF_END_FREQ_HZ;
    radar_configs[2].samples_per_chirp =     P2_XENSIV_BGT60TRXX_CONF_NUM_SAMPLES_PER_CHIRP;
    radar_configs[2].chirps_per_frame =      P2_XENSIV_BGT60TRXX_CONF_NUM_CHIRPS_PER_FRAME;
    radar_configs[2].rx_antennas =           P2_XENSIV_BGT60TRXX_CONF_NUM_RX_ANTENNAS;
    radar_configs[2].tx_antennas =           P2_XENSIV_BGT60TRXX_CONF_NUM_TX_ANTENNAS;
    radar_configs[2].sample_rate =           P2_XENSIV_BGT60TRXX_CONF_SAMPLE_RATE;
    radar_configs[2].chirp_repetition_time = P2_XENSIV_BGT60TRXX_CONF_CHIRP_REPETITION_TIME_S;
    radar_configs[2].frame_repetition_time = P2_XENSIV_BGT60TRXX_CONF_FRAME_REPETITION_TIME_S;

    radar_configs[2].frame_rate = (int)(0.5 + 1.0/(double)radar_configs[2].frame_repetition_time);
    radar_configs[2].num_samples_per_frame =
            radar_configs[2].samples_per_chirp *
            radar_configs[2].chirps_per_frame *
            radar_configs[2].rx_antennas;

    radar_configs[2].fifo_int_level = DEFAULT_FIFO_SETTING; /* Words!This will equal 75% of the fifo. 12288 samples! */


    radar_configs[2].number_of_regs =           P2_XENSIV_BGT60TRXX_CONF_NUM_REGS;
    radar_configs[2].register_list = register_list_p2;

/* Config 4 Micro movements. */
    radar_configs[3].start_freq =            P3_XENSIV_BGT60TRXX_CONF_START_FREQ_HZ;
    radar_configs[3].end_freq =              P3_XENSIV_BGT60TRXX_CONF_END_FREQ_HZ;
    radar_configs[3].samples_per_chirp =     P3_XENSIV_BGT60TRXX_CONF_NUM_SAMPLES_PER_CHIRP;
    radar_configs[3].chirps_per_frame =      P3_XENSIV_BGT60TRXX_CONF_NUM_CHIRPS_PER_FRAME;
    radar_configs[3].rx_antennas =           P3_XENSIV_BGT60TRXX_CONF_NUM_RX_ANTENNAS;
    radar_configs[3].tx_antennas =           P3_XENSIV_BGT60TRXX_CONF_NUM_TX_ANTENNAS;
    radar_configs[3].sample_rate =           P3_XENSIV_BGT60TRXX_CONF_SAMPLE_RATE;
    radar_configs[3].chirp_repetition_time = P3_XENSIV_BGT60TRXX_CONF_CHIRP_REPETITION_TIME_S;
    radar_configs[3].frame_repetition_time = P3_XENSIV_BGT60TRXX_CONF_FRAME_REPETITION_TIME_S;

    radar_configs[3].frame_rate = (int)(0.5 + 1.0/(double)radar_configs[3].frame_repetition_time);
    radar_configs[3].num_samples_per_frame =
            radar_configs[3].samples_per_chirp *
            radar_configs[3].chirps_per_frame *
            radar_configs[3].rx_antennas;

    radar_configs[3].fifo_int_level = DEFAULT_FIFO_SETTING; /* Words!This will equal 75% of the fifo. 12288 samples! */

    radar_configs[3].number_of_regs =           P3_XENSIV_BGT60TRXX_CONF_NUM_REGS;
    radar_configs[3].register_list = register_list_p3;

/* The fifth configuration is the configuration used for customization and is loaded
 * with the same as the first as default. This will then be overwritten if a new
 * blob arrives. */

    memcpy( &radar_configs[4], &radar_configs[0], sizeof(radar_configs[0]) );

    return;
}

/*******************************************************************************
* Function Name: dev_bgt60trxx_register
********************************************************************************
* Summary:
*   Registers this device. This is the only exported symbol from this object.
*
* Parameters:
*   protocol: Pointer the protocol handle
*
* Returns:
*   True if registration is successful, otherwise false.
*
*******************************************************************************/
bool dev_bgt60trxx_register(protocol_t* protocol, cyhal_spi_t* spi)
{
    int status;
    dev_bgt60trxx_t* radar = (dev_bgt60trxx_t*)malloc(sizeof(dev_bgt60trxx_t));
    if(radar == NULL)
    {
        printf("BGT60TRXX::malloc failed.\r\n");
        return false;
    }
    memset(radar, 0, sizeof(dev_bgt60trxx_t));

    _load_presets();

    if( !_init_hw(radar, spi) )
    {
        printf("BGT60TRXX::hw_init failed.\r\n");
        free(radar);
        return false;
    }

    device_manager_t manager = {
        .arg = radar,
        .configure_streams = _configure_streams,
        .start = _start_streams,
        .stop = _stop_streams,
        .poll = _poll_streams,
        .data_received = NULL /* has no input streams */
    };

    int device = protocol_add_device(
        protocol,
        protocol_DeviceType_DEVICE_TYPE_SENSOR,
        "Radar",
        "Radar device (bgt60trxx)",
        manager);

    if(device < 0)
    {
        printf("BGT60TRXX::adding device failed.\r\n");
        free(radar);
        return false;
    }

    status = protocol_add_option_oneof(
        protocol,
        device,
        DEV_BGT60TRXX_OPTION_PRESET,
        "Use case",
        "Presets",
        0,
        (const char* []) { "Default", "Gesture", "Macro Presence", "Micro Presence", "Custom" },
        5);

    if(status != PROTOCOL_STATUS_SUCCESS)
    {
        printf("Adding option failed\n");
        free(radar);
        return false;
    }

    status = protocol_add_option_blob(
        protocol,
        device,
        DEV_BGT60TRXX_OPTION_CUSTOM_SET,
        "Custom data",
        "Binary Custom data",
        NULL /*pb_bytes_array_t* default_value  */
        );

    if(status != PROTOCOL_STATUS_SUCCESS)
    {
        printf("Adding option custom set failed\n");
        free(radar);
        return false;
    }

    if(!_configure_streams(protocol, device, manager.arg))
    {
        printf("BGT60TRXX::configure streams failed.\r\n");
        free(radar);
        return false;
    }

    return true;
}

#endif /* IM_ENABLE_BGT60TRXX */
