/******************************************************************************
* File Name:   dev_pdm_pcm.c
*
* Description: This file implements the interface with the PDM/PCM.
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
#ifdef IM_ENABLE_PDM_PCM

#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include <cyhal.h>
#include <cybsp.h>

#include <cy_dma.h>

#include "protocol/protocol.h"
#include "protocol/pb_encode.h"
#include "dev_pdm_pcm.h"


/******************************************************************************
 * Macros
 *****************************************************************************/

/* Decimation Rate of the PDM/PCM block. Typical value is 64 */
#define DECIMATION_RATE             64u

/* Audio Subsystem Clock. Typical values depends on the desire sample rate:
- 8/16/48kHz    : 24.576 MHz
- 22.05/44.1kHz : 22.579 MHz */
#define AUDIO_SYS_CLOCK_HZ          24576000u

/* PDM/PCM Pins */
#define PDM_DATA                    P10_5
#define PDM_CLK                     P10_4

/* Define how many samples in a frame */
#define FRAME_SIZE                  (1024)

typedef struct {
    int16_t audio_buffer0[FRAME_SIZE];
    int16_t audio_buffer1[FRAME_SIZE];
    int16_t* active_rx_buffer;
    int16_t* full_rx_buffer;
    bool pdm_pcm_initialized;
    cyhal_pdm_pcm_t pdm_pcm;
    bool have_data;
    bool stereo;
    int skipped_frames;
    int init_discard_counter;
} dev_pdm_pcm_t;


dev_pdm_pcm_t aligned_mic __attribute__ ((aligned (32)));

#define MIC_OPTION_KEY_GAIN 10
#define MIC_OPTION_KEY_STEREO 20
#define MIC_OPTION_KEY_FREQUENCY 30

uint32_t* pdm_fifo_read        = (uint32_t*)(0x40A00308);
volatile uint32_t* DW1_CH_STRUCT26_INTR = (uint32_t*)(0x40298690);
DW_Type* DMA_DW1_base = (DW_Type*)0x40290000;

volatile uint32_t* PDM0_TR_CTL = (uint32_t*)0x40A00040;
#define RX_REQ_EN 0x00010000

volatile uint32_t* PERI_TR_1TO1_GR4_TR_CTL2 = (uint32_t*)0x4000D008;
#define TR_SEL 0x00000001

/******************************************************************************
 * DMA-Descriptors for the audio-dma
 *****************************************************************************/
cy_stc_dma_descriptor_t dma_desc[16];

/******************************************************************************
 * Global Variables
 *****************************************************************************/

cyhal_clock_t   audio_clock;
cyhal_clock_t   pll_clock;
cyhal_clock_t   fll_clock_glob;

/*******************************************************************************
* Function Prototypes
*******************************************************************************/
static bool _init_hw(dev_pdm_pcm_t *dev);
static cy_rslt_t _pcm_start(dev_pdm_pcm_t* mic, int sample_rate, int gain, cyhal_pdm_pcm_mode_t mode);
static cy_rslt_t _pcm_stop(dev_pdm_pcm_t* mic);

/* Protocol callbacks */
static bool _write_payload(
    protocol_t* protocol,
    int device_id,
    int stream_id,
    int frame_count,
    int total_bytes,
    pb_ostream_t* ostream,
    void* arg);
static bool _configure_streams(protocol_t* protocol, int device, void* arg);
static void _start_streams(protocol_t* protocol, int device, pb_ostream_t* ostream, void* arg);
static void _stop_streams(protocol_t* protocol, int device, pb_ostream_t* ostream, void* arg);
static void _poll_streams(protocol_t* protocol, int device, pb_ostream_t* ostream, void* arg);

/*******************************************************************************
* Function Definitions
*******************************************************************************/

/******************************************************************************
* Function Name: _init_hw
********************************************************************************
* Summary:
*   Initializes the PDM/PCM.
*
* Parameters:
*   dev: Pointer to the dev_pdm_pcm_t device handle.
*
* Return:
*   True if operation is successful, otherwise false.
*
*******************************************************************************/
static bool _init_hw(dev_pdm_pcm_t *dev)
{
    cy_rslt_t result;
    UNUSED(dev);

    /* Initialize the FLL */
    result = cyhal_clock_reserve(&fll_clock_glob, &CYHAL_CLOCK_FLL);
    if (result != CY_RSLT_SUCCESS)
    {
        return false;
    }

    result = cyhal_clock_set_frequency(&fll_clock_glob, AUDIO_SYS_CLOCK_HZ, NULL);
    if (result != CY_RSLT_SUCCESS)
    {
        return false;
    }

    result = cyhal_clock_set_enabled(&fll_clock_glob, true, true);
    if (result != CY_RSLT_SUCCESS)
    {
        return false;
    }

    /* Initialize the audio subsystem clock (CLK_HF[1])
     * The CLK_HF[1] is the root clock for the I2S and PDM/PCM blocks */
    result = cyhal_clock_reserve(&audio_clock, &CYHAL_CLOCK_HF[1]);
    if (result != CY_RSLT_SUCCESS)
    {
        return false;
    }

    /* Source the audio subsystem clock from PLL */
    result = cyhal_clock_set_source(&audio_clock, &fll_clock_glob);
    if (result != CY_RSLT_SUCCESS)
    {
        return false;
    }

    result = cyhal_clock_set_enabled(&audio_clock, true, true);
    if (result != CY_RSLT_SUCCESS)
    {
        return false;
    }


    uint32_t clock_hz = cyhal_clock_get_frequency(&fll_clock_glob);
    printf("@init fll_clock_glob = %" PRIu32 "\n", clock_hz );
    clock_hz = cyhal_clock_get_frequency(&audio_clock);
    printf("@init audio_clock = %" PRIu32 "\n", clock_hz);

    printf("pdm_pcm: Initialized device.\r\n");
    return true;
}

/*******************************************************************************
* Function Name: _pcm_start
********************************************************************************
* Summary:
*    A function used to initialize and configure the PDM based on the shield
*    selected in the makefile. Starts an asynchronous read which triggers an
*    interrupt when completed. If the device is already initialized, it will
*    re-initialize.
*
* Parameters:
*   mic: Device handle
*   sample_rate: Sample rate in Hz.
*   gain: Gain in dB
*   mode: Left, right, or stereo
*
* Return:
*     The status of the initialization.
*
*******************************************************************************/
static void _configure_dma_descripors( dev_pdm_pcm_t* mic, cy_stc_dma_descriptor_t dma_desc[] ){


    int x = 0;
    for ( x=0;x<16;x++){
        /* Common for all */
        Cy_DMA_Descriptor_SetDescriptorType(&dma_desc[x], CY_DMA_1D_TRANSFER);
        Cy_DMA_Descriptor_SetSrcAddress(&dma_desc[x], pdm_fifo_read);
        Cy_DMA_Descriptor_SetTriggerInType(&dma_desc[x], CY_DMA_DESCR);
        Cy_DMA_Descriptor_SetDataSize(&dma_desc[x], CY_DMA_HALFWORD);
        Cy_DMA_Descriptor_SetSrcTransferSize(&dma_desc[x], CY_DMA_TRANSFER_SIZE_WORD);
        Cy_DMA_Descriptor_SetDstTransferSize(&dma_desc[x], CY_DMA_TRANSFER_SIZE_DATA); /* HalfWord! */
        Cy_DMA_Descriptor_SetXloopDataCount(&dma_desc[x], 128);
        Cy_DMA_Descriptor_SetXloopSrcIncrement(&dma_desc[x], 0);
        Cy_DMA_Descriptor_SetXloopDstIncrement(&dma_desc[x], 1);

        /* This is for generating an event for each 1024 words fetched. */
        if ( (x==7) || (x==15) ){
            Cy_DMA_Descriptor_SetInterruptType(&dma_desc[x], CY_DMA_DESCR );
        }
        else{
            /* This sets the interrupt trigger to the end of the chain. */
            /* The chain is closed and we don't have eny end of chain so we won't  */
            /* get any events for this descriptors. */
            Cy_DMA_Descriptor_SetInterruptType(&dma_desc[x], CY_DMA_DESCR_CHAIN );
        }

        if ( x<8 ){
            Cy_DMA_Descriptor_SetDstAddress(&dma_desc[x], mic->audio_buffer0+x*128);
        }
        else{
            Cy_DMA_Descriptor_SetDstAddress(&dma_desc[x], mic->audio_buffer1+(x-8)*128);
        }

        Cy_DMA_Descriptor_SetNextDescriptor(&dma_desc[x], &dma_desc[x+1] );
    }

    /* Make the descriptors a round robin list by connecting the last to the first. */
    Cy_DMA_Descriptor_SetNextDescriptor(&dma_desc[15], &dma_desc[0] );

}

static void enable_pdm_dma( dev_pdm_pcm_t* mic, cy_stc_dma_descriptor_t dma_desc[] ){

    _configure_dma_descripors( mic, dma_desc );

   Cy_DMA_Channel_SetDescriptor(DMA_DW1_base, 26, &dma_desc[0]);
   Cy_DMA_Channel_SetPriority(DMA_DW1_base, 26, 0);

    *PDM0_TR_CTL = RX_REQ_EN;
    *PERI_TR_1TO1_GR4_TR_CTL2 = TR_SEL;

   /* Should remember to disable DMA when stopped. */
   Cy_DMA_Channel_Enable(DMA_DW1_base, 26);
   Cy_DMA_Enable(DMA_DW1_base);
}


static cy_rslt_t _pcm_start(dev_pdm_pcm_t* mic, int sample_rate, int gain, cyhal_pdm_pcm_mode_t mode)
{
    cy_rslt_t result;

    /* Stop and free the device if running */
    result = _pcm_stop(mic);
    if (CY_RSLT_SUCCESS != result)
    {
        return result;
    }

    /* Set up pointers to two buffers to implement a ping-pong buffer system.
     * One gets filled by the PDM while the other can be processed. */
    memset(mic, 0, sizeof(dev_pdm_pcm_t));
    mic->active_rx_buffer = mic->audio_buffer0;
    mic->full_rx_buffer = mic->audio_buffer1;
    mic->have_data = false;
    mic->stereo = mode == CYHAL_PDM_PCM_MODE_STEREO;
    mic->init_discard_counter = 4; /* Skip the first 4 frames. */

     /* Initialize the PDM/PCM block */
    static cyhal_pdm_pcm_cfg_t pdm_pcm_cfg;
    pdm_pcm_cfg.sample_rate     = sample_rate;
    pdm_pcm_cfg.decimation_rate = DECIMATION_RATE;
    pdm_pcm_cfg.mode            = mode;
    pdm_pcm_cfg.word_length     = 16;  /* bits */
    pdm_pcm_cfg.left_gain       = (int16_t)(gain * 2);   /* dB */
    pdm_pcm_cfg.right_gain      = (int16_t)(gain * 2);   /* dB */

    result = cyhal_pdm_pcm_init(&mic->pdm_pcm, PDM_DATA, PDM_CLK, &audio_clock, &pdm_pcm_cfg);
    if (CY_RSLT_SUCCESS != result)
    {
        return result;
    }

    /*  Adding temporary audio hack here to get the correct frequency to the mic.
    **  This hack should be replaced with hal code if possible.
    **  This is to be in range with mic specification.
    */
    volatile uint32_t* pdm_reg = (uint32_t*)(0x40A00010);

    uint32_t clk_clock_div_stage_1 = 2;
    uint32_t mclkq_clock_div_stage_2 = 1;
    uint32_t cko_clock_div_stage_3 = 8;
    uint32_t needed_sincrate = AUDIO_SYS_CLOCK_HZ /( 2*1*8* 2* sample_rate); /* mic_freq / (2*16000) */
    uint32_t pdm_data = (clk_clock_div_stage_1-1)<<0;
    pdm_data |= (mclkq_clock_div_stage_2-1)<<4;
    pdm_data |= (cko_clock_div_stage_3-1)<<8;
    pdm_data |= needed_sincrate<<16;
    *pdm_reg = pdm_data;

    /* Should check the size of the mic-fifo trigger to ensure */
    /* it is 0x7f since the dma descriptors are dependent on it. */
    enable_pdm_dma( mic, dma_desc );

    result = cyhal_pdm_pcm_start(&mic->pdm_pcm);

    if (CY_RSLT_SUCCESS != result)
    {
        return result;
    }

    mic->pdm_pcm_initialized = true;

    printf("pdm_pcm: Configured device mode=%d, rate=%d, gain=%d\r\n", mode, sample_rate, gain);


    return CY_RSLT_SUCCESS;
}

/*******************************************************************************
* Function Name: _pcm_stop
********************************************************************************
* Summary:
*    A function used to de-initialize and stop the device.
*
* Parameters:
*   mic: Device handle
*
* Return:
*     The status of the initialization.
*
*******************************************************************************/
static cy_rslt_t _pcm_stop(dev_pdm_pcm_t* mic)
{
    cy_rslt_t result;

    if(mic->pdm_pcm_initialized) {
        mic->pdm_pcm_initialized = false;

        result = cyhal_pdm_pcm_stop(&mic->pdm_pcm);
        if(CY_RSLT_SUCCESS != result){
            printf("Stopping -> cyhal_pdm_pcm_stop failed\n");
            return result;
        }

        Cy_DMA_Channel_Disable(DMA_DW1_base, 26);
        Cy_DMA_Disable(DMA_DW1_base);

        result = cyhal_pdm_pcm_clear(&mic->pdm_pcm);
        if(CY_RSLT_SUCCESS != result){
            printf("Stopping -> cyhal_pdm_pcm_clear failed\n");
            return result;
        }

        cyhal_pdm_pcm_free(&mic->pdm_pcm);
    }

    return CY_RSLT_SUCCESS;
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
*   arg: Pointer the device struct (dev_pdm_pcm_t)
*
* Return:
*   True to keep the connection open, otherwise false.
*
*******************************************************************************/
static bool _configure_streams(protocol_t* protocol, int device, void* arg)
{
    UNUSED(arg);

    int status;
    int frequency_index;
    int frequency;

    status = protocol_get_option_oneof(
            protocol,
            device,
            MIC_OPTION_KEY_FREQUENCY,
            &frequency_index);
    if(status != PROTOCOL_STATUS_SUCCESS) {
        protocol_set_device_status(
                protocol,
                device,
                protocol_DeviceStatus_DEVICE_STATUS_ERROR,
                "Failed to get option frequency.");
        return true;
    }

    switch(frequency_index) {
    case 0:
        frequency = 8000;
        break;
    case 1:
        frequency = 16000;
        break;
    default:
        return false;
    }

    bool stereo;
    status = protocol_get_option_bool(protocol, device, MIC_OPTION_KEY_STEREO, &stereo);
    if(status != PROTOCOL_STATUS_SUCCESS) {
        protocol_set_device_status(
                protocol,
                device,
                protocol_DeviceStatus_DEVICE_STATUS_ERROR,
                "Failed to get option stereo.");
        return true;
    }

    if(protocol_clear_streams(protocol, device) != PROTOCOL_STATUS_SUCCESS) {
        protocol_set_device_status(
                protocol,
                device,
                protocol_DeviceStatus_DEVICE_STATUS_ERROR,
                "Failed to clear streams.");
        return true;
    }

    int max_numer_of_frames_in_chunk = stereo ? FRAME_SIZE / 2 : FRAME_SIZE;
    int stream = protocol_add_stream(
        protocol,
        device,
        "Audio",
        protocol_StreamDirection_STREAM_DIRECTION_OUTPUT,
        protocol_DataType_DATA_TYPE_S16,
        frequency,
        max_numer_of_frames_in_chunk,
        "dBFS");

    if(stream < 0) {
        protocol_set_device_status(
                protocol,
                device,
                protocol_DeviceStatus_DEVICE_STATUS_ERROR,
                "Failed to add streams.");
        return true;
    }

    if (stereo) {
        protocol_add_stream_rank(
                protocol,
                device,
                stream,
                "Channel",
                2,
                (const char* []) { "Left", "Right" });
    } else {
        protocol_add_stream_rank(
                protocol,
                device,
                stream,
                "Channel",
                1,
                (const char* []) { "Mono" });
    }

    protocol_set_device_status(
            protocol,
            device,
            protocol_DeviceStatus_DEVICE_STATUS_READY,
            "Device is ready.");

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
*   arg: Pointer the device struct (dev_pdm_pcm_t)
*
*******************************************************************************/
static void _start_streams(protocol_t* protocol, int device, pb_ostream_t* ostream, void* arg)
{
    cy_rslt_t result;
    int gain_db;
    int frequency_index;
    int sample_rate;
    bool stereo;
    dev_pdm_pcm_t* mic = (dev_pdm_pcm_t*)arg;

    UNUSED(ostream);

    protocol_get_option_oneof(protocol, device, MIC_OPTION_KEY_FREQUENCY, &frequency_index);
    protocol_get_option_bool(protocol, device, MIC_OPTION_KEY_STEREO, &stereo);
    protocol_get_option_int(protocol, device, MIC_OPTION_KEY_GAIN, &gain_db);

    switch(frequency_index) {
       case 0:
           sample_rate = 8000;
           break;
       case 1:
       default:
           sample_rate = 16000;
           break;
    }

    result = _pcm_start(
            mic,
            sample_rate,
            gain_db,
            stereo ? CYHAL_PDM_PCM_MODE_STEREO  : CYHAL_PDM_PCM_MODE_LEFT );
    if(CY_RSLT_SUCCESS != result) {
        protocol_set_device_status(
                protocol,
                device,
                protocol_DeviceStatus_DEVICE_STATUS_ERROR,
                "Failed to initialize hardware.");
        return;
    }

    protocol_set_device_status(
            protocol,
            device,
            protocol_DeviceStatus_DEVICE_STATUS_ACTIVE,
            "Device is streaming");
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
*  arg: Pointer the device struct (dev_pdm_pcm_t)
*
*******************************************************************************/
static void _stop_streams(protocol_t* protocol, int device, pb_ostream_t* ostream, void* arg)
{
    cy_rslt_t result;
    dev_pdm_pcm_t* mic = (dev_pdm_pcm_t*)arg;
    UNUSED(ostream);

    result = _pcm_stop(mic);
    if(CY_RSLT_SUCCESS != result) {
        protocol_set_device_status(
                protocol,
                device,
                protocol_DeviceStatus_DEVICE_STATUS_ERROR,
                "Failed to stop the PDM/PCM operation.");
        return;
    }

    protocol_set_device_status(
            protocol,
            device,
            protocol_DeviceStatus_DEVICE_STATUS_READY,
            "Device stopped");
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
*  arg: Pointer the device struct (dev_pdm_pcm_t)
*
*******************************************************************************/
static void _poll_streams(protocol_t* protocol, int device, pb_ostream_t* ostream, void* arg)
{
    dev_pdm_pcm_t* mic = (dev_pdm_pcm_t*)arg;

    int pend = *DW1_CH_STRUCT26_INTR & 0x01;

    if ( pend ){

        /* Full buffer is what is to be sent. */
        /* Assume active buffer is buffer that is in progress to be filled */
        /* and that this is initialized to buffer0, the first one. */
        int16_t* tmp = mic->full_rx_buffer;  /* This is the next buffer that is about to be filled by dma. */
        mic->full_rx_buffer = mic->active_rx_buffer;
        mic->active_rx_buffer = tmp;
        mic->have_data = true;

        *DW1_CH_STRUCT26_INTR = 0x01; /* Clear the interrupt */
    }

    if(mic->have_data) {

        if ( mic->init_discard_counter > 0){
            mic->init_discard_counter--;
            memset(mic->full_rx_buffer,0,FRAME_SIZE*sizeof(int16_t));
        }

        int frames_in_chunk = mic->stereo ? FRAME_SIZE / 2 : FRAME_SIZE;

        protocol_send_data_chunk(
                protocol,
                device,
                0,
                frames_in_chunk,
                mic->skipped_frames,
                ostream,
                _write_payload);

        mic->skipped_frames = 0;

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
*  arg: Pointer the device struct (dev_pdm_pcm_t)
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

    dev_pdm_pcm_t* mic = (dev_pdm_pcm_t*)arg;

    if (!pb_write(ostream, (const pb_byte_t *)mic->full_rx_buffer, total_bytes))
    {
        return false;
    }

    mic->have_data = false;

    return true;
}


/*******************************************************************************
* Function Name: dev_pdm_pcm_register
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
bool dev_pdm_pcm_register(protocol_t* protocol)
{
    int status;
    dev_pdm_pcm_t* mic = &aligned_mic;

    memset(mic, 0, sizeof(dev_pdm_pcm_t));

    if(!_init_hw(mic))
    {
        return false;
    }

    device_manager_t manager = {
        .arg = mic,
        .configure_streams = _configure_streams,
        .start = _start_streams,
        .stop = _stop_streams,
        .poll = _poll_streams,
        .data_received = NULL /* has no input streams */
    };

    int device = protocol_add_device(
        protocol,
        protocol_DeviceType_DEVICE_TYPE_SENSOR,
        "Microphone",
        "PCM Microphone",
        manager);

    if(device < 0)
    {
        return false;
    }

    status = protocol_add_option_int(
        protocol,
        device,
        MIC_OPTION_KEY_GAIN,
        "Gain",
        "Microphone gain in dB",
        10, 0, 10);

    if (status != PROTOCOL_STATUS_SUCCESS)
    {
        return false;
    }

    status = protocol_add_option_bool(
        protocol,
        device,
        MIC_OPTION_KEY_STEREO,
        "Stereo",
        "Stereo or Mono",
        false);

    if(status != PROTOCOL_STATUS_SUCCESS)
    {
        return false;
    }

    status = protocol_add_option_oneof(
        protocol,
        device,
        MIC_OPTION_KEY_FREQUENCY,
        "Frequency",
        "Sample frequency (Hz)",
        1,
        (const char* []) { "8 kHz", "16 kHz" },
        2);

    if(status != PROTOCOL_STATUS_SUCCESS)
    {
        return false;
    }

    if(!_configure_streams(protocol, device, manager.arg))
    {
        return false;
    }

    return true;
}

#endif /* IM_ENABLE_PDM_PCM */

/* [] END OF FILE */
