/******************************************************************************
* File Name:   dev_dps368.c
*
* Description:
*   This file implements the interface with the temperature/pressure sensor.
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

#ifdef IM_ENABLE_DPS368

#include <stdio.h>
#include <cyhal.h>
#include <cybsp.h>
#include <xensiv_dps3xx_mtb.h>
#include <xensiv_dps3xx.h>

#include "protocol/protocol.h"
#include "protocol/pb_encode.h"
#include "clock.h"
#include "dev_dps368.h"
#include "common.h"

/*******************************************************************************
* Macros
*******************************************************************************/
#define DPS_OPTION_KEY_FREQUENCY 1

#ifdef IM_XSS_DPS368
#define DPS368_ADDRESS (XENSIV_DPS3XX_I2C_ADDR_ALT)
#else
#define DPS368_ADDRESS (XENSIV_DPS3XX_I2C_ADDR_DEFAULT)
#endif

/*******************************************************************************
* Types
*******************************************************************************/
typedef struct {
    float data_buffer[2];
    int dropped;
    xensiv_dps3xx_t dps3xx;
    clock_tick_t sample_time_tick;
    uint32_t period_tick;
} dev_dps368_t;

/*******************************************************************************
* Function Prototypes
*******************************************************************************/
static bool _init_hw(dev_dps368_t* dps, cyhal_i2c_t* i2c);
static bool _config_hw(dev_dps368_t* dps, int rate);

static bool _configure_streams(protocol_t* protocol, int device, void* arg);
static void _start_streams(protocol_t* protocol, int device, pb_ostream_t* ostream, void* arg);
static void _stop_streams(protocol_t* protocol, int device, pb_ostream_t* ostream, void* arg);
static bool _write_payload(
    protocol_t* protocol,
    int device_id,
    int stream_id,
    int frame_count,
    int total_bytes,
    pb_ostream_t* ostream,
    void* arg);
static void _poll_streams(protocol_t* protocol, int device, pb_ostream_t* ostream, void* arg);

/*******************************************************************************
* Function Definitions
*******************************************************************************/

/* There is a copy-paste bug in xensiv_dps3xx.c in on line
 * 681-682. Is says user_config.temperature_rate and
 * user_config.temperature_oversample but should be pressure_oversample and
 * pressure_rate.
 *
 * This is a workaround for that.
 */
#define HAL_XENSIV_DPS3XX_BUG_WORKAROUND

#ifdef HAL_XENSIV_DPS3XX_BUG_WORKAROUND

#define PRS_CFG_6 0x06
#define TMP_CFG_7 0x07
#define CFG_REG_9 0x09
#define T_SHIFT 0x08
#define P_SHIFT 0x04

#define XENSIV_DPS3XX_PSR_READ_REG_ADDR             (0x00)
#define XENSIV_DPS3XX_PSR_READ_LEN                  (3)
#define XENSIV_DPS3XX_TMP_READ_REG_ADDR             (0x03)
#define XENSIV_DPS3XX_TMP_READ_LEN                  (3)
#define XENSIV_DPS3XX_PSR_TMP_READ_REG_ADDR         (0x00)
#define XENSIV_DPS3XX_PSR_TMP_READ_LEN              (0x06)

#define POW_2_23_MINUS_1                            (0x7FFFFF)
#define POW_2_24                                    (0x1000000)

static cy_rslt_t _xensiv_dps3xx_reg_read(
        xensiv_dps3xx_t* dev,
        uint8_t reg_addr,
        uint8_t* data,
        uint8_t length)
{
    return dev->comm.read(
            dev->comm.context,
            dev->user_config.i2c_timeout,
            dev->i2c_address,
            reg_addr,
            data,
            length);
}

static cy_rslt_t _xensiv_dps3xx_reg_write(
        xensiv_dps3xx_t* dev,
        uint8_t reg_addr,
        uint8_t* data,
        uint8_t length)
{
    return dev->comm.write(
            dev->comm.context,
            dev->user_config.i2c_timeout,
            dev->i2c_address,
            reg_addr,
            data,
            length);
}

/*--------------------------------------------------------------------------------------------------
 * _xensiv_dps3xx_calc_temperature
 *------------------------------------------------------------------------------------------------*/
static float _xensiv_dps3xx_calc_temperature(xensiv_dps3xx_t* dev, int32_t temp_raw)
{
    if (temp_raw > POW_2_23_MINUS_1)
    {
        temp_raw -= POW_2_24;
    }
    dev->temp_scaled = (float)temp_raw / (float)dev->tmp_osr_scale_coeff;

    int64_t c0 = dev->calib_coeffs.C0;
    int64_t c1 = dev->calib_coeffs.C1;
    return (c0 / 2.0f) + (c1 * dev->temp_scaled);
}


/*--------------------------------------------------------------------------------------------------
 * _xensiv_dps3xx_calc_pressure
 *------------------------------------------------------------------------------------------------*/
static float _xensiv_dps3xx_calc_pressure(xensiv_dps3xx_t* dev, int32_t press_raw)
{
    if (press_raw > POW_2_23_MINUS_1)
    {
        press_raw -= POW_2_24;
    }

    float press_scaled = (float)press_raw / dev->prs_osr_scale_coeff;
    int64_t c00 = dev->calib_coeffs.C00;
    int64_t c10 = dev->calib_coeffs.C10;
    int64_t c20 = dev->calib_coeffs.C20;
    int64_t c30 = dev->calib_coeffs.C30;
    int64_t c01 = dev->calib_coeffs.C01;
    int64_t c11 = dev->calib_coeffs.C11;
    int64_t c21 = dev->calib_coeffs.C21;

    float Pcal = c00
                 + press_scaled * (c10 + press_scaled * (c20 + press_scaled * c30))
                 + (dev->temp_scaled * c01)
                 + (dev->temp_scaled * press_scaled * (c11 + press_scaled * c21));
    return Pcal * 0.01f;
}


static bool enable_shift( xensiv_dps3xx_t* dev ){
    cy_rslt_t result;
    uint8_t reg_val;
    result = _xensiv_dps3xx_reg_read(dev, CFG_REG_9, &reg_val, 1);
    if (result != CY_RSLT_SUCCESS)
    {
        return false;
    }

    reg_val |= T_SHIFT | P_SHIFT;
    result = _xensiv_dps3xx_reg_write(dev, CFG_REG_9, &reg_val, 1);
    if (result != CY_RSLT_SUCCESS)
    {
        return false;
    }

    return true;
}

static bool disable_shift( xensiv_dps3xx_t* dev ){
    cy_rslt_t result;
    uint8_t reg_val;
    result = _xensiv_dps3xx_reg_read(dev, CFG_REG_9, &reg_val, 1);
    if (result != CY_RSLT_SUCCESS)
    {
        return false;
    }

    reg_val &= ~(T_SHIFT | P_SHIFT);
    result = _xensiv_dps3xx_reg_write(dev, CFG_REG_9, &reg_val, 1);
    if (result != CY_RSLT_SUCCESS)
    {
        return false;
    }

    return true;
}

static bool apply_hal_xensiv_dps_3xx_bug_workaround(
        dev_dps368_t* dps,
        xensiv_dps3xx_config_t *config)
{
    cy_rslt_t result;
    uint8_t reg_val;

    reg_val = (uint8_t)config->pressure_rate | (uint8_t)config->pressure_oversample;
    result = _xensiv_dps3xx_reg_write(&dps->dps3xx, PRS_CFG_6, &reg_val, 1);
    if (result != CY_RSLT_SUCCESS)
    {
        return false;
    }

    reg_val = (uint8_t)config->temperature_rate | (uint8_t)config->temperature_oversample;
    result = _xensiv_dps3xx_reg_write(&dps->dps3xx, PRS_CFG_6, &reg_val, 1);
    if (result != CY_RSLT_SUCCESS)
    {
        return false;
    }


    /* This should be the same for the temperature */
    switch (config->pressure_oversample)
    {
        case XENSIV_DPS3XX_OVERSAMPLE_1:
            dps->dps3xx.prs_osr_scale_coeff = _XENSIV_DPS3XX_OSR_SF_1;
            break;

        case XENSIV_DPS3XX_OVERSAMPLE_2:
            dps->dps3xx.prs_osr_scale_coeff = _XENSIV_DPS3XX_OSR_SF_2;
            break;

        case XENSIV_DPS3XX_OVERSAMPLE_4:
            dps->dps3xx.prs_osr_scale_coeff = _XENSIV_DPS3XX_OSR_SF_4;
            break;

        case XENSIV_DPS3XX_OVERSAMPLE_8:
            dps->dps3xx.prs_osr_scale_coeff = _XENSIV_DPS3XX_OSR_SF_8;
            break;

        case XENSIV_DPS3XX_OVERSAMPLE_16:
            dps->dps3xx.prs_osr_scale_coeff = _XENSIV_DPS3XX_OSR_SF_16;
            break;

        case XENSIV_DPS3XX_OVERSAMPLE_32:
            dps->dps3xx.prs_osr_scale_coeff = _XENSIV_DPS3XX_OSR_SF_32;
            break;

        case XENSIV_DPS3XX_OVERSAMPLE_64:
            dps->dps3xx.prs_osr_scale_coeff = _XENSIV_DPS3XX_OSR_SF_64;
            break;

        case XENSIV_DPS3XX_OVERSAMPLE_128:
            dps->dps3xx.prs_osr_scale_coeff = _XENSIV_DPS3XX_OSR_SF_128;
            break;
    }


    switch (config->temperature_oversample)
    {
        case XENSIV_DPS3XX_OVERSAMPLE_1:
            dps->dps3xx.tmp_osr_scale_coeff = _XENSIV_DPS3XX_OSR_SF_1;
            break;

        case XENSIV_DPS3XX_OVERSAMPLE_2:
            dps->dps3xx.tmp_osr_scale_coeff = _XENSIV_DPS3XX_OSR_SF_2;
            break;

        case XENSIV_DPS3XX_OVERSAMPLE_4:
            dps->dps3xx.tmp_osr_scale_coeff = _XENSIV_DPS3XX_OSR_SF_4;
            break;

        case XENSIV_DPS3XX_OVERSAMPLE_8:
            dps->dps3xx.tmp_osr_scale_coeff = _XENSIV_DPS3XX_OSR_SF_8;
            break;

        case XENSIV_DPS3XX_OVERSAMPLE_16:
            dps->dps3xx.tmp_osr_scale_coeff = _XENSIV_DPS3XX_OSR_SF_16;
            break;

        case XENSIV_DPS3XX_OVERSAMPLE_32:
            dps->dps3xx.tmp_osr_scale_coeff = _XENSIV_DPS3XX_OSR_SF_32;
            break;

        case XENSIV_DPS3XX_OVERSAMPLE_64:
            dps->dps3xx.tmp_osr_scale_coeff = _XENSIV_DPS3XX_OSR_SF_64;
            break;

        case XENSIV_DPS3XX_OVERSAMPLE_128:
            dps->dps3xx.tmp_osr_scale_coeff = _XENSIV_DPS3XX_OSR_SF_128;
            break;
    }

    return true;
}

#endif /* HAL_XENSIV_DPS3XX_BUG_WORKAROUND */

/*******************************************************************************
* Function Name: _init_hw
********************************************************************************
* Summary:
*    A function used to initialize the DPS368 Pressure sensor.
*
* Parameters:
*  dps: Pointer to the device struct (dev_dps368_t)
*  i2c: Pointer to the I2C instance
*
* Return:
*   True if initialization is successful, otherwise false.
*
*******************************************************************************/
static bool _init_hw(dev_dps368_t* dps, cyhal_i2c_t* i2c)
{
    cy_rslt_t result;

    /* Initialize pressure sensor */
    result = xensiv_dps3xx_mtb_init_i2c(&dps->dps3xx, i2c, DPS368_ADDRESS);
    if (result != CY_RSLT_SUCCESS)
    {
        return false;
    }

    printf("dps368: Initialized device.\r\n");

    return true;
}

/*******************************************************************************
* Function Name: _config_hw
********************************************************************************
* Summary:
*    A function used to configure the DPS368 Pressure sensor.
*
* Parameters:
*   dps: Pointer to the device struct (dev_dps368_t)
*   rate: Sampling rate
*
* Return:
*   True if configuration is successful, otherwise false.
*
*******************************************************************************/
static bool _config_hw(dev_dps368_t* dps, int rate)
{
    cy_rslt_t result;
    xensiv_dps3xx_config_t config;

    /* Clear data */
    memset(&dps->data_buffer, 0, sizeof(float) * 2);

    /* Gets the current configuration parameters. */
    result = xensiv_dps3xx_get_config(&dps->dps3xx, &config);
    if (result != CY_RSLT_SUCCESS)
    {
        return false;
    }

    config.fifo_enable = false;
    config.dev_mode = XENSIV_DPS3XX_MODE_BACKGROUND_ALL;

    switch(rate) {
    case 8: /* This will take approx 2*220 = 440 ms */
        enable_shift( &dps->dps3xx );   /* Shift will also be enabled through the hal but  */
                                        /* also here for clarity. */
        config.pressure_oversample = XENSIV_DPS3XX_OVERSAMPLE_16;
        config.temperature_oversample = XENSIV_DPS3XX_OVERSAMPLE_16;
        config.pressure_rate = XENSIV_DPS3XX_RATE_8;
        config.temperature_rate = XENSIV_DPS3XX_RATE_8;
        break;
    case 16: /* This will take approx 2*236 = 472 ms */
        disable_shift( &dps->dps3xx );  /* Hal have no code for disabling, hence this code here. */
        config.pressure_oversample = XENSIV_DPS3XX_OVERSAMPLE_8;
        config.temperature_oversample = XENSIV_DPS3XX_OVERSAMPLE_8;
        config.pressure_rate = XENSIV_DPS3XX_RATE_16;
        config.temperature_rate = XENSIV_DPS3XX_RATE_16;
        break;
    case 32: /* This will take approx 2*268 = 536 ms */
        disable_shift( &dps->dps3xx );    /* Hal have no code for disabling, hence this code here. */
        config.pressure_oversample = XENSIV_DPS3XX_OVERSAMPLE_4;
        config.temperature_oversample = XENSIV_DPS3XX_OVERSAMPLE_4;
        config.pressure_rate = XENSIV_DPS3XX_RATE_32;
        config.temperature_rate = XENSIV_DPS3XX_RATE_32;
        break;
    case 64: /* This will take approx 2*332 = 664 ms */
        disable_shift( &dps->dps3xx );    /* Hal have no code for disabling, hence this code here. */
        config.pressure_oversample = XENSIV_DPS3XX_OVERSAMPLE_2;
        config.temperature_oversample = XENSIV_DPS3XX_OVERSAMPLE_2;
        config.pressure_rate = XENSIV_DPS3XX_RATE_64;
        config.temperature_rate = XENSIV_DPS3XX_RATE_64;
        break;
    case 128: /* This will take approx 2*460 = 920 ms */
        disable_shift( &dps->dps3xx );    /* Hal have no code for disabling, hence this code here. */
        config.pressure_oversample = XENSIV_DPS3XX_OVERSAMPLE_1;
        config.temperature_oversample = XENSIV_DPS3XX_OVERSAMPLE_1;
        config.pressure_rate = XENSIV_DPS3XX_RATE_128;
        config.temperature_rate = XENSIV_DPS3XX_RATE_128;
        break;
    default:
        return false;
    }

    /* Update sample period */
    dps->sample_time_tick = 0;
    dps->period_tick = CLOCK_TICK_PER_SECOND / rate;

    /* Ensure to have this first and then let the workaround */
    /* override what can be needed. */
    /* Sets the current configuration parameters for the sensor.  */
    result = xensiv_dps3xx_set_config(&dps->dps3xx, &config);
    if (result != CY_RSLT_SUCCESS)
    {
        printf("xensiv_dps3xx_set_config FAILED\r\n");
        return false;
    }

    #ifdef HAL_XENSIV_DPS3XX_BUG_WORKAROUND
    if(!apply_hal_xensiv_dps_3xx_bug_workaround(dps, &config))
    {
        printf("WORKAROUND FAILED\r\n");
        return false;
    }
    #endif

    /* Poll and read out data to clear it from some junk. */
    /* Perhaps the sensor isn't ready? */
    xensiv_dps3xx_read(&dps->dps3xx, &dps->data_buffer[0], &dps->data_buffer[1]);
    xensiv_dps3xx_read(&dps->dps3xx, &dps->data_buffer[0], &dps->data_buffer[1]);
    xensiv_dps3xx_read(&dps->dps3xx, &dps->data_buffer[0], &dps->data_buffer[1]);

    printf("dps368: Configured device. rate=%d Hz\r\n", rate);

    return true;
}

/*******************************************************************************
* Function Name: _configure_streams
********************************************************************************
* Summary:
*  Called when the device stream is configured or re-configured.
*
* Parameters:
*  protocol: pointer the protocol handle
*  device: the device index
*  arg: pointer the device struct (dev_dps368_t)
*
*  Return:
*    True to keep the connection open, false to close.
*
*******************************************************************************/
static bool _configure_streams(protocol_t* protocol, int device, void* arg)
{
    int rate_index;
    int rate;
    int status;
    dev_dps368_t* dps = (dev_dps368_t*)arg;

    status = protocol_get_option_oneof(protocol, device, DPS_OPTION_KEY_FREQUENCY, &rate_index);
    if(status != PROTOCOL_STATUS_SUCCESS) {
        protocol_set_device_status(
                protocol,
                device,
                protocol_DeviceStatus_DEVICE_STATUS_ERROR,
                "Failed to get option frequency.");
        return true;
    }

    switch(rate_index) {
        case 0: rate = 8; break;
        case 1: rate = 16; break;
        case 2: rate = 32; break;
        case 3: rate = 64; break;
        case 4: rate = 128; break;
        default: return false;
   }

    if(protocol_clear_streams(protocol, device) != PROTOCOL_STATUS_SUCCESS) {
        protocol_set_device_status(
                protocol,
                device,
                protocol_DeviceStatus_DEVICE_STATUS_ERROR,
                "Failed to clear streams.");
        return true;
    }

    int stream = protocol_add_stream(
        protocol,
        device,
        "Data",
        protocol_StreamDirection_STREAM_DIRECTION_OUTPUT,
        protocol_DataType_DATA_TYPE_F32,
        rate,
        1,
        NULL);

    if(stream < 0) {
        protocol_set_device_status(
                protocol,
                device,
                protocol_DeviceStatus_DEVICE_STATUS_ERROR,
                "Failed to add streams.");
        return true;
    }

    status = protocol_add_stream_rank(
            protocol,
            device,
            stream,
            "Channel",
            2,
            (const char* []) { "Pressure", "Temp" });
    if(status != PROTOCOL_STATUS_SUCCESS) {
            protocol_set_device_status(
                    protocol,
                    device,
                    protocol_DeviceStatus_DEVICE_STATUS_ERROR,
                    "Failed to add stream dimension.");
            return true;
        }

    if(!_config_hw(dps, rate)) {
        protocol_set_device_status(
                protocol,
                device,
                protocol_DeviceStatus_DEVICE_STATUS_ACTIVE,
                "Failed to configure device");
    } else {
        protocol_set_device_status(
                protocol,
                device,
                protocol_DeviceStatus_DEVICE_STATUS_READY,
                "Device is ready.");
    }
    return true;
}

/*******************************************************************************
* Function Name: _start_streams
********************************************************************************
* Summary:
*  Called when streaming is started. This may also initialize the device.
*
* Parameters:
*  protocol: pointer the protocol handle
*  device: the device index
*  ostream: Pointer to the output stream to write to
*  arg: pointer the device struct (dev_dps368_t)
*
*******************************************************************************/
static void _start_streams(protocol_t* protocol, int device, pb_ostream_t* ostream, void* arg)
{
    UNUSED(ostream);

    dev_dps368_t* dps = (dev_dps368_t*)arg;
    clock_tick_t time = clock_get_tick();

    protocol_set_device_status(
            protocol,
            device,
            protocol_DeviceStatus_DEVICE_STATUS_ACTIVE,
            "Device is streaming");

    /* Initialize this as the start time. One sample period will pass */
    /* before the first sample is taken. */
    dps->sample_time_tick = time;
}

/*******************************************************************************
* Function Name: _stop_streams
********************************************************************************
* Summary:
*  Called when streaming is stopped.
*
* Parameters:
*  protocol: pointer the protocol handle
*  device: the device index
*  ostream: Pointer to the output stream to write to
*  arg: pointer the device struct (dev_dps368_t)
*
*******************************************************************************/
static void _stop_streams(protocol_t* protocol, int device, pb_ostream_t* ostream, void* arg)
{
    UNUSED(arg);
    UNUSED(ostream);

    protocol_set_device_status(
            protocol,
            device,
            protocol_DeviceStatus_DEVICE_STATUS_READY,
            "Device stopped");
}


/*******************************************************************************
* Function Name: _write_payload
********************************************************************************
* Summary:
*  Used by protocol_send_data_chunk to write the actual data.
*
* Parameters:
*  protocol: pointer the protocol handle
*  stream_id: the device index
*  stream_id: the stream index
*  frame_count: number of frames to write
*  total_bytes: total number of bytes to write (= frame_count * sizeof(type) * frame_shape.flat)
*  ostream: pointer to the output stream to write to
*  arg: pointer the device struct (dev_dps368_t)
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

    dev_dps368_t* dps = (dev_dps368_t*)arg;

    if (!pb_write(ostream, (const pb_byte_t *)dps->data_buffer, total_bytes))
    {
        return false;
    }

    return true;
}

/*******************************************************************************
* Function Name: _poll_streams
********************************************************************************
* Summary:
*  Called periodically to send data messages when the device is active.
*
* Parameters:
*  protocol: pointer the protocol handle
*  device: the device index
*  ostream: pointer to the output stream to write to
*  arg: pointer the device struct (dev_dps368_t)
*
*******************************************************************************/
static void _poll_streams(protocol_t* protocol, int device, pb_ostream_t* ostream, void* arg)
{
    cy_rslt_t result;
    dev_dps368_t* dps = (dev_dps368_t*)arg;

    clock_tick_t time = clock_get_tick();

    if(time >= dps->sample_time_tick + dps->period_tick ) {

        /* dps->sample_time_tick = time;
         * Use a fixed calculation for when this tick should have happened.
         * This also mean that the dps->sample_time_tick need to be initialized
         * whenever a new streaming is started. */
        dps->sample_time_tick += dps->period_tick;

        /* NOTE: In xensiv_dps3xx_read there is a call to _xensiv_dps3xx_read_raw_values
         * which in turn calls _xensiv_dps3xx_wait_data_ready. This is not optimal
         * and should be avoided. This may be the cause for some dropped packages.
         * Dropped packages is a problem when the data rate is very high.
         * Using a polling technique requires really good timing to synchronize.
         * Check data ready above could be a solution but it also means that there will be
         * a wait loop which have its own danger.
         * It would be better to call _xensiv_dps3xx_reg_read directly to minimize delays.
         * Also for the future split of streams it is beneficial to have it here.
         */

        uint8_t read_reg_addr = XENSIV_DPS3XX_PSR_TMP_READ_REG_ADDR;
        uint8_t read_reg_len = XENSIV_DPS3XX_PSR_TMP_READ_LEN;
        uint8_t raw_value[XENSIV_DPS3XX_PSR_TMP_READ_LEN];
        result = _xensiv_dps3xx_reg_read(&dps->dps3xx, read_reg_addr, raw_value, read_reg_len);


        int32_t press_raw = (int32_t)(raw_value[2]) + (raw_value[1] << 8) + (raw_value[0] << 16);
        int32_t temp_raw = (int32_t)(raw_value[5]) + (raw_value[4] << 8) + (raw_value[3] << 16);
        dps->data_buffer[0] = _xensiv_dps3xx_calc_pressure(&dps->dps3xx, press_raw);
        dps->data_buffer[1] = _xensiv_dps3xx_calc_temperature(&dps->dps3xx, temp_raw);

        if (result != CY_RSLT_SUCCESS) {
            printf("xensiv_dps3xx_read READ FAILED. Frame dropped!\r\n");
            dps->dropped++;
        } else {
            protocol_send_data_chunk(protocol, device, 0, 1, dps->dropped, ostream, _write_payload);
            dps->dropped = 0;
        }
    }
}

/*******************************************************************************
* Function Name: dev_dps368_register
********************************************************************************
* Summary:
*  Registers this device. This is the only exported symbol from this object.
*
* Parameters:
*  protocol: Pointer to the protocol handle.
*  i2c: Pointer to the I2C instance.
*
* Returns:
*  True on success, else false.
*
*******************************************************************************/
bool dev_dps368_register(protocol_t* protocol, cyhal_i2c_t* i2c)
{
    int status;

    dev_dps368_t* dps = (dev_dps368_t*)malloc(sizeof(dev_dps368_t));
    if(dps == NULL)
    {
        return false;
    }
    memset(dps, 0, sizeof(dev_dps368_t));

    /* Initialize the DPS device with the provided I2C interface */
    if(!_init_hw(dps, i2c))
    {
        free(dps);
        return false;
    }

    device_manager_t manager = {
        .arg = dps,
        .configure_streams = _configure_streams,
        .start = _start_streams,
        .stop = _stop_streams,
        .poll = _poll_streams,
        .data_received = NULL /* has no input streams */
    };

    /* Add the DPS device to the protocol and get its device identifier */
    int device = protocol_add_device(
        protocol,
        protocol_DeviceType_DEVICE_TYPE_SENSOR,
        "DPS",
        "Pressure and Temperature (DPS368)",
        manager);

    if(device < 0)
    {
        return false;
    }

    /* Add an option to configure the sampling frequency of the DPS device */
    status = protocol_add_option_oneof(
        protocol,
        device,
        DPS_OPTION_KEY_FREQUENCY,
        "Frequency",
        "Sample frequency (Hz). As frequency goes up, oversampling goes down.",
        3,
        (const char* []) { "8 Hz", "16 Hz", "32 Hz", "64 Hz", "128 Hz" },
        5);

    if(status != PROTOCOL_STATUS_SUCCESS)
    {
        return false;
    }

    /* Configure the data streams for the DPS device using the protocol */
    if(!_configure_streams(protocol, device, manager.arg))
    {
        return false;
    }

    return true;
}

#endif /* IM_ENABLE_DPS368 */

/* [] END OF FILE */
