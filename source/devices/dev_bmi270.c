/******************************************************************************
* File Name: dev_bmi270.c
*
* Description: This file implements the interface with the bmi270 sensor.
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

#ifdef IM_ENABLE_BMI270

#include <cyhal.h>
#include <cybsp.h>
#include <bmi270.h>

#include "protocol/protocol.h"
#include "protocol/pb_encode.h"
#include "clock.h"
#include "dev_bmi270.h"
#include "common.h"


/*******************************************************************************
* Macros
*******************************************************************************/

#define DEV_BMI270_OPTION_KEY_RATE         (1)
#define DEV_BMI270_OPTION_KEY_ACCEL_RANGE  (2)
#define DEV_BMI270_OPTION_KEY_GYRO_RANGE   (3)
#define DEV_BMI270_OPTION_KEY_STREAM_MODE  (4)

#define _I2C_TIMEOUT_MS            (10U)
#define _READ_WRITE_LEN            (46U)
#define _SOFT_RESET_DELAY_US       (300)

#ifdef IM_XSS_BMI270
#define BMI270_ADDRESS (BMI2_I2C_SEC_ADDR)
#else
#define BMI270_ADDRESS (BMI2_I2C_PRIM_ADDR)
#endif

/* At 400Hz/8 = 50 Hz chunk frequency */
/* Increase this to 16 if 800 Hz mode is enabled */
#define MAX_FRAMES_IN_CHUNK   (8)

/* X Y Z */
#define AXIS_COUNT            (3)

/* Accel, Gyro */
#define SENSOR_COUNT          (2)

/* Earth's gravity in m/s^2 */
#define GRAVITY_EARTH  (9.80665f)

/*******************************************************************************
* Types
*******************************************************************************/

typedef enum {
    /* Both accelerometer and gyroscope are in the same stream.
     * The shape of each frame will be [2,3] as
     * Stream 0: {{ACCEL_X, ACCEL_Y, ACCEL_Z}, {GYRO_X, GYRO_Y, GYRO_Z}} */
    BMI270_MODE_STREAM_COMBINED = 0,

    /* The accelerometer and gyroscope are split in two separate streams
     * each with the shape [3] as
     * Stream 0: {ACCEL_X, ACCEL_Y, ACCEL_Z}
     * Stream 1: {GYRO_X, GYRO_Y, GYRO_Z}
     */
    BMI270_MODE_STREAM_SPLIT = 3,

    /* Only the accelerometer is enabled.
     * Stream 0: {ACCEL_X, ACCEL_Y, ACCEL_Z}
     */
    BMI270_MODE_STREAM_ONLY_ACCEL = 1,

    /* Only the gyroscope is enabled.
     * Stream 0: {GYRO_X, GYRO_Y, GYRO_Z}
     */
    BMI270_MODE_STREAM_ONLY_GYRO = 2,
} stream_mode_t;

typedef struct {
    /* Hardware device */
    struct bmi2_dev sensor;

    /* Accelerator range in G. Valid values are: 2,4,8,16 */
    float accel_range;

    /* Gyroscope range in DPS. Valid values are: 125,250,500,1000,2000 */
    float gyro_range;

    /* Tick of last sample */
    clock_tick_t sample_time_tick;
    /* Used to ensure the first read */
    bool first_sample;

    /* The sample period in ticks */
    uint32_t period_tick;

    union {
        struct { /* When mode is BMI270_MODE_STREAM_SPLIT or BMI270_MODE_STREAM_ONLY_XXX */
            /* Converted data as meter per second squared */
            float accel_data[MAX_FRAMES_IN_CHUNK * AXIS_COUNT];

            /* Converted data as degrees per second*/
            float gyro_data[MAX_FRAMES_IN_CHUNK * AXIS_COUNT];
        };

        /* When mode is BMI270_MODE_STREAM_COMBINED */
        float data_compined[MAX_FRAMES_IN_CHUNK * SENSOR_COUNT * AXIS_COUNT];
    };

    /* Number of frames collected in accel_data and gyro_data. */
    /* Cleared after each sent data-chunk. Equal or less than frames_in_chunk */
    int frames_sampled;

    /* Max number of frames in each chunk. Is less or equal to MAX_FRAMES_IN_CHUNK*/
    int frames_target;

    /* Number of frames dropped. This is cleared each data-chunk. */
    int frames_dropped;

    /* How streams are presented. */
    stream_mode_t stream_mode;
} dev_bmi270_t;

/*******************************************************************************
* Function Prototypes
*******************************************************************************/
static BMI2_INTF_RETURN_TYPE _bmi2_i2c_read(
        uint8_t reg_addr,
        uint8_t* reg_data,
        uint32_t len,
        void* intf_ptr);

static BMI2_INTF_RETURN_TYPE _bmi2_i2c_write(
        uint8_t reg_addr,
        const uint8_t* reg_data,
        uint32_t len,
        void* intf_ptr);

static void _bmi2_delay_us(uint32_t us, void* intf_ptr);

static float _lsb_to_mps2(int16_t val, float g_range, uint8_t bit_width);

static float _lsb_to_dps(int16_t val, float dps, uint8_t bit_width);

static bool _init_hw(dev_bmi270_t *dev, cyhal_i2c_t* i2c);

static bool _config_hw(
        dev_bmi270_t* dev,
        int rate,
        int accel_range,
        int gyro_range,
        stream_mode_t mode);

static bool _read_hw(dev_bmi270_t* dev);

static bool _configure_streams(protocol_t* protocol, int device, void* arg);

static void _start_streams(protocol_t* protocol, int device, pb_ostream_t* ostream, void* arg);

static void _stop_streams(protocol_t* protocol, int device, pb_ostream_t* ostream, void* arg);

static void _poll_streams(
        protocol_t* protocol,
        int device,
        pb_ostream_t* ostream,
        void* arg);

static bool _write_payload(
    protocol_t* protocol,
    int device_id,
    int stream_id,
    int frame_count,
    int total_bytes,
    pb_ostream_t* ostream,
    void* arg);

/*******************************************************************************
* Function Definitions
*******************************************************************************/

/*****************************************************************************
* Function name: _bmi2_i2c_read
*****************************************************************************
* Summary:
* This internal function reads I2C function map to host MCU
*
* Parameters:
*  reg_addr    8bit register address of the sensor
*  reg_data    Data from the specified address
*  len         Length of the reg_data array
*  intf_ptr    Void pointer that can enable the linking of descriptors for
*  interface related callbacks
*
* Return:
*  int8_t     Status of execution
*
*****************************************************************************/
static BMI2_INTF_RETURN_TYPE _bmi2_i2c_read(
        uint8_t reg_addr,
        uint8_t* reg_data,
        uint32_t len,
        void* intf_ptr)
{
    cyhal_i2c_t *i2c = (cyhal_i2c_t*)intf_ptr;

    return (BMI2_INTF_RETURN_TYPE)cyhal_i2c_master_mem_read(
        i2c,
        BMI270_ADDRESS,
        reg_addr,
        1,
        reg_data,
        (uint16_t)len,
        _I2C_TIMEOUT_MS);
}

/*****************************************************************************
* Function name: _bmi2_i2c_write
*****************************************************************************
* Summary:
* This internal function writes I2C function map to host MCU
*
* Parameters:
*  reg_addr    8bit register address of the sensor
*  reg_data    Data from the specified address
*  len         Length of the reg_data array
*  intf_ptr    Void pointer that can enable the linking of descriptors for
*  interface related callbacks
*
* Return:
*  int8_t     Status of execution
*
*****************************************************************************/
static BMI2_INTF_RETURN_TYPE _bmi2_i2c_write(
        uint8_t reg_addr,
        const uint8_t* reg_data,
        uint32_t len,
        void* intf_ptr)
{
    cyhal_i2c_t *i2c = (cyhal_i2c_t*)intf_ptr;

    return (BMI2_INTF_RETURN_TYPE)cyhal_i2c_master_mem_write(
        i2c,
        BMI270_ADDRESS,
        reg_addr,
        1,
        reg_data,
        (uint16_t)len,
        _I2C_TIMEOUT_MS);
}

/*****************************************************************************
* Function name: _bmi2_delay_us
*****************************************************************************
* Summary:
* This internal function maps delay function to host MCU
*
* Parameters:
*  us    The time period in microseconds
*  intf_ptr  Void pointer that can enable the linking of descriptors for
*  interface related callbacks
*
* Return:
*  void
*
*****************************************************************************/
static void _bmi2_delay_us(uint32_t us, void* intf_ptr)
{
    UNUSED(intf_ptr);

    cyhal_system_delay_us(us);
}

/******************************************************************************
* Function Name: _lsb_to_mps2
********************************************************************************
* Summary:
*   This function converts lsb to meter per second squared for 16 bit accelerometer
*   at range 2G, 4G, 8G or 16G.
*/
static float _lsb_to_mps2(int16_t val, float g_range, uint8_t bit_width)
{
    double power = 2;

    float half_scale = (float)((pow((double)power, (double)bit_width) / 2.0f));

    return (GRAVITY_EARTH * val * g_range) / half_scale;
}

/*******************************************************************************
* Function Name: _lsb_to_dps
********************************************************************************
* Summary:
*   This function converts lsb to degree per second for 16 bit gyro at
*   range 125, 250, 500, 1000 or 2000dps.
*******************************************************************************/
static float _lsb_to_dps(int16_t val, float dps, uint8_t bit_width)
{
    double power = 2;

    float half_scale = (float)((pow((double)power, (double)bit_width) / 2.0f));

    return (dps / (half_scale)) * (val);
}

/******************************************************************************
* Function Name: _init_hw
********************************************************************************
* Summary:
*   Initializes the bmi270 IMU.
*
* Parameters:
*   dev: Pointer to the dev_bmi270_t device handle.
*   i2c: Pointer to the I2C interface resource.
*
* Return:
*   True if operation is successful, otherwise false.
*
*******************************************************************************/
static bool _init_hw(dev_bmi270_t *dev, cyhal_i2c_t* i2c)
{
    dev->sensor.intf = BMI2_I2C_INTF;
    dev->sensor.read = _bmi2_i2c_read;
    dev->sensor.write = _bmi2_i2c_write;
    dev->sensor.delay_us = _bmi2_delay_us;
    dev->sensor.intf_ptr = i2c;
    dev->sensor.read_write_len = _READ_WRITE_LEN;
    dev->sensor.config_file_ptr = NULL;

    dev->stream_mode = BMI270_MODE_STREAM_COMBINED;

    if(bmi270_init(&(dev->sensor)) != BMI2_OK)
    {
        return false;
    }

    printf("bmi270: Initialized device.\r\n");

    return true;
}

/******************************************************************************
* Function Name: _config_hw
********************************************************************************
* Summary:
*   Configures the bmi270 output data rate and measurement range.
*
* Parameters:
*   dev: Pointer to the dev_bmi270_t device handle.
*   rate: Sample frequency (Hz).
*   accel_range: Measurement range in G.
*   gyro_range: Measurement range in DPS.
*   mode: Stream mode.
*
* Return:
*   True if configuration is successful, otherwise false.
*
*******************************************************************************/
static bool _config_hw(dev_bmi270_t* dev, int rate, int accel_range, int gyro_range, stream_mode_t mode)
{
    int8_t result;

    dev->sample_time_tick = 0;
    dev->first_sample = true;
    dev->period_tick = CLOCK_TICK_PER_SECOND / rate;
    dev->frames_dropped = 0;
    dev->frames_sampled = 0;
    dev->accel_range = accel_range;
    dev->gyro_range = gyro_range;
    dev->stream_mode = mode;

    struct bmi2_sens_config config[SENSOR_COUNT];
    config[0].type = BMI2_ACCEL;
    config[1].type = BMI2_GYRO;

    /* Get sensor configuration */
    result = bmi2_get_sensor_config(config, SENSOR_COUNT, &(dev->sensor));
    if (BMI2_OK != result)
    {
        return result;
    }

    /* Set output data rate and range based on rate and range parameters */
    switch(rate) {
        case 25:
            config[0].cfg.acc.odr = BMI2_ACC_ODR_25HZ;
            config[1].cfg.gyr.odr = BMI2_GYR_ODR_25HZ;
            dev->frames_target = 1;
            break;
        case 50:
            config[0].cfg.acc.odr = BMI2_ACC_ODR_50HZ;
            config[1].cfg.gyr.odr = BMI2_GYR_ODR_50HZ;
            dev->frames_target = 1;
            break;
        case 100:
            config[0].cfg.acc.odr = BMI2_ACC_ODR_100HZ;
            config[1].cfg.gyr.odr = BMI2_GYR_ODR_100HZ;
            dev->frames_target = 2;
            break;
        case 200:
            config[0].cfg.acc.odr = BMI2_ACC_ODR_200HZ;
            config[1].cfg.gyr.odr = BMI2_GYR_ODR_200HZ;
            dev->frames_target = 4;
            break;
        case 400:
            config[0].cfg.acc.odr = BMI2_ACC_ODR_400HZ;
            config[1].cfg.gyr.odr = BMI2_GYR_ODR_400HZ;
            dev->frames_target = 8;
            break;
        case 800:
            config[0].cfg.acc.odr = BMI2_ACC_ODR_800HZ;
            config[1].cfg.gyr.odr = BMI2_GYR_ODR_800HZ;
            dev->frames_target = MAX_FRAMES_IN_CHUNK;
            break;
        default: return false;
    }

    switch(accel_range) {
        case 2: config[0].cfg.acc.range = BMI2_ACC_RANGE_2G; break;
        case 4: config[0].cfg.acc.range = BMI2_ACC_RANGE_4G; break;
        case 8: config[0].cfg.acc.range = BMI2_ACC_RANGE_8G; break;
        case 16: config[0].cfg.acc.range = BMI2_ACC_RANGE_16G; break;
        default: return false;
    }

    switch(gyro_range) {
        case 125: config[1].cfg.gyr.range = BMI2_GYR_RANGE_125; break;
        case 250: config[1].cfg.gyr.range = BMI2_GYR_RANGE_250; break;
        case 500: config[1].cfg.gyr.range = BMI2_GYR_RANGE_500; break;
        case 1000: config[1].cfg.gyr.range = BMI2_GYR_RANGE_1000; break;
        case 2000: config[1].cfg.gyr.range = BMI2_GYR_RANGE_2000; break;
        default: return false;
    }

    config[0].type = BMI2_ACCEL;
    config[1].type = BMI2_GYRO;

    /* The bandwidth parameter is used to configure the number of sensor samples that are averaged
     * if it is set to 2, then 2^(bandwidth parameter) samples
     * are averaged, resulting in 4 averaged samples
     * Note1 : For more information, refer the datasheet.
     * Note2 : A higher number of averaged samples will result in a lower noise level of the signal, but
     * this has an adverse effect on the power consumed.
     */
    config[0].cfg.acc.bwp = BMI2_ACC_OSR4_AVG1;

    /* Enable the filter performance mode where averaging of samples
     * will be done based on above set bandwidth and ODR.
     * There are two modes
     *  0 -> Ultra low power mode
     *  1 -> High performance mode(Default)
     * For more info refer datasheet.
     */
    config[0].cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;

    /* Gyroscope Bandwidth parameters. By default the gyro bandwidth is in normal mode. */
    config[1].cfg.gyr.bwp = BMI2_GYR_NORMAL_MODE;

    /* Enable/Disable the noise performance mode for precision yaw rate sensing
     * There are two modes
     *  0 -> Ultra low power mode(Default)
     *  1 -> High performance mode
     */
    config[1].cfg.gyr.noise_perf = BMI2_POWER_OPT_MODE;

    /* Enable/Disable the filter performance mode where averaging of samples
     * will be done based on above set bandwidth and ODR.
     * There are two modes
     *  0 -> Ultra low power mode
     *  1 -> High performance mode(Default)
     */
    config[1].cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;

    /* Configure the sensors */
    result = bmi2_set_sensor_config(config, SENSOR_COUNT, &(dev->sensor));
    if (result != BMI2_OK)
    {
        return result;
    }

    /* Enable the sensors */
    uint8_t sens_list[SENSOR_COUNT] = { BMI2_ACCEL, BMI2_GYRO };
    switch(mode) {
    case BMI270_MODE_STREAM_COMBINED:
    case BMI270_MODE_STREAM_SPLIT:
        result = bmi2_sensor_enable(sens_list, 2, &(dev->sensor));
        break;
    case BMI270_MODE_STREAM_ONLY_ACCEL:
        result = bmi2_sensor_enable(sens_list, 1, &(dev->sensor));
        break;
    case BMI270_MODE_STREAM_ONLY_GYRO:
        result = bmi2_sensor_enable(sens_list + 1, 1, &(dev->sensor));
        break;
    }

    if (result != BMI2_OK)
    {
        return result;
    }

    /* It takes a while for the sensor to deliver data.
     * Wait for 5 successful reads. Give up after 2 seconds.
     */
    int success = 0;
    for(int i = 0; i < 20; i++)
    {
        if(_read_hw(dev))
            success++;        /* We got data! */

        if(success >= 5) {
            dev->frames_sampled = 0;

               printf("bmi270: Configured device. mode=%i, rate=%d Hz, frames/chunk=%d, gyro=%ddps, accel=%dG\r\n",
                        dev->stream_mode, rate, dev->frames_target, gyro_range, accel_range);
            return true;    /* OK, all good. */
        }

        cyhal_system_delay_ms(100);
    }

   return false;
}

/******************************************************************************
* Function Name: _read_hw
********************************************************************************
* Summary:
*   Reads the current bmi270 data and convert it.
*
* Parameters:
*   dev: Pointer to the bmi270 device handle.
*
* Return:
*   True if data retrieval is successful, otherwise false.
*
*******************************************************************************/
static bool _read_hw(dev_bmi270_t* dev)
{
    int8_t result;
    struct bmi2_sens_data data = { { 0 } };
    struct bmi2_dev *sensor = &dev->sensor;

    result = bmi2_get_sensor_data(&data, sensor);

    if(result != BMI2_OK)
    {
        return false;
    }

    if(dev->stream_mode == BMI270_MODE_STREAM_COMBINED) {
        if(!(data.status & BMI2_DRDY_ACC) || !(data.status & BMI2_DRDY_GYR))
            return false;

        float *dest = dev->data_compined + dev->frames_sampled * AXIS_COUNT * SENSOR_COUNT;
        *dest++ = _lsb_to_mps2(data.acc.x, dev->accel_range, sensor->resolution);
        *dest++ = _lsb_to_mps2(data.acc.y, dev->accel_range, sensor->resolution);
        *dest++ = _lsb_to_mps2(data.acc.z, dev->accel_range, sensor->resolution);
        *dest++ = _lsb_to_dps(data.gyr.x, dev->gyro_range, sensor->resolution);
        *dest++ = _lsb_to_dps(data.gyr.y, dev->gyro_range, sensor->resolution);
        *dest++ = _lsb_to_dps(data.gyr.z, dev->gyro_range, sensor->resolution);
    }

    if(dev->stream_mode == BMI270_MODE_STREAM_SPLIT || dev->stream_mode == BMI270_MODE_STREAM_ONLY_ACCEL) {
        if(!(data.status & BMI2_DRDY_ACC))
            return false;

        float *dest = dev->accel_data + dev->frames_sampled * AXIS_COUNT;

        *dest++ = _lsb_to_mps2(data.acc.x, dev->accel_range, sensor->resolution);
        *dest++ = _lsb_to_mps2(data.acc.y, dev->accel_range, sensor->resolution);
        *dest++ = _lsb_to_mps2(data.acc.z, dev->accel_range, sensor->resolution);
    }

    if(dev->stream_mode == BMI270_MODE_STREAM_SPLIT || dev->stream_mode == BMI270_MODE_STREAM_ONLY_GYRO) {
        if(!(data.status & BMI2_DRDY_GYR))
            return false;

        float *dest = dev->gyro_data + dev->frames_sampled * AXIS_COUNT;

        *dest++ = _lsb_to_dps(data.gyr.x, dev->gyro_range, sensor->resolution);
        *dest++ = _lsb_to_dps(data.gyr.y, dev->gyro_range, sensor->resolution);
        *dest++ = _lsb_to_dps(data.gyr.z, dev->gyro_range, sensor->resolution);
    }

    dev->frames_sampled++;

    return true;
}

/******************************************************************************
* Function Name: _configure_streams
********************************************************************************
* Summary:
*   Configures the data streams based on the user current settings.
*
* Parameters:
*   protocol: Pointer to the protocol handle.
*   device: The device index.
*   dev: Pointer to the bmi270 device handle.
*
* Return:
*   True to keep the connection open, otherwise false.
*
*******************************************************************************/
static bool _configure_streams(protocol_t* protocol, int device, void* arg)
{
    int result;
    int rate_index;
    int rate;
    int mode_index;
    stream_mode_t mode;
    const char *stream0_unit;
    const char *stream1_unit;
    const char *stream0_name;
    const char *stream1_name;
    int stream0;
    int stream1;

    if(protocol_get_option_oneof(protocol, device, DEV_BMI270_OPTION_KEY_RATE, &rate_index) != PROTOCOL_STATUS_SUCCESS) {
        protocol_set_device_status(
            protocol,
            device,
            protocol_DeviceStatus_DEVICE_STATUS_ERROR,
            "Failed to get option frequency.");
        return true;
    }

    switch(rate_index) {
       case 0: rate = 50; break;
       case 1: rate = 100; break;
       case 2: rate = 200; break;
       case 3: rate = 400; break;
       default: return false;
    }

    if(protocol_get_option_oneof(protocol, device, DEV_BMI270_OPTION_KEY_STREAM_MODE, &mode_index) != PROTOCOL_STATUS_SUCCESS) {
        protocol_set_device_status(
            protocol,
            device,
            protocol_DeviceStatus_DEVICE_STATUS_ERROR,
            "Failed to get option stream mode.");
        return true;
    }

    /* Clear any existing streams */
    if(protocol_clear_streams(protocol, device) != PROTOCOL_STATUS_SUCCESS) {
        protocol_set_device_status(
            protocol,
            device,
            protocol_DeviceStatus_DEVICE_STATUS_ERROR,
            "Failed to clear streams.");
        return true;
    }

    switch(mode_index) {
       case 0:
           mode = BMI270_MODE_STREAM_COMBINED;
           stream0_unit = "m/s\xc2\xb2, \xc2\xb0/s"; /* Meter per second squared, Degrees per second */
           stream1_unit = NULL;
           stream0_name = "Combined";
           stream1_name = NULL;
           break;
       case 1:
           mode = BMI270_MODE_STREAM_SPLIT;
           stream0_unit = "m/s\xc2\xb2";         /* Meter per second squared */
           stream1_unit = "\xc2\xb0/s";         /* Degrees per second */
           stream0_name = "Accel";
           stream1_name = "Gyro";
           break;
       case 2:
           mode = BMI270_MODE_STREAM_ONLY_ACCEL;
           stream0_unit = "m/s\xc2\xb2";         /* Meter per second squared */
           stream1_unit = NULL;
           stream0_name = "Accel";
           stream1_name = NULL;
           break;
       case 3:
           mode = BMI270_MODE_STREAM_ONLY_GYRO;
           stream0_unit = "\xc2\xb0/s";         /* Degrees per second */
           stream1_unit = NULL;
           stream0_name = "Gyro";
           stream1_name = NULL;
           break;
       default: return false;
    }

    /* Add stream #0 */
    stream0 = protocol_add_stream(
           protocol,
           device,
           stream0_name,
           protocol_StreamDirection_STREAM_DIRECTION_OUTPUT,
           protocol_DataType_DATA_TYPE_F32,
           rate,
           1,
           stream0_unit);
    if(stream0 < 0) {
        protocol_set_device_status(
            protocol,
            device,
            protocol_DeviceStatus_DEVICE_STATUS_ERROR,
            "Failed to add streams.");
        return true;
    }

    if(mode == BMI270_MODE_STREAM_COMBINED) {
        result = protocol_add_stream_rank(
            protocol,
            device,
            stream0,
            "Sensor",
            2,
            (const char* []) { "Accel", "Gyro" });
        if(result != PROTOCOL_STATUS_SUCCESS) {
            protocol_set_device_status(
                protocol,
                device,
                protocol_DeviceStatus_DEVICE_STATUS_ERROR,
                "Failed to add streams dimension.");
            return true;
        }
    }

    /* Add optional stream #1 */
    if(stream1_name != NULL) {
        stream1 = protocol_add_stream(
           protocol,
           device,
           stream1_name,
           protocol_StreamDirection_STREAM_DIRECTION_OUTPUT,
           protocol_DataType_DATA_TYPE_F32,
           rate,
           1,
           stream1_unit);
        if(stream1 < 0) {
            protocol_set_device_status(
                protocol,
                device,
                protocol_DeviceStatus_DEVICE_STATUS_ERROR,
                "Failed to add streams.");
            return true;
        }

        result = protocol_add_stream_rank(
            protocol,
            device,
            stream1,
            "Axis",
            3,
            (const char* []) { "X", "Y", "Z" });
        if(result != PROTOCOL_STATUS_SUCCESS) {
            protocol_set_device_status(
                protocol,
                device,
                protocol_DeviceStatus_DEVICE_STATUS_ERROR,
                "Failed to add streams dimension.");
            return true;
        }
    }

    result = protocol_add_stream_rank(
        protocol,
            device,
            stream0,
            "Axis",
            3,
            (const char* []) { "X", "Y", "Z" });
    if(result != PROTOCOL_STATUS_SUCCESS) {
        protocol_set_device_status(
            protocol,
            device,
            protocol_DeviceStatus_DEVICE_STATUS_ERROR,
            "Failed to add streams dimension.");
        return true;
    }

    protocol_set_device_status(
        protocol,
        device,
        protocol_DeviceStatus_DEVICE_STATUS_READY,
        "Device is ready.");

    return true;
}

/******************************************************************************
* Function Name: _start_streams
********************************************************************************
* Summary:
*   Starts the data streaming process.
*
* Parameters:
*   protocol: Pointer to the protocol handle.
*   device: The device index.
*   ostream: Pointer to the output stream to write to
*   arg: Pointer to the bmi270 device handle.
*
*******************************************************************************/
static void _start_streams(protocol_t* protocol, int device, pb_ostream_t* ostream, void* arg)
{
    int rate_index;
    int rate;
    int accel_range_index;
    int gyro_range_index;
    int accel_range;
    int gyro_range;
    int mode_index;
    stream_mode_t mode;
    dev_bmi270_t* dev = (dev_bmi270_t*)arg;
    UNUSED(ostream);

    protocol_get_option_oneof(protocol, device, DEV_BMI270_OPTION_KEY_RATE, &rate_index);
    switch(rate_index) {
       case 0: rate = 50; break;
       case 1: rate = 100; break;
       case 2: rate = 200; break;
       case 3: rate = 400; break;
       default: return;
    }

    protocol_get_option_oneof(protocol, device, DEV_BMI270_OPTION_KEY_ACCEL_RANGE, &accel_range_index);
    switch(accel_range_index) {
       case 0: accel_range = 2; break;
       case 1: accel_range = 4; break;
       case 2: accel_range = 8; break;
       case 3: accel_range = 16; break;
       default: return;
    }

    protocol_get_option_oneof(protocol, device, DEV_BMI270_OPTION_KEY_GYRO_RANGE, &gyro_range_index);
    switch(gyro_range_index) {
       case 0: gyro_range = 125; break;
       case 1: gyro_range = 250; break;
       case 2: gyro_range = 500; break;
       case 3: gyro_range = 1000; break;
       case 4: gyro_range = 2000; break;
       default: return;
    }

    protocol_get_option_oneof(protocol, device, DEV_BMI270_OPTION_KEY_STREAM_MODE, &mode_index);
    switch(mode_index) {
       case 0: mode = BMI270_MODE_STREAM_COMBINED; break;
       case 1: mode = BMI270_MODE_STREAM_SPLIT; break;
       case 2: mode = BMI270_MODE_STREAM_ONLY_ACCEL; break;
       case 3:mode = BMI270_MODE_STREAM_ONLY_GYRO; break;
       default: return;
    }

    if(!_config_hw(dev, rate, accel_range, gyro_range, mode)) {
        protocol_set_device_status(
            protocol,
            device,
            protocol_DeviceStatus_DEVICE_STATUS_ERROR,
            "Failed to configure hardware.");
    } else {
        protocol_set_device_status(
            protocol,
            device,
            protocol_DeviceStatus_DEVICE_STATUS_ACTIVE,
            "Device is streaming.");
    }

    /* During the startup phase we shouldn't test for beeing late. */
    /* This is due to the fact that the buffers may contain bad data and  */
    /* Should be read regardless. */
    dev->first_sample = true;
    dev->sample_time_tick = clock_get_tick();

}

/******************************************************************************
* Function Name: _stop_streams
********************************************************************************
* Summary:
*   Stops the streaming process.
*
* Parameters:
*   protocol: Pointer to the protocol handle.
*   device: The device index.
*   ostream: Pointer to the output stream to write to
*   arg: Pointer to the bmi270 device handle.
*
*******************************************************************************/
static void _stop_streams(protocol_t* protocol, int device, pb_ostream_t* ostream, void* arg)
{
    dev_bmi270_t* dev = (dev_bmi270_t*)arg;
    UNUSED(ostream);

    uint8_t sens_list[2] = { BMI2_ACCEL, BMI2_GYRO };
    bmi2_sensor_disable(sens_list, 2, &(dev->sensor));

    protocol_set_device_status(
        protocol,
        device,
        protocol_DeviceStatus_DEVICE_STATUS_READY,
        "Device stopped");

}

/******************************************************************************
* Function Name: _write_payload
********************************************************************************
* Summary:
*   Writes the data into the output stream.
*
* Parameters:
*   protocol: Pointer to the protocol handle.
*   device_id: The device index.
*   stream_id: The stream index.
*   frame_count: Number of frames to write.
*   total_bytes: Total number of bytes to write (frame_count * sizeof(type) * frame_shape.flat).
*   ostream: Pointer to the output stream to write to.
*   arg: Pointer to the bmi270 device handle.
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
    UNUSED(frame_count);
    UNUSED(protocol);

    dev_bmi270_t* dev = (dev_bmi270_t*)arg;
    float *frame;

    switch(dev->stream_mode)
    {
    case BMI270_MODE_STREAM_COMBINED:
        return pb_write(ostream, (const pb_byte_t *)dev->data_compined, total_bytes);
    case BMI270_MODE_STREAM_SPLIT:
        if(stream_id == 0)
        {
            frame = dev->accel_data;
        }
        else
        {
            frame = dev->gyro_data;
        }
        return pb_write(ostream, (const pb_byte_t *)frame, total_bytes);
    case BMI270_MODE_STREAM_ONLY_ACCEL:
        return pb_write(ostream, (const pb_byte_t *)dev->accel_data, total_bytes);
    case BMI270_MODE_STREAM_ONLY_GYRO:
        return pb_write(ostream, (const pb_byte_t *)dev->gyro_data, total_bytes);
    default:
        return false;
    }
}



/******************************************************************************
* Function Name: _poll_streams
********************************************************************************
* Summary:
*   Periodically polls for data and sends data-chunk messages.
*
* Parameters:
*   protocol: Pointer to the protocol handle.
*   device: The device index.
*   ostream: Pointer to the output stream to write to.
*   dev: Pointer to the bmi270 device handle.
*
*******************************************************************************/
static void _poll_streams(protocol_t* protocol, int device, pb_ostream_t* ostream, void* arg)
{
    dev_bmi270_t* dev = (dev_bmi270_t*)arg;
    clock_tick_t current_time = clock_get_tick();



    /* Reinterpret this timing as the time we wish the sample to happen at. */
    clock_tick_t current_treshold = dev->sample_time_tick;


    clock_tick_t total_drift = current_time - current_treshold;

/* If we are to late we skip this frame and save time
* Previous data package will be resent */
    bool late = false;
    uint32_t drift_ms = (uint32_t)total_drift;
    if (drift_ms > 200){ /* Tens of microseconds. 200 is equal to 2 ms.*/
        late=1;
    }

    /* The first sampling is now done as soon as possible. */
    if(current_time >= current_treshold ) {

        /* If we are late.. Skip the reading of the sensor.. */
        if ( late && !dev->first_sample){
            late = false;
        }
        else{
            if(!_read_hw(dev)) {
                return;
            }
            dev->first_sample = false;
        }

        /* This is updated whenever there is an _read_hw call! However since the _read_hw might fail */
        /* and when we are resending the previous data we have to force this to 1 anyway. */
        dev->frames_sampled = 1;

        /* Since we in reality dont drop any frames anymore. */
        dev->frames_dropped = 0;

        /* When we should do the next frame. */
        dev->sample_time_tick += dev->period_tick;

        /* Always send something. */
        {
            protocol_send_data_chunk(protocol, device, 0, dev->frames_sampled, dev->frames_dropped, ostream, _write_payload);

            if(dev->stream_mode == BMI270_MODE_STREAM_SPLIT)
                protocol_send_data_chunk(protocol, device, 1, dev->frames_sampled, dev->frames_dropped, ostream, _write_payload);

            dev->frames_dropped = 0;
            dev->frames_sampled = 0;

        }
    }
}

/******************************************************************************
* Function Name: dev_bmi270_register
********************************************************************************
* Summary:
*   Registers the bmi270 device with the protocol, configuring it for use.
*
* Parameters:
*   protocol: Pointer to the protocol handle.
*   i2c: Pointer to the I2C interface resource.
*
* Return:
*   True if registration is successful, otherwise false.
*
*******************************************************************************/
bool dev_bmi270_register(protocol_t* protocol, cyhal_i2c_t* i2c)
{
    int status;

    dev_bmi270_t *dev = (dev_bmi270_t*)malloc(sizeof(dev_bmi270_t));
    if(dev == NULL)
    {
        return false;
    }
    memset(dev, 0, sizeof(dev_bmi270_t));


    if(!_init_hw(dev, i2c))
    {
        free(dev);
        return false;
    }

    device_manager_t manager = {
        .arg = dev,
        .configure_streams = _configure_streams,
        .start = _start_streams,
        .stop = _stop_streams,
        .poll = _poll_streams,
        .data_received = NULL /* has no input streams */
    };

    int device = protocol_add_device(
        protocol,
        protocol_DeviceType_DEVICE_TYPE_SENSOR,
        "IMU",
        "Accelerometer and Gyroscope (BMI270)",
        manager);

    if(device < 0)
    {
        return false;
    }

    status = protocol_add_option_oneof(
        protocol,
        device,
        DEV_BMI270_OPTION_KEY_RATE,
        "Frequency",
        "Sample frequency (Hz)",
        0,
        (const char* []) { "50 Hz", "100 Hz", "200 Hz", "400 Hz" },
        4);

    if(status != PROTOCOL_STATUS_SUCCESS)
    {
        return false;
    }

    status = protocol_add_option_oneof(
        protocol,
        device,
        DEV_BMI270_OPTION_KEY_ACCEL_RANGE,
        "Accel Range",
        "Min/Max gravity range in G",
        2,
        (const char* []) { "2 G", "4 G", "8 G", "16 G" },
        4);

    if(status != PROTOCOL_STATUS_SUCCESS)
    {
        return false;
    }

    status = protocol_add_option_oneof(
        protocol,
        device,
        DEV_BMI270_OPTION_KEY_GYRO_RANGE,
        "Gyro Range",
        "Angular Rate Measurement Range",
        2,
        (const char* []) { "125 dps", "250 dps", "500 dps", "1000 dps", "2000 dps" },
        5);

    if(status != PROTOCOL_STATUS_SUCCESS)
    {
        return false;
    }

     status = protocol_add_option_oneof(
        protocol,
        device,
        DEV_BMI270_OPTION_KEY_STREAM_MODE,
        "Mode",
        "Stream Configuration",
        0,
        (const char* []) { "Combined", "Split", "Only Accel", "Only Gyro" },
        4);

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

#endif /* IM_ENABLE_BMI270 */

/* [] END OF FILE */
