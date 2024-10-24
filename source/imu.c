/******************************************************************************
* File Name:   imu.c
*
* Description: This file implements the interface with the motion sensor, as
*              a timer to feed the pre-processor at 50Hz.
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

#include "imu.h"
#include "cyhal.h"
#include "cybsp.h"
#ifdef IM_BMI_270_IMU_I2C
#include "mtb_bmi270.h"
#elif IM_BMX_160_IMU_SPI
#include "mtb_bmx160.h"
#else
#include "mtb_bmi160.h"
#endif
#include "config.h"


/*******************************************************************************
* Macros
*******************************************************************************/

#define IMU_SCAN_RATE         50
#define IMU_TIMER_FREQUENCY   100000
#define IMU_TIMER_PERIOD      (IMU_TIMER_FREQUENCY/IMU_SCAN_RATE)
#define IMU_TIMER_PRIORITY    3

#ifdef IM_XSS_BMI270
#define BMI270_ADDRESS (MTB_BMI270_ADDRESS_SEC)
#else
#define BMI270_ADDRESS (MTB_BMI270_ADDRESS_DEFAULT)
#endif

/*******************************************************************************
* Global Variables
*******************************************************************************/
#ifdef IM_BMX_160_IMU_SPI
    /* BMX160 driver structures */
    mtb_bmx160_data_t data;
    mtb_bmx160_t sensor_bmx160;
#endif

#ifdef IM_BMI_160_IMU_SPI
    /* BMI160 driver structures */
    mtb_bmi160_data_t data;
    mtb_bmi160_t sensor_bmi160;
#endif

#ifdef IM_BMI_160_IMU_I2C
    /* BMI160 driver structures */
    mtb_bmi160_data_t data;
    mtb_bmi160_t sensor_bmi160;
#endif

#ifdef IM_BMI_270_IMU_I2C
    /* BMI270 driver structures */
    mtb_bmi270_data_t data;
    mtb_bmi270_t sensor_bmi270;
#endif

/* timer used for getting data */
cyhal_timer_t imu_timer;

/*******************************************************************************
* Function Prototypes
*******************************************************************************/
void imu_interrupt_handler(void* callback_arg, cyhal_timer_event_t event);
cy_rslt_t imu_timer_init(void);


/*******************************************************************************
* Function Name: imu_init
********************************************************************************
* Summary:
*    A function used to initialize the IMU based on the shield selected in the
*    makefile. Starts a timer that triggers an interrupt at 50Hz.
*
* Parameters:
*   None
*
* Return:
*     The status of the initialization.
*
*
*******************************************************************************/
cy_rslt_t imu_init(void)
{
    cy_rslt_t result;

#ifdef IM_BMX_160_IMU_SPI
    /* Initialize the IMU */
    result = mtb_bmx160_init_spi(&sensor_bmx160, &spi, CYBSP_SPI_CS);
    if(CY_RSLT_SUCCESS != result)
    {
        return result;
    }

    /* Set the output data rate and range of the accelerometer */
    sensor_bmx160.sensor1.accel_cfg.odr = IMU_SAMPLE_RATE;
    sensor_bmx160.sensor1.accel_cfg.range = IMU_SAMPLE_RANGE;

    /* Set the sensor configuration */
    bmi160_set_sens_conf(&(sensor_bmx160.sensor1));
#endif

#ifdef IM_BMI_160_IMU_SPI
    /* Initialize the IMU */
    result = mtb_bmi160_init_spi(&sensor_bmi160, &spi, CYBSP_SPI_CS);
    if(CY_RSLT_SUCCESS != result)
    {
        return result;
    }

    /* Set the output data rate and range of the accelerometer */
    sensor_bmi160.sensor.accel_cfg.odr = IMU_SAMPLE_RATE;
    sensor_bmi160.sensor.accel_cfg.range = IMU_SAMPLE_RANGE;

    /* Set the sensor configuration */
    bmi160_set_sens_conf(&(sensor_bmi160.sensor));
#endif

#ifdef IM_BMI_160_IMU_I2C
    /* Initialize the IMU */
    result = mtb_bmi160_init_i2c(&sensor_bmi160, &i2c, MTB_BMI160_DEFAULT_ADDRESS);
    if(CY_RSLT_SUCCESS != result)
    {
        return result;
    }

    /* Set the default configuration for the BMI160 */
    result = mtb_bmi160_config_default(&sensor_bmi160);
    if(CY_RSLT_SUCCESS != result)
    {
        return result;
    }

    /* Set the output data rate and range of the accelerometer */
    sensor_bmi160.sensor.accel_cfg.odr = IMU_SAMPLE_RATE;
    sensor_bmi160.sensor.accel_cfg.range = IMU_SAMPLE_RANGE;

    /* Set the sensor configuration */
    bmi160_set_sens_conf(&(sensor_bmi160.sensor));
#endif

#ifdef IM_BMI_270_IMU_I2C
    struct bmi2_sens_config config = {0};

    /* Initialize the IMU */
    result = mtb_bmi270_init_i2c(&sensor_bmi270, &i2c, BMI270_ADDRESS);
    if(CY_RSLT_SUCCESS != result)
    {
        return result;
    }

    /* Set the default configuration for the BMI160 */
    result = mtb_bmi270_config_default(&sensor_bmi270);
    if(CY_RSLT_SUCCESS != result)
    {
        return result;
    }

    /* Set the output data rate and range of the accelerometer */
    config.type = BMI2_ACCEL;
    config.cfg.acc.odr = BMI2_ACC_ODR_50HZ;
    config.cfg.acc.range = BMI2_ACC_RANGE_8G;
    result = bmi2_set_sensor_config(&config, 1, &(sensor_bmi270.sensor));
#endif

    imu_flag = false;

    /* Timer for data collection */
    result = imu_timer_init();
    if(CY_RSLT_SUCCESS != result)
    {
        return result;
    }

    return CY_RSLT_SUCCESS;
}


/*******************************************************************************
* Function Name: imu_timer_init
********************************************************************************
* Summary:
*   Sets up an interrupt that triggers at the desired frequency.
*
* Returns:
*   The status of the initialization.
*
*
*******************************************************************************/
cy_rslt_t imu_timer_init(void)
{
    cy_rslt_t rslt;
    const cyhal_timer_cfg_t timer_cfg =
    {
        .compare_value = 0,                 /* Timer compare value, not used */
        .period = IMU_TIMER_PERIOD,         /* Defines the timer period */
        .direction = CYHAL_TIMER_DIR_UP,    /* Timer counts up */
        .is_compare = false,                /* Don't use compare mode */
        .is_continuous = true,              /* Run the timer indefinitely */
        .value = 0                          /* Initial value of counter */
    };

    /* Initialize the timer object. Does not use pin output ('pin' is NC) and
     * does not use a pre-configured clock source ('clk' is NULL). */
    rslt = cyhal_timer_init(&imu_timer, NC, NULL);
    if (CY_RSLT_SUCCESS != rslt)
    {
        return rslt;
    }

    /* Apply timer configuration such as period, count direction, run mode, etc. */
    rslt = cyhal_timer_configure(&imu_timer, &timer_cfg);
    if (CY_RSLT_SUCCESS != rslt)
    {
        return rslt;
    }

    /* Set the frequency of timer to 100KHz */
    rslt = cyhal_timer_set_frequency(&imu_timer, IMU_TIMER_FREQUENCY);
    if (CY_RSLT_SUCCESS != rslt)
    {
        return rslt;
    }

    /* Assign the ISR to execute on timer interrupt */
    cyhal_timer_register_callback(&imu_timer, imu_interrupt_handler, NULL);
    /* Set the event on which timer interrupt occurs and enable it */
    cyhal_timer_enable_event(&imu_timer, CYHAL_TIMER_IRQ_TERMINAL_COUNT, IMU_TIMER_PRIORITY, true);
    /* Start the timer with the configured settings */
    rslt = cyhal_timer_start(&imu_timer);
    if (CY_RSLT_SUCCESS != rslt)
    {
        return rslt;
    }

    return CY_RSLT_SUCCESS;
}


/*******************************************************************************
* Function Name: imu_interrupt_handler
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
void imu_interrupt_handler(void *callback_arg, cyhal_timer_event_t event)
{
    (void) callback_arg;
    (void) event;

    imu_flag = true;
}


/*******************************************************************************
* Function Name: imu_get_data
********************************************************************************
* Summary:
*   Reads accelerometer data from the IMU and stores it in a buffer.
*
* Parameters:
*     imu_data: Stores IMU accelerometer data
*
*
*******************************************************************************/
void imu_get_data(float *imu_data)
{
    /* Read data from IMU sensor */
    cy_rslt_t result;
#ifdef IM_BMX_160_IMU_SPI
    result = mtb_bmx160_read(&sensor_bmx160, &data);
#endif
#if defined(IM_BMI_160_IMU_SPI) || (IM_BMI_160_IMU_I2C)
    result = mtb_bmi160_read(&sensor_bmi160, &data);
#endif
#ifdef IM_BMI_270_IMU_I2C
    result = mtb_bmi270_read(&sensor_bmi270, &data);
#endif
    if (CY_RSLT_SUCCESS == result)
    {
        #if defined(IM_BMI_160_IMU_SPI) || (IM_BMX_160_IMU_SPI)
            imu_data[0] = ((float)data.accel.y) / (float)0x1000;
            imu_data[1] = ((float)data.accel.x) / (float)0x1000;
            imu_data[2] = ((float)data.accel.z) / (float)0x1000;
        #endif
        #if defined(IM_IMU_BMI270) || (IM_XSS_BMI270)
            imu_data[0] = ((float)data.sensor_data.acc.x) / (float)0x1000;
            imu_data[1] = ((float)data.sensor_data.acc.y) / (float)0x1000;
            imu_data[2] = ((float)data.sensor_data.acc.z) / (float)0x1000;
        #endif
        #ifdef IM_BMI_160_IMU_I2C
            imu_data[0] = ((float)data.accel.x) / (float)0x1000;
            imu_data[1] = ((float)data.accel.y) / (float)0x1000;
            imu_data[2] = ((float)data.accel.z) / (float)0x1000;
        #endif
    }
}
