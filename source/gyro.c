/******************************************************************************
* File Name:   gyro.c
*
* Description: This file implements the interface with the motion sensor to, as
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

#include "gyro.h"
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

#define GYRO_SCAN_RATE         50
#define GYRO_TIMER_FREQUENCY   100000
#define GYRO_TIMER_PERIOD      (GYRO_TIMER_FREQUENCY/GYRO_SCAN_RATE)
#define GYRO_TIMER_PRIORITY    5

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
    mtb_bmx160_data_t data_gyro;
    mtb_bmx160_t sensor_gyro;
#endif

#ifdef IM_BMI_160_IMU_SPI
    /* BMI160 driver structures */
    mtb_bmi160_data_t data_gyro;
    mtb_bmi160_t sensor_gyro;
#endif

#ifdef IM_BMI_160_IMU_I2C
    /* BMI160 driver structures */
    mtb_bmi160_data_t data_gyro;
    mtb_bmi160_t sensor_gyro;
#endif

#ifdef IM_BMI_270_IMU_I2C
    /* BMI270 driver structures */
    mtb_bmi270_data_t data_gyro;
    mtb_bmi270_t sensor_gyro;
#endif

/* timer used for getting data */
cyhal_timer_t gyro_timer;

/*******************************************************************************
* Function Prototypes
*******************************************************************************/
void gyro_interrupt_handler(void* callback_arg, cyhal_timer_event_t event);
cy_rslt_t gyro_timer_init(void);


/*******************************************************************************
* Function Name: gyro_init
********************************************************************************
* Summary:
*    A function used to initialize the gyroscope based on the shield selected in the
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
cy_rslt_t gyro_init(void)
{
    cy_rslt_t result;

#ifdef IM_BMX_160_IMU_SPI
    /* Initialize the IMU */
    result = mtb_bmx160_init_spi(&sensor_gyro, &spi, CYBSP_SPI_CS);
    if(CY_RSLT_SUCCESS != result)
    {
        return result;
    }

    /* Set the output data rate and range of the gyroscope */
    sensor_gyro.sensor1.gyro_cfg.odr = IMU_SAMPLE_RATE;
    sensor_gyro.sensor1.gyro_cfg.range = IMU_SAMPLE_RANGE;

    /* Set the sensor configuration */
    bmi160_set_sens_conf(&(sensor_gyro.sensor1));
#endif

#ifdef IM_BMI_160_IMU_SPI
    /* Initialize the IMU */
    result = mtb_bmi160_init_spi(&sensor_gyro, &spi, CYBSP_SPI_CS);
    if(CY_RSLT_SUCCESS != result)
    {
        return result;
    }

    /* Set the output data rate and range of the gyroscope */
    sensor_gyro.sensor.gyro_cfg.odr = IMU_SAMPLE_RATE;
    sensor_gyro.sensor.gyro_cfg.range = IMU_SAMPLE_RANGE;

    /* Set the sensor configuration */
    bmi160_set_sens_conf(&(sensor_gyro.sensor));
#endif

#ifdef IM_BMI_160_IMU_I2C
    /* Initialize the IMU */
    result = mtb_bmi160_init_i2c(&sensor_gyro, &i2c, MTB_BMI160_DEFAULT_ADDRESS);
    if(CY_RSLT_SUCCESS != result)
    {
        return result;
    }

    /* Set the default configuration for the BMI160 */
    result = mtb_bmi160_config_default(&sensor_gyro);
    if(CY_RSLT_SUCCESS != result)
    {
        return result;
    }

    /* Set the output data rate and range of the gyroscope */
    sensor_gyro.sensor.gyro_cfg.odr = IMU_SAMPLE_RATE;
    sensor_gyro.sensor.gyro_cfg.range = IMU_SAMPLE_RANGE;

    /* Set the sensor configuration */
    bmi160_set_sens_conf(&(sensor_gyro.sensor));
#endif

#ifdef IM_BMI_270_IMU_I2C
    struct bmi2_sens_config config = {0};

    /* Initialize the gyro */
    result = mtb_bmi270_init_i2c(&sensor_gyro, &i2c, BMI270_ADDRESS);
    if(CY_RSLT_SUCCESS != result)
    {
        return result;
    }

    /* Set the default configuration for the BMI270 */
    result = mtb_bmi270_config_default(&sensor_gyro);
    if(CY_RSLT_SUCCESS != result)
    {
        return result;
    }

    /* Set the output data rate and range of the gyroscope */
    config.type = BMI2_GYRO;
    config.cfg.gyr.odr = BMI2_GYR_ODR_50HZ;
    config.cfg.gyr.range = BMI2_GYR_RANGE_500;
    result = bmi2_set_sensor_config(&config, 1, &(sensor_gyro.sensor));
#endif

    gyro_flag = false;

    /* Timer for data collection */
    result = gyro_timer_init();
    if(CY_RSLT_SUCCESS != result)
    {
        return result;
    }

    return CY_RSLT_SUCCESS;
}


/*******************************************************************************
* Function Name: gyro_timer_init
********************************************************************************
* Summary:
*   Sets up an interrupt that triggers at the desired frequency.
*
* Returns:
*   The status of the initialization.
*
*
*******************************************************************************/
cy_rslt_t gyro_timer_init(void)
{
    cy_rslt_t rslt;
    const cyhal_timer_cfg_t timer_cfg =
    {
        .compare_value = 0,                 /* Timer compare value, not used */
        .period = GYRO_TIMER_PERIOD,        /* Defines the timer period */
        .direction = CYHAL_TIMER_DIR_UP,    /* Timer counts up */
        .is_compare = false,                /* Don't use compare mode */
        .is_continuous = true,              /* Run the timer indefinitely */
        .value = 0                          /* Initial value of counter */
    };

    /* Initialize the timer object. Does not use pin output ('pin' is NC) and
     * does not use a pre-configured clock source ('clk' is NULL). */
    rslt = cyhal_timer_init(&gyro_timer, NC, NULL);
    if (CY_RSLT_SUCCESS != rslt)
    {
        return rslt;
    }

    /* Apply timer configuration such as period, count direction, run mode, etc. */
    rslt = cyhal_timer_configure(&gyro_timer, &timer_cfg);
    if (CY_RSLT_SUCCESS != rslt)
    {
        return rslt;
    }

    /* Set the frequency of timer to 100KHz */
    rslt = cyhal_timer_set_frequency(&gyro_timer, GYRO_TIMER_FREQUENCY);
    if (CY_RSLT_SUCCESS != rslt)
    {
        return rslt;
    }

    /* Assign the ISR to execute on timer interrupt */
    cyhal_timer_register_callback(&gyro_timer, gyro_interrupt_handler, NULL);
    /* Set the event on which timer interrupt occurs and enable it */
    cyhal_timer_enable_event(&gyro_timer, CYHAL_TIMER_IRQ_TERMINAL_COUNT, GYRO_TIMER_PRIORITY, true);
    /* Start the timer with the configured settings */
    rslt = cyhal_timer_start(&gyro_timer);
    if (CY_RSLT_SUCCESS != rslt)
    {
        return rslt;
    }

    return CY_RSLT_SUCCESS;
}


/*******************************************************************************
* Function Name: gyro_interrupt_handler
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
void gyro_interrupt_handler(void *callback_arg, cyhal_timer_event_t event)
{
    (void) callback_arg;
    (void) event;

    gyro_flag = true;
}


/*******************************************************************************
* Function Name: gyro_get_data
********************************************************************************
* Summary:
*   Reads gyroscope data and stores it in a buffer.
*
* Parameters:
*     gyro_data: Stores gyroscope data
*
*
*******************************************************************************/
void gyro_get_data(float *gyro_data)
{
    /* Read data from IMU sensor */
    cy_rslt_t result;
#ifdef IM_BMX_160_IMU_SPI
    result = mtb_bmx160_read(&sensor_gyro, &data_gyro);
#endif
#if defined(IM_BMI_160_IMU_SPI) || (IM_BMI_160_IMU_I2C)
    result = mtb_bmi160_read(&sensor_gyro, &data_gyro);
#endif
#ifdef IM_BMI_270_IMU_I2C
    result = mtb_bmi270_read(&sensor_gyro, &data_gyro);
#endif
    if (CY_RSLT_SUCCESS == result)
    {

    #if defined(IM_BMI_160_IMU_SPI) || (IM_BMX_160_IMU_SPI)
        gyro_data[0] = ((float)data_gyro.gyro.y) / (float)0x1000;
        gyro_data[1] = ((float)data_gyro.gyro.x) / (float)0x1000;
        gyro_data[2] = ((float)data_gyro.gyro.z) / (float)0x1000;
    #endif
    #if defined(IM_IMU_BMI270) || (IM_XSS_BMI270)
        gyro_data[0] = ((float)data_gyro.sensor_data.gyr.x) / (float)0x1000;
        gyro_data[1] = ((float)data_gyro.sensor_data.gyr.y) / (float)0x1000;
        gyro_data[2] = ((float)data_gyro.sensor_data.gyr.z) / (float)0x1000;
    #endif
    #ifdef IM_BMI_160_IMU_I2C
        gyro_data[0] = ((float)data_gyro.gyro.x) / (float)0x1000;
        gyro_data[1] = ((float)data_gyro.gyro.y) / (float)0x1000;
        gyro_data[2] = ((float)data_gyro.gyro.z) / (float)0x1000;
    #endif
    }
}
