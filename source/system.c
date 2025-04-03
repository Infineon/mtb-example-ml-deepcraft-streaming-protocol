/******************************************************************************
* File Name:   system.c
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

#include <cyhal.h>
#include <stdio.h>
#include <cybsp.h>

#include "common.h"
#include "protocol/protocol.h"

#include "devices/dev_bmi270.h"
#include "devices/dev_bmm350.h"
#include "devices/dev_bgt60trxx.h"
#include "devices/dev_dps368.h"
#include "devices/dev_pdm_pcm.h"

#include "system.h"

/*******************************************************************************
* Function Prototypes
*******************************************************************************/

static bool _init_i2c(cyhal_i2c_t* i2c);
static bool _init_spi(cyhal_spi_t* spi);

/*******************************************************************************
* Function Name: _init_i2c
********************************************************************************
* Summary:
*   Initializes the I2C interface.
*
* Parameters:
*   i2c: Pointer to the I2C object.
*
* Return:
*   True if initialization is successful, otherwise false.
*******************************************************************************/
static bool _init_i2c(cyhal_i2c_t* i2c)
{
    cy_rslt_t result;

    /* I2C config structure */
    cyhal_i2c_cfg_t i2c_config =
    {
         .is_slave = false,
         .address = 0,
         .frequencyhal_hz = 400000
    };

    /* Initialize I2C for IMU communication */
    result = cyhal_i2c_init(i2c, CYBSP_I2C_SDA, CYBSP_I2C_SCL, NULL);
    if(result != CY_RSLT_SUCCESS)
    {
        return false;
    }

    /* Configure the I2C */
    result = cyhal_i2c_configure(i2c, &i2c_config);
    if(result != CY_RSLT_SUCCESS)
    {
        return false;
    }

    return true;
}

/*******************************************************************************
* Function Name: _init_spi
********************************************************************************
* Summary:
*   Initializes the SPI interface.
*
* Parameters:
*   spi: Pointer to the SPI object.
*
* Return:
*   True if initialization is successful, otherwise false.
*******************************************************************************/
static bool _init_spi(cyhal_spi_t* spi)
{
    cy_rslt_t result;

    result = cyhal_spi_init(
            spi,
            CYBSP_RSPI_MOSI,
            CYBSP_RSPI_MISO,
            CYBSP_RSPI_CLK,
            NC,
            NULL,
            8,
            CYHAL_SPI_MODE_00_MSB,
            false);
    if(result != CY_RSLT_SUCCESS)
    {
        return false;
    }

    result = cyhal_spi_set_frequency(spi, 18750000UL);

    if(result != CY_RSLT_SUCCESS)
    {
        return false;
    }

    return true;
}

/*******************************************************************************
* Function Name: system_load_device_drivers
********************************************************************************
* Summary:
*   Load all device drivers
*
* Parameters:
*   protocol: Pointer to the protocol_t instance.
*
*******************************************************************************/
void system_load_device_drivers(protocol_t* protocol)
{
    static cyhal_i2c_t i2c;
    static cyhal_spi_t spi;

    /* Enable i2c */
    if(!_init_i2c(&i2c))
    {
        printf("Init i2c failed.\n");
        halt_error(LED_CODE_I2C_ERROR);
    }

    /* Enable SPI(if available) */
    if(!_init_spi(&spi))
    {
        printf("Init spi failed.\n");
        halt_error(LED_CODE_SPI_ERROR);
    }

#ifdef IM_ENABLE_PDM_PCM
    if(!dev_pdm_pcm_register(protocol))
    {
        printf("PDM PCM registration failed.\n");
        halt_error(LED_CODE_SENSOR_PDM_PCM_ERROR);
    }
#endif

#ifdef IM_ENABLE_BMI270
    if(!dev_bmi270_register(protocol, &i2c))
    {
        printf("BMI270 registration failed.\n");
        halt_error(LED_CODE_SENSOR_BMI270_ERROR);
    }
#endif

#ifdef IM_ENABLE_BMM350
    if(!dev_bmm350_register(protocol, &i2c)) {
        printf("BMM350 registration failed.\n");
        halt_error(LED_CODE_SENSOR_BMM350_ERROR);
    }
#endif

#ifdef IM_ENABLE_DPS368
    if(!dev_dps368_register(protocol, &i2c))
    {
        printf("DPS368 registration failed.\n");
        halt_error(LED_CODE_SENSOR_DPS368_ERROR);
    }
#endif

#ifdef IM_ENABLE_BGT60TRXX
    if(!dev_bgt60trxx_register(protocol, &spi))
    {
        printf("BGT60TRXX registration failed.\n");
        halt_error(LED_CODE_SENSOR_BGT60TRXX_ERROR);
    }
#endif

}

/* [] END OF FILE */
