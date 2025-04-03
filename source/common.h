/******************************************************************************
* File Name: common.h
*
* Description:
*   This file contains common utility functions used across the project.
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

#ifndef __COMMON_H__
#define __COMMON_H__

#include <stdbool.h>

/*******************************************************************************
* Defines
*******************************************************************************/

#define LED_SHORT_ON_TIME 100         /* Short blink duration in milliseconds */
#define LED_LONG_ON_TIME 300          /* Long blink duration in milliseconds */
#define LED_OFF_TIME 300              /* Pause duration between blinks */
#define LED_CODE_SEPARATOR_TIME 3000  /* Pause duration between repeating error codes in milliseconds */

/*
*  Blink pattern:
*  0:  .        10: ..-      20: .--.
*  1:  -        11: -.-      21: ---.
*  2:  ..       12: .--      22: ...-
*  3:  -.       13: ---      23: -..-
*  4:  .-       14: ....     24: .-.-
*  5:  --       15: -...     25: --.-
*  6:  ...      16: .-..     26: ..--
*  7:  -..      17: --..     27: -.--
*  8:  .-.      18: ..-.     28: .---
*  9:  --.      19: -.-.     29: ----
*
*  Avoid patterns with only dots or dashes.
*  They are hard distinguish from each other.
*/
#define LED_CODE_GENERIC_ERROR          (3)
#define LED_CODE_UART_ERROR             (4)
#define LED_CODE_MEMORY_ERROR           (7)
#define LED_CODE_I2C_ERROR              (8)
#define LED_CODE_STDOUT_RETARGET_ERROR  (9)
#define LED_CODE_CLOCK_ERROR            (10)
#define LED_CODE_SPI_ERROR              (11)
#define LED_CODE_WATCHDOG_ERROR         (12)
#define LED_CODE_SENSOR_PDM_PCM_ERROR   (15) /* Microphone error */
#define LED_CODE_SENSOR_BMI270_ERROR    (16) /* Accelerometer/Gyro */
#define LED_CODE_SENSOR_BMM350_ERROR    (17) /* Magnetometer */
#define LED_CODE_SENSOR_DPS368_ERROR    (18) /* Digital pressure sensor */
#define LED_CODE_SENSOR_BGT60TRXX_ERROR (19) /* RADAR sensor */

/*
 * Enable verbose printing to debug console.
 * This might have a performance impact and should normally be disabled.
 * Use this for high-frequency printing.*/
/* #define ENABLE_DEBUG_PRINT (1) */

#if ENABLE_DEBUG_PRINT
    #define DEBUG_PRINT(...)  printf(__VA_ARGS__)
#else
    #define DEBUG_PRINT(...)  /* Do nothing */
#endif

/*******************************************************************************
* Function Prototypes
*******************************************************************************/

void set_led(bool state);
void halt_error(int led_flash_code);

#endif /* __COMMON_H__ */

/* [] END OF FILE */
