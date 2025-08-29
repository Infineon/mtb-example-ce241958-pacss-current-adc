/******************************************************************************
 * File Name:   main.c
 *
 * Description: This is the source code for the CE241958 PACSS current ADC
 *  application for ModusToolbox.
 *
 * Related Document: See README.md
 *
 *
 *******************************************************************************
 * Copyright 2025, Cypress Semiconductor Corporation (an Infineon company) or
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

/*******************************************************************************
 * Include header files
 ********************************************************************************/
#include "cy_pdl.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include <stdio.h>
#include <string.h>

/*******************************************************************************
 * Macros and Constants
 ********************************************************************************/
#define CY_ADC_HBPGR_VOLTAGE               (1.2F)    /* HPBGR reference voltage */
#define CY_ADC_VOLTS_TO_MV                 (1000u)   /* Volts to mV */
#define SHUNT_RESISTANCE_VALUE             (4.02F)  /* Shunt resistance */

/*******************************************************************************
 * Function Prototypes
 ********************************************************************************/
static void DSADC_Dchan0_IntrISR(void);

/*******************************************************************************
 * Global Variables
 ********************************************************************************/
static volatile bool g_currentDataValid = false;
float shuntResistorCurrent              = 0.0F;

/******************************************************************************
 * Switch interrupt configuration structure
 *******************************************************************************/
cy_stc_sysint_t DCHAN0IntrConfig =
{
    .intrSrc      = (IRQn_Type) pacss_interrupts_dch_0_IRQn,
    .intrPriority = 2U,
};

/*******************************************************************************
 * Function Name: main
 ********************************************************************************
 * Summary:
 * System entrance point. This function performs
 *  1. Initializes the BSP.
 *  2. This function initializes the peripherals to measure the current
 *     and display the result accordingly.
 *
 * Parameters:
 *  None
 *
 * Return:
 *  int
 *
 *******************************************************************************/
int main(void)
{
    cy_rslt_t result;
    float     millivoltsAcrossShunt = 0.0F;
    int32_t   currentChannelResult  = 0;
 
    __enable_irq(); /* Enable global interrupts. */


    /* Allocate context for UART operation */
    cy_stc_scb_uart_context_t uartContext;

    /* Initialize the device and board peripherals */
    result = cybsp_init();
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }
    Cy_SCB_UART_Init(CYBSP_UART_HW, &CYBSP_UART_config, &uartContext);
    Cy_SCB_UART_Enable(CYBSP_UART_HW);
    cy_retarget_io_init(CYBSP_UART_HW);

    /* Hook interrupt service routine */
    (void) Cy_SysInt_Init(&DCHAN0IntrConfig, &DSADC_Dchan0_IntrISR);
    /* Clearing the PACSS interrupt in NVIC */
    NVIC_ClearPendingIRQ(DCHAN0IntrConfig.intrSrc);
    /* set mask to enable interrupts */
    Cy_DSADC_SetInterruptMask(PACSS_DCHAN0, CY_DSADC_INTR_MASK);
    /* enable interrupt for interrupt controller */
    NVIC_EnableIRQ(DCHAN0IntrConfig.intrSrc);

    printf("\x1b[2J\x1b[;H");
    printf("CE241958  - PSoC 4 - PACSS current ADC\n\r");

    /* Start the conversion process */
    Cy_DSADC_StartConversionAchan(PACSS_ACHAN0);

    if (NULL != pacss_0_dchan_0_channel_config.fir)
    {
        /* Settle the FIR filters
         * first CY_ADC_FIR_NUM_TAPS measurements are incorrect
         * so we need to skip them
         */
        uint32_t currentFilterSettle = 0U;
        while ((currentFilterSettle < pacss_0_dchan_0_channel_config.fir->numTaps))
        {
            if (g_currentDataValid)
            {
                g_currentDataValid = false;
                currentFilterSettle++;
            }
        }
    }

    while (1)
    {
        if (g_currentDataValid)
        {
            g_currentDataValid = false;
            /* Get the ADC result and calculate the shunt voltage */
            currentChannelResult = Cy_DSADC_GetResult(PACSS_DCHAN0);
            millivoltsAcrossShunt = (float) Cy_DSADC_CountsTo_mVolts(currentChannelResult, CY_ADC_HBPGR_VOLTAGE, CY_DSADC_ANALOG_GAIN_DCHAN0, &pacss_0_dchan_0_channel_config);

            shuntResistorCurrent = (millivoltsAcrossShunt) / SHUNT_RESISTANCE_VALUE;

            printf("PACSS Current: %.2f[mA]\r\n", shuntResistorCurrent);
        }
        Cy_SysLib_Delay(1000);
    }
}

/*******************************************************************************
 * Function Name: isr_counter
 ********************************************************************************
 * Summary: Interrupt service routine for valid interrupt.
 *
 * Parameters:
 *  None
 *
 * Return:
 *  None
 *******************************************************************************/
static void DSADC_Dchan0_IntrISR(void)
{
    g_currentDataValid = true;

    Cy_DSADC_ClearInterrupt(PACSS_DCHAN0, CY_DSADC_INTR_MASK);
}

/* [] END OF FILE */
