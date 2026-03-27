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
 * (c) 2025, Infineon Technologies AG, or an affiliate of Infineon
 * Technologies AG. All rights reserved.
 * This software, associated documentation and materials ("Software") is
 * owned by Infineon Technologies AG or one of its affiliates ("Infineon")
 * and is protected by and subject to worldwide patent protection, worldwide
 * copyright laws, and international treaty provisions. Therefore, you may use
 * this Software only as provided in the license agreement accompanying the
 * software package from which you obtained this Software. If no license
 * agreement applies, then any use, reproduction, modification, translation, or
 * compilation of this Software is prohibited without the express written
 * permission of Infineon.
 * 
 * Disclaimer: UNLESS OTHERWISE EXPRESSLY AGREED WITH INFINEON, THIS SOFTWARE
 * IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 * INCLUDING, BUT NOT LIMITED TO, ALL WARRANTIES OF NON-INFRINGEMENT OF
 * THIRD-PARTY RIGHTS AND IMPLIED WARRANTIES SUCH AS WARRANTIES OF FITNESS FOR A
 * SPECIFIC USE/PURPOSE OR MERCHANTABILITY.
 * Infineon reserves the right to make changes to the Software without notice.
 * You are responsible for properly designing, programming, and testing the
 * functionality and safety of your intended application of the Software, as
 * well as complying with any legal requirements related to its use. Infineon
 * does not guarantee that the Software will be free from intrusion, data theft
 * or loss, or other breaches ("Security Breaches"), and Infineon shall have
 * no liability arising out of any Security Breaches. Unless otherwise
 * explicitly approved by Infineon, the Software may not be used in any
 * application where a failure of the Product or any consequences of the use
 * thereof can reasonably be expected to result in personal injury.
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
