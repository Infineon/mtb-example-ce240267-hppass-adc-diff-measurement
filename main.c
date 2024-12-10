/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the HPPASS SAR ADC differential
* measurement example for ModusToolbox.
*
* Related Document: See README.md
*
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

/*******************************************************************************
* Header Files
*******************************************************************************/
#include "cybsp.h"
#include "cy_pdl.h"
#include "mtb_hal.h"
#include "cy_retarget_io.h"


/*******************************************************************************
* Macros
********************************************************************************/
/* The number of SAR ADC pseudo differential channels
 * Pseudo differential channel 0 - AN_A0(Positive input)/AN_A1(Negative input)
 * Pseudo differential channel 1 - AN_A2(Positive input)/AN_A3(Negative input)
 * Pseudo differential channel 2 - AN_A4(Positive input)/AN_A5(Negative input)
 */
#define NUM_ADC_DIFF_CHANNELS               (3U)

/* The definition of HPPASS AC startup timeout in microseconds.
 * HPPASS startup time contains AREF startup 40us, CSG startup about 15us and 
 * SAR ADC maximum self-calibration time 9ms (HPPASS input clock is 240MHz). To be
 * on the safe side, add to 10ms.
 */
#define HPPASS_AC_STARTUP_TIMEOUT           (10000U)

/* ADC result data contains channel number identifier, get ADC channel ID and
 * data from result data: [20:16] - channel ID, [15:0] - channel result
 */
#define GET_ADC_CHAN_ID(result)     ((uint8_t)_FLD2VAL(CY_HPPASS_FIFO_RD_DATA_CHAN_ID, result))
#define GET_ADC_CHAN_RESULT(result) ((uint16_t) _FLD2VAL(CY_HPPASS_FIFO_RD_DATA_RESULT, result))

/*******************************************************************************
* Global Variables
********************************************************************************/
/* For the Retarget-IO (Debug UART) usage */
static cy_stc_scb_uart_context_t  DEBUG_UART_context;   /* Debug UART context */
static mtb_hal_uart_t DEBUG_UART_hal_obj;               /* Debug UART HAL object */

/* The interrupt configuration structure of ADC DMA */
const cy_stc_sysint_t adc_dma_intr_config =
{
    .intrSrc = ADC_DMA_IRQ,
    .intrPriority = 0U,
};

/* The result buffer for ADC pseudo differential channels, contain channel ID */
uint32_t adc_result_buffer[NUM_ADC_DIFF_CHANNELS] = {0};

/* ADC DMA completion flag */
volatile bool adc_dma_done_flag = false;

/* ADC DMA completion flag */
volatile bool adc_dma_error_flag = false;

/*******************************************************************************
* Function Prototypes
********************************************************************************/
/* ADC DMA interrupt handler */
void adc_dma_intr_handler(void);


/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
* This is the main function for CPU.
*
* Parameters:
*  void
*
* Return:
*  int
*
*******************************************************************************/
int main(void)
{
    cy_rslt_t result;

    /* Initialize the device and board peripherals */
    result = cybsp_init();
    /* Board init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Initialize the debug UART */
    result = Cy_SCB_UART_Init(DEBUG_UART_HW, &DEBUG_UART_config, &DEBUG_UART_context);
    /* UART init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }
    Cy_SCB_UART_Enable(DEBUG_UART_HW);

    /* Setup the HAL UART */
    result = mtb_hal_uart_setup(&DEBUG_UART_hal_obj, &DEBUG_UART_hal_config, &DEBUG_UART_context, NULL);
    /* HAL UART init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Initialize retarget-io to use the debug UART port */
    result = cy_retarget_io_init(&DEBUG_UART_hal_obj);
    /* retarget-io init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Initialize user timer using the config structure generated using device configurator*/
    if (CY_TCPWM_SUCCESS != Cy_TCPWM_Counter_Init(USER_TIMER_HW, USER_TIMER_NUM, &USER_TIMER_config))
    {
        CY_ASSERT(0);
    }
    /* Enable the initialized user timer */
    Cy_TCPWM_Counter_Enable(USER_TIMER_HW, USER_TIMER_NUM);

    /* Initialize ADC DMA descriptor 0 */
    if (CY_DMA_SUCCESS != Cy_DMA_Descriptor_Init(&ADC_DMA_Descriptor_0, &ADC_DMA_Descriptor_0_config))
    {
        CY_ASSERT(0);
    }
    /* Set source and destination address for ADC DMA descriptor 0 */
    Cy_DMA_Descriptor_SetSrcAddress(&ADC_DMA_Descriptor_0, (uint32_t *) CY_HPPASS_SAR_FIFO_READ_PTR(0));
    Cy_DMA_Descriptor_SetDstAddress(&ADC_DMA_Descriptor_0, (uint32_t *) adc_result_buffer);
    /* ADC DMA channel configuration */
    if (CY_DMA_SUCCESS != Cy_DMA_Channel_Init(ADC_DMA_HW, ADC_DMA_CHANNEL, &ADC_DMA_channelConfig))
    {
        CY_ASSERT(0);
    }
    Cy_DMA_Channel_SetDescriptor(ADC_DMA_HW, ADC_DMA_CHANNEL, &ADC_DMA_Descriptor_0);
    /* Initialize and enable interrupt for ADC DMA */
    Cy_DMA_Channel_SetInterruptMask(ADC_DMA_HW, ADC_DMA_CHANNEL, CY_DMA_INTR_MASK);
    Cy_SysInt_Init(&adc_dma_intr_config, adc_dma_intr_handler);
    NVIC_EnableIRQ(adc_dma_intr_config.intrSrc);
    /* Enable ADC DMA channel */
    adc_dma_done_flag = false;
    adc_dma_error_flag = false;
    Cy_DMA_Channel_Enable(ADC_DMA_HW, ADC_DMA_CHANNEL);
    /* Enable DMA */
    Cy_DMA_Enable(ADC_DMA_HW);

    /* Start the HPPASS autonomous controller (AC) from state 0 */
    if(CY_HPPASS_SUCCESS != Cy_HPPASS_AC_Start(0U, HPPASS_AC_STARTUP_TIMEOUT))
    {
        CY_ASSERT(0);
    }

    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
    printf("\x1b[2J\x1b[;H");
    printf("********************************************************************************\r\n");
    printf("HPPASS: SAR ADC pseudo differential measurement example\r\n");
    printf("********************************************************************************\r\n");

    /* Enable global interrupts */
    __enable_irq();

    /* Software start user timer */
    Cy_TCPWM_TriggerStart_Single(USER_TIMER_HW, USER_TIMER_NUM);
    
    for (;;)
    {
        /* Check if all ADC results are transferred to memory by DMA */
        if(adc_dma_done_flag)
        {
            adc_dma_done_flag = false;

            /* Get channel ID of ADC result data, print out ADC differential channel result by UART */
            printf("ADC differential channels result - ");
            for(int i = 0; i < NUM_ADC_DIFF_CHANNELS; i++)
            {
                switch(GET_ADC_CHAN_ID(adc_result_buffer[i]))
                {
                    case ADC_DIFF_0_CHAN_IDX:
                        printf("diff 0: 0x%04x, ", GET_ADC_CHAN_RESULT(adc_result_buffer[i]));
                        break;
                    case ADC_DIFF_1_CHAN_IDX:
                        printf("diff 1: 0x%04x, ", GET_ADC_CHAN_RESULT(adc_result_buffer[i]));
                        break;
                    case ADC_DIFF_2_CHAN_IDX:
                        printf("diff 2: 0x%04x", GET_ADC_CHAN_RESULT(adc_result_buffer[i]));
                        break;
                    default:
                        break;
                }
            }
            printf("\r\n");
        }

        /* Check if DMA error occurred */
        if(adc_dma_error_flag)
        {
            adc_dma_error_flag = false;
            printf("ADC DMA error\r\n\r\n");
        }
    }
}

/*******************************************************************************
* Function Name: adc_dma_intr_handler
********************************************************************************
* Summary:
* This function is the ADC DMA interrupt handler.
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
void adc_dma_intr_handler(void)
{
    Cy_DMA_Channel_ClearInterrupt(ADC_DMA_HW, ADC_DMA_CHANNEL);
    /* Check interrupt cause to capture errors. */
    cy_en_dma_intr_cause_t dma_status = Cy_DMA_Channel_GetStatus(ADC_DMA_HW, ADC_DMA_CHANNEL);
    if (CY_DMA_INTR_CAUSE_COMPLETION == dma_status)
    {
        adc_dma_done_flag = true;
    }
    else
    {
        /* DMA error occurred */
        adc_dma_error_flag = true;
    }
}

/* [] END OF FILE */
