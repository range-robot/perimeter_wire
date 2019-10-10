/*
 * Code generated from Atmel Start.
 *
 * This file will be overwritten when reconfiguring your Atmel Start project.
 * Please copy examples or other code you want to keep to a separate file
 * to avoid losing it when reconfiguring.
 */

#include "driver_examples.h"
#include "driver_init.h"
#include "utils.h"

/*! The buffer size for ADC */
#define ADC_0_BUFFER_SIZE 16
static uint8_t ADC_0_buffer[ADC_0_BUFFER_SIZE];

static void convert_cb_ADC_0(const struct adc_dma_descriptor *const descr)
{
}

/**
 * Example of using ADC_0 to generate waveform.
 */
void ADC_0_example(void)
{
	/* Enable ADC freerun mode in order to make example work */
	adc_dma_register_callback(&ADC_0, ADC_DMA_COMPLETE_CB, convert_cb_ADC_0);
	adc_dma_enable_channel(&ADC_0, 0);
	adc_dma_read(&ADC_0, ADC_0_buffer, ADC_0_BUFFER_SIZE);
}
