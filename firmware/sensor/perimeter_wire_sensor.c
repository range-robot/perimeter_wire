#include "perimeter_wire_sensor.h"
#include "uplink.h"
#include <Adafruit_ZeroFFT.h>
#include <hal_adc_dma.h>
#include <hpl_adc_config.h>

/*! The buffer size for ADC */
#define ADC_BUFFER_COUNT (PWSENS_CHANNEL_COUNT * PWSENS_FFT_WINDOW_SIZE)
static int16_t ADC_buffer[ADC_BUFFER_COUNT];

enum {
	PWSENS_STOPPED,
	PWSENS_COVERTING,
	PWSENS_FFT
} pwsens_state;
bool pwsens_adc_running;
uint8_t pwsens_enable;

void pwsens_start_conversion(void)
{
	pwsens_state = PWSENS_COVERTING;
	hri_adc_write_INPUTCTRL_INPUTOFFSET_bf(PWSENS_ADC.device.hw, 0);
	adc_dma_enable_channel(&PWSENS_ADC, 0);
	adc_dma_read(&PWSENS_ADC, (uint8_t*)ADC_buffer, ADC_BUFFER_COUNT);
}

void pwsens_convert_cb(const struct adc_dma_descriptor *const descr)
{
	adc_dma_disable_channel(&PWSENS_ADC, 0);
		
	pwsens_state = PWSENS_FFT;
}

void pwsens_init(void)
{
	adc_dma_set_conversion_mode(&PWSENS_ADC, ADC_CONVERSION_MODE_FREERUN);
	adc_dma_register_callback(&PWSENS_ADC, ADC_DMA_COMPLETE_CB, pwsens_convert_cb);
	pwsens_adc_running = false;
	pwsens_state = PWSENS_STOPPED;
	pwsens_enable = 0;
}

void pwsens_set_enable(uint8_t enable)
{
	pwsens_enable = enable;
	if (enable != 0 && pwsens_state == PWSENS_STOPPED)
	{
		pwsens_start_conversion();
	}
}

void pwsens_task(void) 
{
	static int i = 0;
	if (pwsens_state == PWSENS_FFT)
	{
		
		ZeroFFT(ADC_buffer, PWSENS_FFT_WINDOW_SIZE);

		uplink_set_channel_a(ADC_buffer[3]);
		uplink_set_channel_b(ADC_buffer[4]);
		uplink_set_channel_c(ADC_buffer[5]);
		uplink_set_channel_d(ADC_buffer[6]);
		
		i+=3;
		if (i >= 1024) {
			i = 0;
		}

		if (pwsens_enable != 0)
			pwsens_start_conversion();
	}
}