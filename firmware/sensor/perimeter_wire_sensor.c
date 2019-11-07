#include "perimeter_wire_sensor.h"
#include "uplink.h"
#include <hal_adc_dma.h>
#include <hpl_adc_config.h>
#include <stdlib.h>

#define CODE_SIZE (16)

/*! The buffer size for ADC */
typedef int16_t pwgen_sample_type;
static pwgen_sample_type ADC_buffer[PWSENS_SAMPLE_COUNT];

bool pwsens_adc_running;
uint8_t pwsens_enable;

struct
{
	enum
	{
		PWSENS_STOPPED,
		PWSENS_COVERTING,
		PWSENS_ANALYSE
	} status;
	uint8_t channel;
	uint8_t divider[PWSENS_CHANNEL_COUNT];
	uint16_t code[PWSENS_CHANNEL_COUNT];
	int16_t mag[PWSENS_CHANNEL_COUNT];
} pwsens_state;

static inline void pwsens_adc_set_channel(uint8_t channel)
{
	adc_pos_input_t channel_mux_map[PWSENS_CHANNEL_COUNT] = {
		0x06, // AIN6
		0x03, // AIN3
		0x00, // AIN0
		0x07, // AIN7
	};
	adc_dma_set_inputs(&PWSENS_ADC, channel_mux_map[channel], CONF_ADC_0_MUXNEG, 0);
}

void pwsens_start_conversion(void)
{
	pwsens_state.status = PWSENS_COVERTING;
	hri_adc_write_INPUTCTRL_INPUTOFFSET_bf(PWSENS_ADC.device.hw, 0);
	adc_dma_enable_channel(&PWSENS_ADC, 0);
	adc_dma_read(&PWSENS_ADC, (uint8_t*)ADC_buffer, PWSENS_SAMPLE_COUNT);
}

void pwsens_convert_cb(const struct adc_dma_descriptor *const descr)
{
	adc_dma_disable_channel(&PWSENS_ADC, 0);
	pwsens_state.status = PWSENS_ANALYSE;
}

void pwsens_init(void)
{
	adc_dma_set_conversion_mode(&PWSENS_ADC, ADC_CONVERSION_MODE_FREERUN);
	adc_dma_register_callback(&PWSENS_ADC, ADC_DMA_COMPLETE_CB, pwsens_convert_cb);
	pwsens_adc_running = false;
	pwsens_state.status = PWSENS_STOPPED;
	pwsens_enable = 0;
}

void pwsens_set_enable(uint8_t enable)
{
	pwsens_enable = enable;
	if (enable != 0 && pwsens_state.status == PWSENS_STOPPED)
	{
		pwsens_adc_set_channel(0);
		pwsens_state.channel = 0;
		pwsens_start_conversion();
	}
}

void pwsens_set_channel(uint8_t channel, uint8_t divider, uint16_t code)
{
	pwsens_state.divider[channel] = divider;
	pwsens_state.code[channel] = code;
}

// digital matched filter (cross correlation)
// http://en.wikipedia.org/wiki/Cross-correlation
// H[] holds the double sided filter coeffs, M = H.length (number of points in FIR)
// subsample is the number of times for each filter coeff to repeat
// ip[] holds input data (length > nPts + M )
// nPts is the length of the required output data
int16_t corrFilter(const int8_t *H, int8_t subsample, int16_t M, const pwgen_sample_type *ip, int16_t nPts, uint16_t* quality)
{
	int16_t sumMax = 0; // max correlation sum
	int16_t sumMin = 0; // min correlation sum
	int16_t Ms = M * subsample; // number of filter coeffs including subsampling

	// compute sum of absolute filter coeffs
	int16_t Hsum = 0;
	for (int16_t i=0; i<M; i++)
		Hsum += abs(H[i]);
	Hsum *= subsample;

	// compute correlation
	// for each input value
	for (int16_t j=0; j<nPts; j++)
	{
		int16_t sum = 0;
		const int8_t *Hi = H;
		int8_t ss = 0;
		const pwgen_sample_type *ipi = ip;
		// for each filter coeffs
		for (int16_t i=0; i<Ms; i++)
		{
			sum += ((int16_t)(*Hi)) * ((int16_t)(*ipi));
			ss++;
			if (ss == subsample) {
				ss=0;
				Hi++; // next filter coeffs
			}
			ipi++;
		}
		if (sum > sumMax) sumMax = sum;
		if (sum < sumMin) sumMin = sum;
		ip++;
	}
	// normalize to 4095
	//sumMin = ((float)sumMin) / ((float)(Hsum*127)) * 4095.0;
	//sumMax = ((float)sumMax) / ((float)(Hsum*127)) * 4095.0;
	// compute ratio min/max
	if (sumMax > -sumMin) {
		*quality = (100*sumMax) / -sumMin;
		return sumMax;
	} else {
		*quality = (100* (-sumMin)) / sumMax;
		return sumMin;
	}
}


void pwsens_task(void)
{
	if (pwsens_state.status == PWSENS_ANALYSE)
	{
		uint8_t channel = pwsens_state.channel;
		// next channel
		uint8_t next_channel = channel + 1;
		if (next_channel >= PWSENS_CHANNEL_COUNT)
			next_channel = 0;
		pwsens_state.channel = next_channel;
		pwsens_adc_set_channel(next_channel);

		// oversampling factor
		int8_t subSample = pwsens_state.divider[channel];
		
		uint16_t quality;
		// magnitude for tracking (fast but inaccurate)
		uint16_t code = pwsens_state.code[channel];
		int8_t sigcode[CODE_SIZE];
		for (int i = 0; i < CODE_SIZE; i++)
		{
			sigcode[i] =  ((code & (1 << i)) == 0) ? -1 : 1;
		}
		
		// center
		int16_t center = 1 << 11;
		for (int i = 0; i < PWSENS_SAMPLE_COUNT; i++)
		{
			ADC_buffer[i] = (ADC_buffer[i] - center) >> 4;
		}
		int16_t mag = corrFilter(sigcode, subSample, CODE_SIZE, ADC_buffer, PWSENS_SAMPLE_COUNT - CODE_SIZE * subSample, &quality);
		pwsens_state.mag[channel] = mag;

		//if (swapCoilPolarity)
		//	mag[idx] *= -1;

		// smoothed magnitude used for signal-off detection
		//smoothMag[idx] = 0.99 * smoothMag[idx] + 0.01 * ((float)abs(mag[idx]));

		// perimeter inside/outside detection
		//if (mag[idx] > 0){
		//	signalCounter[idx] = min(signalCounter[idx]+1, 3);
		//	} else {
		//	signalCounter[idx] = max(signalCounter[idx]-1, -3);
		//}
		//if (signalCounter[idx] < 0){
		//	lastInsideTime[idx] = millis();
		//}

				
		uplink_set_channel(channel, mag);
		
		if (pwsens_enable != 0)
		{
			pwsens_start_conversion();
		}
	}
}