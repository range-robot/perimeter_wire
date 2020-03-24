#include "perimeter_wire_sensor.h"
#include "uplink.h"
#include <hal_adc_dma.h>
#include <hpl_adc_config.h>
#include <stdlib.h>
#include "error.h"


/*
NOTE:
When the ADC runs at high speed e.g. 2MHz a break of the DMA transfer was observed
Error callback not fired. Seems like the DMA ist stalled.
Reduce the ADC freq. to solve the problem. (1.5MHz seems to work fine)
*/

/*! The buffer size for ADC */
typedef int16_t pwgen_sample_type;
static pwgen_sample_type ADC_buffer[PWSENS_SAMPLE_COUNT];

const uint8_t channel_offset[3] = {
	0,
	2,
	1
};

volatile uint8_t pwsens_flags;


struct
{
	uint16_t code;		// match code
	uint8_t repeat;		// repeat code
	uint8_t divider;	// freq divider
	volatile enum
	{
		PWSENS_STOPPED,
		PWSENS_COVERTING,
		PWSENS_ANALYSE
	} status;
} pwsens_state;

/* code */
#define CODE_SIZE (16)
pwsens_code_buffer[CODE_SIZE];

void pwsens_set_sig_code(uint16_t code, bool differentiate)
{
	int8_t lastvalue = ((code & (1 << (CODE_SIZE-1))) == 0) ? -1 : 1;
	for (int i = 0; i < CODE_SIZE; i++)
	{
		int8_t val = ((code & (1 << i)) == 0) ? -1 : 1;
		if (differentiate && val == lastvalue)
			pwsens_code_buffer[i] = 0;
		else
			pwsens_code_buffer[i] = val;
		lastvalue = val;
	}
}

// buffer access
uint16_t pwsens_get_buffer_length()
{
	return PWSENS_SAMPLE_COUNT;
}
uint16_t pwsens_get_buffer_value(uint16_t index)
{
	return (uint16_t)ADC_buffer[index];
}	

void pwsens_start_conversion(void)
{
	pwsens_state.status = PWSENS_COVERTING;
	hri_adc_write_INPUTCTRL_INPUTOFFSET_bf(PWSENS_ADC.device.hw, 0);
	adc_dma_set_inputs(&PWSENS_ADC, 0x04 /* AIN4 */, 0x00 /* AIN0 */, 0);
	adc_dma_set_conversion_mode(&PWSENS_ADC, ADC_CONVERSION_MODE_FREERUN);
	adc_dma_enable_channel(&PWSENS_ADC, 0);
	adc_dma_read(&PWSENS_ADC, (uint8_t*)ADC_buffer, PWSENS_SAMPLE_COUNT);
	//adc_dma_start_conversion(&PWSENS_ADC);
}

void pwsens_convert_cb(const struct adc_dma_descriptor *const descr)
{
	adc_dma_disable_channel(&PWSENS_ADC, 0);
	adc_dma_set_conversion_mode(&PWSENS_ADC, ADC_CONVERSION_MODE_SINGLE_CONVERSION);
	pwsens_state.status = PWSENS_ANALYSE;
}

void pwsens_error_cb(const struct adc_dma_descriptor *const descr)
{
	adc_dma_disable_channel(&PWSENS_ADC, 0);
	adc_dma_set_conversion_mode(&PWSENS_ADC, ADC_CONVERSION_MODE_SINGLE_CONVERSION);
	system_throw_error(ERR_ABORTED);
	pwsens_start_conversion();
}

void pwsens_init(void)
{
	adc_dma_set_conversion_mode(&PWSENS_ADC, ADC_CONVERSION_MODE_FREERUN);
	adc_dma_register_callback(&PWSENS_ADC, ADC_DMA_ERROR_CB, pwsens_error_cb);
	adc_dma_register_callback(&PWSENS_ADC, ADC_DMA_COMPLETE_CB, pwsens_convert_cb);
	adc_dma_set_inputs(&PWSENS_ADC, 0x04 /* AIN4 */, 0x00 /* AIN0 */, 0);
	pwsens_state.status = PWSENS_STOPPED;
	pwsens_flags = 0;
}

void pwsens_set_flags(uint8_t flags)
{
	pwsens_flags = flags;
	if (((flags & PWSENS_FLAGS_START) != 0) && pwsens_state.status == PWSENS_STOPPED)
	{
		pwsens_set_sig_code(pwsens_state.code, (pwsens_flags & PWSENS_FLAGS_DIFFERENTIAL) != 0);
		pwsens_start_conversion();
	}
}

void pwsens_set_config(uint8_t channel, uint8_t divider, uint16_t code, uint8_t repeat)
{
	pwsens_state.divider = divider;
	pwsens_state.code = code;
	pwsens_state.repeat = repeat;
}


int16_t crossCorrelation(const int8_t* Hi, const pwgen_sample_type* ipi, uint8_t repeat, uint8_t subsample, uint16_t nCoeffsSubsample)
{
	int16_t sum = 0;
	
	for (uint8_t ri = 0; ri < repeat; ++ri)
	{
		int8_t ss = 0;
		// for each filter coeffs
		for (int16_t i=0; i<nCoeffsSubsample; i++)
		{
			sum += ((int16_t)(*Hi)) * ((int16_t)(*ipi));
			ss++;
			if (ss == subsample) {
				ss=0;
				Hi++; // next filter coeffs
			}
			ipi += PWSENS_SAMPLE_STRIDE;
		}
	}
	return sum;
}

// digital matched filter (cross correlation)
// http://en.wikipedia.org/wiki/Cross-correlation
// coeffs[] holds the double sided filter coeffs, nCoeffs = coeffs.length (number of points in FIR)
// subsample is the number of times for each filter coeff to repeat
// repeat is the number of time for each code to repeat
// ip[] holds input data (length > nPts + M )
// nPts is the length of the required output data
int16_t corrFilter(const int8_t *coeffs, uint16_t nCoeffs, uint8_t subsample, uint8_t repeat, const pwgen_sample_type *ip, int16_t nPts, uint16_t* quality)
{
	int16_t sumMax = 0; // max correlation sum
	int16_t sumMin = 0; // min correlation sum
	uint16_t nCoeffsSubsample = nCoeffs * subsample; // number of filter coeffs including subsampling

	// compute correlation
	// for each input value
	for (int16_t j=0; j<nPts; j++)
	{
		int16_t sum = crossCorrelation(coeffs, ip, repeat, subsample, nCoeffsSubsample);
		if (sum > sumMax)
		{
			sumMax = sum;
		}
		if (sum < sumMin)
		{
			sumMin = sum;
		}
		ip += PWSENS_SAMPLE_STRIDE;
	}
	// compute ratio min/max
	if (sumMax > -sumMin) {
		*quality = (1000*sumMax) / -sumMin;
		return sumMax;
	} else {
		*quality = (1000* (-sumMin)) / sumMax;
		return sumMin;
	}
}

// digital matched filter (cross correlation)
// http://en.wikipedia.org/wiki/Cross-correlation
// coeffs[] holds the double sided filter coeffs, nCoeffs = coeffs.length (number of points in FIR)
// subsample is the number of times for each filter coeff to repeat
// repeat is the number of time for each code to repeat
// ip[] holds input data (length > nPts + M )
// nPts is the length of the required output data
// mag magnitude results length must be PWSENS_SAMPLE_STRIDE
uint32_t corrFilterSync(const int8_t *coeffs, uint16_t nCoeffs, uint8_t subsample, uint8_t repeat, const pwgen_sample_type *ip, int16_t nPts, int16_t* mag, uint16_t* pos)
{
	uint16_t nCoeffsSubsample = nCoeffs * subsample; // number of filter coeffs including subsampling

	// compute correlation
	// for each input value
	int32_t sum[PWSENS_SAMPLE_STRIDE] = {0};
	int32_t sumSQ[PWSENS_SAMPLE_STRIDE] = {0};
	int32_t normMax = 0;
	uint16_t maxPos = 0;
	uint8_t offset = 0;
	for (int16_t j=0; j<nPts; j++)
	{
		if (offset >= PWSENS_SAMPLE_STRIDE)
			offset = 0;

		int32_t s = (int32_t) crossCorrelation(coeffs, ip, repeat, subsample, nCoeffsSubsample);
		sum[offset] = s;
		sumSQ[offset] = s * s;
		int32_t norm = sumSQ[0] + sumSQ[1] + sumSQ[2];  // PWSENS_SAMPLE_STRIDE
		ASSERT(norm >= 0);
		if (norm > normMax)
		{
			normMax = norm;
			mag[0] = sum[0];
			mag[1] = sum[1];
			mag[2] = sum[2];  // PWSENS_SAMPLE_STRIDE
			maxPos = j;
		}
		ip ++;
		offset++;
	}
	*pos = maxPos;
	return (uint32_t)normMax;
}



void pwsens_task(void)
{	
	if (pwsens_state.status == PWSENS_ANALYSE)
	{
		uint8_t flags = pwsens_flags;
		if ((flags & PWSENS_FLAGS_SYNC_MODE) == 0)
		{
			int8_t divider = pwsens_state.divider;	
			uint8_t repeat = pwsens_state.repeat;
			for (uint8_t channel = 0; channel < PWSENS_CHANNEL_COUNT; channel++)
			{
				if ((flags & (PWSENS_FLAGS_ENABLE_CH_A << channel)) == 0)
					continue;

				// apply filter to each channel individually.
				uint16_t quality;
				uint16_t nPos = (PWSENS_SAMPLE_COUNT / PWSENS_SAMPLE_STRIDE) - CODE_SIZE * divider * repeat;
				int16_t mag = corrFilter(pwsens_code_buffer, CODE_SIZE, divider, repeat,
										ADC_buffer + channel_offset[channel], nPos,
										&quality);
				uplink_set_channel(channel, mag, quality);
			}  // for channel
		}
		else
		{
			int8_t divider = pwsens_state.divider;	
			uint8_t repeat = pwsens_state.repeat;
			
			// apply filter to all channels at once
			uint16_t pos;
			uint16_t nPos = (PWSENS_SAMPLE_COUNT) - CODE_SIZE * divider * repeat * PWSENS_SAMPLE_STRIDE;
			uint16_t mag[PWSENS_SAMPLE_STRIDE];
			uint32_t norm = corrFilterSync(pwsens_code_buffer, CODE_SIZE, divider, repeat,
						ADC_buffer, nPos,
						&mag, &pos);
			uplink_set_channel(0, mag[channel_offset[0]], pos);
			uplink_set_channel(1, mag[channel_offset[1]], norm >> 8);
			uplink_set_channel(2, mag[channel_offset[2]], 0);
		}

		if ((flags & PWSENS_FLAGS_ENABLE) != 0 && (flags & PWSENS_FLAGS_CONTINUOUS) != 0)
		{
			pwsens_start_conversion();
		}
		else
		{
			pwsens_state.status = PWSENS_STOPPED;
		}	
	}
}
