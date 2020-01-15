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
uint8_t pwsens_flags;
enum
{
	PWSENS_FLAGS_ENABLE = 0x01,
	PWSENS_FLAGS_DIFFERENTIAL = 0x80
};

typedef struct
{
	int16_t mag;		// resulting magnitude
	uint16_t code;		// match code
	uint8_t repeat;		// repeat code
	uint8_t divider;	// freq divider
	uint16_t reserved;
} channel_state_t;

struct
{
	channel_state_t ch[PWSENS_CHANNEL_COUNT];
	enum
	{
		PWSENS_STOPPED,
		PWSENS_COVERTING,
		PWSENS_ANALYSE
	} status;
	uint8_t channel;
} pwsens_state;

static inline void pwsens_adc_set_channel(uint8_t channel)
{
	adc_pos_input_t channel_mux_map[PWSENS_CHANNEL_COUNT] = {
		0x02, // AIN2 ref
		0x05, // AIN4 A (X)
		0x04, // AIN3 B (Y)
		0x06, // AIN6 C (Z)
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
	pwsens_flags = 0;
}

void pwsens_set_flags(uint8_t flags)
{
	pwsens_flags = flags;
	if (((flags & PWSENS_FLAGS_ENABLE) != 0) && pwsens_state.status == PWSENS_STOPPED)
	{
		pwsens_adc_set_channel(0);
		pwsens_state.channel = 0;
		pwsens_start_conversion();
	}
}

void pwsens_set_channel(uint8_t channel, uint8_t divider, uint16_t code, uint8_t repeat)
{
	pwsens_state.ch[channel].divider = divider;
	pwsens_state.ch[channel].code = code;
	pwsens_state.ch[channel].repeat = repeat;
}

// digital matched filter (cross correlation)
// http://en.wikipedia.org/wiki/Cross-correlation
// H[] holds the double sided filter coeffs, M = H.length (number of points in FIR)
// subsample is the number of times for each filter coeff to repeat
// repeat is the number of time for each code to repeat
// ip[] holds input data (length > nPts + M )
// nPts is the length of the required output data
int16_t corrFilter(const int8_t *H, int8_t subsample, uint8_t repeat, int16_t M, const pwgen_sample_type *ip, int16_t nPts, uint16_t* quality)
{
	int16_t sumMax = 0; // max correlation sum
	int16_t sumMin = 0; // min correlation sum
	int16_t Ms = M * subsample; // number of filter coeffs including subsampling

	// compute correlation
	// for each input value
	for (int16_t j=0; j<nPts; j++)
	{
		int16_t sum = 0;
		const pwgen_sample_type *ipi = ip;
		
		for (uint8_t ri = 0; ri < repeat; ++ri)
		{
			const int8_t *Hi = H;
			int8_t ss = 0;
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
		channel_state_t* channel_state = &pwsens_state.ch[channel];
		int8_t subSample = channel_state->divider;
		
		// generate signature code, could be stored
		// TODO (when more ram is available)
		uint16_t code = channel_state->code;
		int8_t sigcode[CODE_SIZE];
		bool differentiate = (pwsens_flags & PWSENS_FLAGS_DIFFERENTIAL) != 0;
		int8_t lastvalue = ((code & (1 << (CODE_SIZE-1))) == 0) ? -1 : 1;
		for (int i = 0; i < CODE_SIZE; i++)
		{
			int8_t val = ((code & (1 << i)) == 0) ? -1 : 1;
			if (differentiate && val == lastvalue)
			    sigcode[i] = 0;
			else
				sigcode[i] = val;
			lastvalue = val;
		}
		
		// center
		int32_t sum = 0;
		for (int i = 0; i < PWSENS_SAMPLE_COUNT; i++)
		{
			// reduce resolution
			ADC_buffer[i] = ADC_buffer[i] >> 4;
			sum += ADC_buffer[i];
		}
		int16_t center = sum / PWSENS_SAMPLE_COUNT;
		for (int i = 0; i < PWSENS_SAMPLE_COUNT; i++)
		{
			// adjust center position
			ADC_buffer[i] = ADC_buffer[i] - center;
		}
		

		uint16_t quality;
		uint8_t repeat = channel_state->repeat;
		int16_t mag = corrFilter(sigcode, subSample, repeat, CODE_SIZE, ADC_buffer, PWSENS_SAMPLE_COUNT - CODE_SIZE * subSample * repeat, &quality);
		channel_state->mag = mag;

				
		uplink_set_channel(channel, mag);
		
		if ((pwsens_flags & PWSENS_FLAGS_ENABLE) != 0)
		{
			pwsens_start_conversion();
		}
	}
}