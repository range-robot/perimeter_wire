#include "perimeter_wire_sensor.h"
#include "uplink.h"
#include <hal_adc_dma.h>
#include <hpl_adc_config.h>
#include <stdlib.h>
#include "driver_init.h"
#include "error.h"
#include "matching_filter.h"

/*
NOTE:
When the ADC runs at high speed e.g. 2MHz a break of the DMA transfer was observed
Error callback not fired. Seems like the DMA ist stalled.
Reduce the ADC freq. to solve the problem. (1.5MHz seems to work fine)
*/

static pwgen_sample_type ADC_buffer1[PWSENS_SAMPLE_COUNT];
static pwgen_sample_type ADC_buffer2[PWSENS_SAMPLE_COUNT];
static pwgen_sample_type* ADC_buffer_reading = ADC_buffer1;
static pwgen_sample_type* ADC_buffer_anlysing = ADC_buffer2;
static struct
{
	int16_t buffer[PWSENS_CHANNEL_COUNT][PWSENS_MAX_FILTER];
	uint8_t pos;
} filter_state;;
static struct timer_task pwsens_dma_task;


// channel offset
// pyhsical mapping
// +1 offset for inputscan
const uint8_t channel_offset[3] = {
	1,
	0,
	2
};

volatile uint8_t pwsens_flags;


struct
{
	uint16_t cfilter_length;  // precalculated filter length
	uint16_t code;		// match code
	uint8_t repeat;		// repeat code
	uint8_t divider;	// freq divider
	uint8_t filter;		// filter length
	volatile enum
	{
		PWSENS_STOPPED,
		PWSENS_COVERTING,
		PWSENS_ANALYSE
	} status;
} pwsens_state;

/* code */
#define CODE_SIZE (16)
int8_t pwsens_code_buffer[CODE_SIZE];

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
	return (uint16_t)ADC_buffer_anlysing[index];
}	

void pwsens_start_conversion(void)
{
	adc_dma_set_inputs(&PWSENS_ADC, 0x04 /* AIN4 */, 0x00 /* AIN0 */, 0);
	adc_dma_set_conversion_mode(&PWSENS_ADC, ADC_CONVERSION_MODE_FREERUN);
	hri_adc_write_INPUTCTRL_INPUTOFFSET_bf(PWSENS_ADC.device.hw, 0);
	adc_dma_enable_channel(&PWSENS_ADC, 0);
	adc_dma_read(&PWSENS_ADC, (uint8_t*)ADC_buffer_reading, PWSENS_SAMPLE_COUNT);
	timer_add_task(&TIMER_0, &pwsens_dma_task);
	//adc_dma_start_conversion(&PWSENS_ADC);
}

void pwsens_convert_cb(const struct adc_dma_descriptor *const descr)
{
	adc_dma_disable_channel(&PWSENS_ADC, 0);
	adc_dma_set_conversion_mode(&PWSENS_ADC, ADC_CONVERSION_MODE_SINGLE_CONVERSION);
	pwsens_state.status = PWSENS_ANALYSE;
	timer_remove_task(&TIMER_0, &pwsens_dma_task);
}

void pwsens_error_cb(const struct adc_dma_descriptor *const descr)
{
	adc_dma_disable_channel(&PWSENS_ADC, 0);
	adc_dma_set_conversion_mode(&PWSENS_ADC, ADC_CONVERSION_MODE_SINGLE_CONVERSION);
	system_throw_error(ERR_ABORTED);
	timer_remove_task(&TIMER_0, &pwsens_dma_task);
	pwsens_state.status = PWSENS_COVERTING;
	pwsens_start_conversion();
}

void pwsens_dma_task_10ms(const struct timer_task *const timer_task)
{
	if (pwsens_state.status == PWSENS_COVERTING)
	{
		// Timeout, restart dma transfer
		adc_dma_disable_channel(&PWSENS_ADC, 0);
		adc_dma_set_conversion_mode(&PWSENS_ADC, ADC_CONVERSION_MODE_SINGLE_CONVERSION);
		system_throw_error(ERR_ABORTED);
		pwsens_state.status = PWSENS_COVERTING;
		pwsens_start_conversion();
	}
}

void pwsens_init(void)
{
	pwsens_dma_task.cb = &pwsens_dma_task_10ms;
	pwsens_dma_task.interval = 10; // 10 ms
	pwsens_dma_task.mode = TIMER_TASK_ONE_SHOT;

	adc_dma_set_conversion_mode(&PWSENS_ADC, ADC_CONVERSION_MODE_FREERUN);
	adc_dma_register_callback(&PWSENS_ADC, ADC_DMA_ERROR_CB, pwsens_error_cb);
	adc_dma_register_callback(&PWSENS_ADC, ADC_DMA_COMPLETE_CB, pwsens_convert_cb);
	adc_dma_set_inputs(&PWSENS_ADC, 0x04 /* AIN4 */, 0x00 /* AIN0 */, 0);
	pwsens_state.status = PWSENS_STOPPED;
	filter_state.pos = 0;
	pwsens_flags = 0;
}

void pwsens_set_config(uint8_t flags, uint8_t divider, uint16_t code, uint8_t repeat, uint8_t filter)
{
	// Do not update when running
	if (pwsens_state.status == PWSENS_STOPPED)
	{
		pwsens_flags = flags;
		pwsens_state.divider = divider;
		pwsens_state.code = code;

		if (filter > PWSENS_MAX_FILTER)
			filter = PWSENS_MAX_FILTER;
		pwsens_state.filter = filter;

		if (((flags & PWSENS_FLAGS_START) != 0))
		{
			// size check
			uint16_t min_samples = PWSENS_SAMPLE_STRIDE * CODE_SIZE * divider * (repeat == 0 ? 2 : (repeat + 1)) + PWSENS_CFILTER_OFFSET;
			if (min_samples > PWSENS_SAMPLE_COUNT)
				return;

			// calculations
			pwsens_state.repeat = repeat == 0 ? 1 : repeat;
			if ((flags & PWSENS_FLAGS_SYNC_MODE) == 0)
			{	// 1 steps per stride
				pwsens_state.cfilter_length = repeat >= 1 ?
					((PWSENS_SAMPLE_COUNT - PWSENS_CFILTER_OFFSET) / PWSENS_SAMPLE_STRIDE) - CODE_SIZE * divider * repeat :
					2 * CODE_SIZE * divider;
			}
			else
			{	// 3 steps per stride
				pwsens_state.cfilter_length = repeat >= 1 ?
					(PWSENS_SAMPLE_COUNT - PWSENS_CFILTER_OFFSET) - CODE_SIZE * divider * repeat * PWSENS_SAMPLE_STRIDE :
					2 * CODE_SIZE * PWSENS_SAMPLE_STRIDE * divider;
			}

			uplink_reset_measurement_counter();
			pwsens_set_sig_code(pwsens_state.code, (pwsens_flags & PWSENS_FLAGS_DIFFERENTIAL) != 0);
			pwsens_state.status = PWSENS_COVERTING;
			pwsens_start_conversion();
		}
	}
	else
	{
		// running, only update flags
		pwsens_flags = (pwsens_flags & (~PWSENS_FLAGS_RUNNING_UPDATE)) | (flags & PWSENS_FLAGS_RUNNING_UPDATE);
	}
}

int16_t calculate_filter(int16_t* buffer, uint8_t size)
{
	int32_t value = 0;
	for (uint8_t i = 0; i < size; i++)
	{
		value += buffer[i];
	}
	return value / size;
}

void pwsens_task(void)
{	
	if (pwsens_state.status == PWSENS_ANALYSE)
	{
		// reading buffer is ready, save and swap buffers
		pwgen_sample_type* ADC_buffer = ADC_buffer_reading;
		ADC_buffer_reading = ADC_buffer_anlysing;
		ADC_buffer_anlysing = ADC_buffer;

		// store config values
		int16_t mag[PWSENS_SAMPLE_STRIDE];
		uint8_t flags = pwsens_flags;
		int8_t divider = pwsens_state.divider;
		uint8_t repeat = pwsens_state.repeat;
		uint16_t cfilter_length = pwsens_state.cfilter_length;
		uint8_t filter = pwsens_state.filter;

		if (divider == 0)
		{
			int32_t sums[PWSENS_SAMPLE_STRIDE] = {0};
			for (uint16_t i = 0; i < PWSENS_SAMPLE_COUNT / PWSENS_SAMPLE_STRIDE; i++)
			{
				for (uint8_t j = 0; j < PWSENS_SAMPLE_STRIDE; j++)
				{
					int16_t value = ADC_buffer[i * PWSENS_SAMPLE_STRIDE + j];
					sums[j] += repeat == 1 ? value : value * value;
				}
			}
			for (uint8_t j = 0; j < PWSENS_SAMPLE_STRIDE; j++)
				mag[j] = sums[j] / (PWSENS_SAMPLE_COUNT / PWSENS_SAMPLE_STRIDE);
		}
		// matching filter
		else if ((flags & PWSENS_FLAGS_SYNC_MODE) == 0)
		{
			for (uint8_t channel = 0; channel < PWSENS_CHANNEL_COUNT; channel++)
			{
				if ((flags & (PWSENS_FLAGS_ENABLE_CH_A << channel)) == 0)
					continue;

				// apply filter to each channel individually.
				uint16_t quality;
				mag[channel] = corrFilter(pwsens_code_buffer, CODE_SIZE, divider, repeat,
										ADC_buffer + channel + PWSENS_CFILTER_OFFSET, cfilter_length,
										&quality);
			}  // for channel
		}
		else
		{
			// apply filter to all channels at once
			uint16_t pos;
			uint32_t norm = corrFilterSync(pwsens_code_buffer, CODE_SIZE, divider, repeat,
						ADC_buffer + PWSENS_CFILTER_OFFSET, cfilter_length,
						mag, &pos);
		}

		// start next measurement
		// TODO(michael): this should be possible before filter step (stpeed up!)
		// however th inputscan does not work reliably
		if ((flags & PWSENS_FLAGS_ENABLE) != 0 && (flags & PWSENS_FLAGS_CONTINUOUS) != 0)
		{
			pwsens_state.status = PWSENS_COVERTING;
			pwsens_start_conversion();
		}
		else
		{
			pwsens_state.status = PWSENS_STOPPED;
		}

		// low pass filter
		if (filter > 1)
		{
			// store value
			uint8_t pos = filter_state.pos;
			filter_state.buffer[0][pos] = mag[0];
			filter_state.buffer[1][pos] = mag[1];
			filter_state.buffer[2][pos] = mag[2];
			pos ++;
			if (pos >= filter)
			{
				pos = 0;
				int16_t value;
				value = calculate_filter(filter_state.buffer[channel_offset[0]], filter);
				uplink_set_channel(0, value, 0);
				value = calculate_filter(filter_state.buffer[channel_offset[1]], filter);
				uplink_set_channel(1, value, 0);
				value = calculate_filter(filter_state.buffer[channel_offset[2]], filter);
				uplink_set_channel(2, value, 0);
			}
			filter_state.pos = pos;
		}
		else
		{
			uplink_set_channel(0, mag[channel_offset[0]], 0);
			uplink_set_channel(1, mag[channel_offset[1]], 0);
			uplink_set_channel(2, mag[channel_offset[2]], 0);
		}
		uplink_increment_measurement_counter();
	}
	else if (pwsens_state.status == PWSENS_COVERTING)
	{
		// TODO(michael): check timeout (DMA failure)
	}
}
