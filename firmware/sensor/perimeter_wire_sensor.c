#include "perimeter_wire_sensor.h"
#include "uplink.h"
#include <hal_adc_dma.h>
#include <hpl_adc_config.h>
#include <stdlib.h>

/*! The buffer size for ADC */
typedef int16_t pwgen_sample_type;
static pwgen_sample_type ADC_buffer[PWSENS_SAMPLE_COUNT];
int8_t sigcode[] = { 1,-1, 1,-1, 1,-1, 1,-1, 1,-1, 1,-1, 1,-1, 1,-1, 1,-1, 1,-1, 1,-1, 1,-1 };

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
	uint16_t mag[PWSENS_CHANNEL_COUNT];
} pwsens_state;


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
	pwsens_state.channel = 0;
	pwsens_state.status = PWSENS_STOPPED;
	pwsens_enable = 0;
}

void pwsens_set_enable(uint8_t enable)
{
	pwsens_enable = enable;
	if (enable != 0 && pwsens_state.status == PWSENS_STOPPED)
	{
		pwsens_start_conversion();
	}
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
	static int i = 0;
	if (pwsens_state.status == PWSENS_ANALYSE)
	{
		// oversampling factor
		int8_t subSample = 5;
		
		uint16_t quality;
		// magnitude for tracking (fast but inaccurate)
		int16_t sigcode_size = sizeof sigcode;
		
		// center
		int16_t center = 1 << 11;
		for (int i = 0; i < PWSENS_SAMPLE_COUNT; i++)
		{
			ADC_buffer[i] = (ADC_buffer[i] - center) >> 4;
		}
		int16_t mag = corrFilter(sigcode, subSample, sigcode_size, ADC_buffer, PWSENS_SAMPLE_COUNT-sigcode_size*subSample, &quality);
		pwsens_state.mag[pwsens_state.channel] = abs(mag);

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
		
		uplink_set_channel_a(pwsens_state.mag[0]);
		uplink_set_channel_b(quality);
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