#ifndef SRC_MATCHING_FILTER_H
#define SRC_MATCHING_FILTER_H

#include <stdint.h>
#include "perimeter_wire_sensor.h"


static inline int16_t crossCorrelation(const int8_t* coeffs, uint16_t nCoeffs, const pwgen_sample_type* ipi, uint8_t repeat, uint8_t subsample)
{
	int16_t sum = 0;
	ASSERT(repeat >= 1);

	for (uint8_t ri = 0; ri < repeat; ri++)
	{
		// for each filter coeffs
		for (uint16_t i = 0; i < nCoeffs; i++)
		{
			int8_t coeff = coeffs[i];
			for (uint8_t j = 0; j < subsample; j++)
			{
				sum += coeff * (*ipi);
				ipi += PWSENS_SAMPLE_STRIDE;
			}
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

	// compute correlation
	// for each input value
	for (int16_t j=0; j<nPts; j++)
	{
		int16_t sum = crossCorrelation(coeffs, nCoeffs, ip, repeat, subsample);
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
	// compute correlation
	// for each input value
	int32_t sum[PWSENS_SAMPLE_STRIDE] = {0};
	int32_t sumSQ[PWSENS_SAMPLE_STRIDE] = {0};
	int32_t normMax = -1;
	uint16_t maxPos = 0;
	uint8_t offset = 0;
	for (int16_t j=0; j<nPts; j++)
	{
		if (offset >= PWSENS_SAMPLE_STRIDE)
			offset = 0;

		int32_t s = (int32_t) crossCorrelation(coeffs, nCoeffs, ip, repeat, subsample);
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

#endif  // SRC_MATCHING_FILTER_H