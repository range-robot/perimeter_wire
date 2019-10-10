#ifndef PERIMETER_WIRE_SENSOR_H_
#define PERIMETER_WIRE_SENSOR_H_

#include "driver_init.h"

/* CONFIG */
#define PWSENS_ADC ADC_0
#define PWSENS_CHANNEL_COUNT 1
#define PWSENS_SAMPLE_COUNT 256
#define PWSENS_SAMPLE_RATE (214330)
/* END CONFIG */


void pwsens_init(void);
void pwsens_task(void);
void pwsens_set_enable(uint8_t enable);

#endif /* PERIMETER_WIRE_SENSOR_H_ */