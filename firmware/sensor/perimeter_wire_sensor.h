#ifndef PERIMETER_WIRE_SENSOR_H_
#define PERIMETER_WIRE_SENSOR_H_

#include "driver_init.h"

/* CONFIG */
#define PWSENS_ADC ADC_0
#define PWSENS_CHANNEL_COUNT 4
#define PWSENS_SAMPLE_COUNT 256
#define PWSENS_SAMPLE_RATE (214330)
/* END CONFIG */


void pwsens_init(void);
void pwsens_task(void);
void pwsens_set_flags(uint8_t flags);
void pwsens_set_channel(uint8_t channel, uint8_t divider, uint16_t code, uint8_t repeat);

#endif /* PERIMETER_WIRE_SENSOR_H_ */