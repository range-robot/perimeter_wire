#ifndef PERIMETER_WIRE_SENSOR_H_
#define PERIMETER_WIRE_SENSOR_H_

#include "driver_init.h"

/* CONFIG */
#define PWSENS_ADC ADC_0
#define PWSENS_CHANNEL_COUNT (3)
#define PWSENS_SAMPLE_COUNT (768)
#define PWSENS_SAMPLE_STRIDE (3)
/* END CONFIG */


void pwsens_init(void);
void pwsens_task(void);
void pwsens_set_flags(uint8_t flags);
void pwsens_set_config(uint8_t channel, uint8_t divider, uint16_t code, uint8_t repeat);


// buffer access
uint16_t pwsens_get_buffer_length(void);
uint16_t pwsens_get_buffer_value(uint16_t index);
#endif /* PERIMETER_WIRE_SENSOR_H_ */