#ifndef PERIMETER_WIRE_SENSOR_H_
#define PERIMETER_WIRE_SENSOR_H_

#include <stdint.h>

/* CONFIG */
/*! The buffer size for ADC */
typedef int16_t pwgen_sample_type;
#define PWSENS_ADC ADC_0
#define PWSENS_CHANNEL_COUNT (3)
#define PWSENS_SAMPLE_COUNT (768)
#define PWSENS_SAMPLE_STRIDE (3)
#define PWSENS_MAX_FILTER (32)
// skip first measurements
#define PWSENS_CFILTER_OFFSET (3)
/* END CONFIG */


void pwsens_init(void);
void pwsens_task(void);
void pwsens_set_config(uint8_t flags, uint8_t divider, uint16_t code, uint8_t repeat, uint8_t filter);


// buffer access
uint16_t pwsens_get_buffer_length(void);
uint16_t pwsens_get_buffer_value(uint16_t index);
#endif /* PERIMETER_WIRE_SENSOR_H_ */