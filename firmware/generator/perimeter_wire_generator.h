#ifndef PERIMETER_WIRE_GENERATOR_H_
#define PERIMETER_WIRE_GENERATOR_H_

#include <hal_timer.h>
#include "atmel_start_pins.h"

/* CONFIG */
#define PGWEN_TIMER_FREQUENCY 62500
#define PWGEN_ENABLE_A EN_A
#define PWGEN_IN1_A IN1_A
#define PWGEN_IN2_A IN2_A
#define PWGEN_ERROR_A ERROR_A
#define PWGEN_ENABLE_B EN_B
#define PWGEN_IN1_B IN1_B
#define PWGEN_IN2_B IN2_B
#define PWGEN_ERROR_B ERROR_B
/* END CONFIG */

enum pwgen_mode_t {
	PWGEN_MODE_DISABLED,
	PWGEN_MODE_H,
	PWGEN_MODE_SINGLE
};

enum pwgen_channel_t {
	PWGEN_A,
	PWGEN_B
};

struct pwgen_t {
	enum pwgen_channel_t channel;
	// pins
	uint32_t enable;
	uint32_t in1, in2;
	uint32_t error;
	
	// state
	enum {
		_PWGEN_STATE_IDLE,
		_PWGEN_STATE_RUN
	} state;
	enum pwgen_mode_t mode;
	struct timer_task task;
};

struct pwgen_config_t
{
	enum pwgen_mode_t mode;
	uint32_t frequency;	
};

void pwgen_init(struct pwgen_t* const gen, enum pwgen_channel_t channel);
void pwgen_set_config(struct pwgen_t* const gen, const struct pwgen_config_t* const config, struct timer_descriptor* const timer);

#endif /* PERIMETER_WIRE_GENERATOR_H_ */