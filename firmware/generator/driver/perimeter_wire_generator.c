#include "perimeter_wire_generator.h"
#include "leds.h"
void _pwgen_timer_tick(struct pwgen_t* const gen)
{
	switch (gen->mode)
	{
		case PWGEN_MODE_H:
			gpio_toggle_pin_level(gen->in1);
			gpio_toggle_pin_level(gen->in2);
			break;
		case PWGEN_MODE_SINGLE:
			gpio_toggle_pin_level(gen->in1);
		break;
			default:
		case PWGEN_MODE_DISABLED:
			break;
	}
}

struct pwgen_t *_pwgenA, *_pwgenB;

void _pwgenA_timer_cb(const struct timer_task *const timer_task)
{
	_pwgen_timer_tick(_pwgenA);
}
void _pwgenB_timer_cb(const struct timer_task *const timer_task)
{
	_pwgen_timer_tick(_pwgenB);
}

void pwgen_init(struct pwgen_t* const gen, enum pwgen_channel_t channel)
{
	gen->channel = channel;
	gen->state = _PWGEN_STATE_IDLE;
	gen->task.mode = TIMER_TASK_REPEAT;
	gen->task.time_label = 0;
	gen->task.interval = 500;
	switch (channel)
	{
		case PWGEN_A:
		gen->led = &LED_mode_A;
		gen->enable = PWGEN_ENABLE_A;
		gen->error = PWGEN_ERROR_A;
		gen->in1 = PWGEN_IN1_A;
		gen->in2 = PWGEN_IN2_A;
		_pwgenA = gen;
		gen->task.cb = _pwgenA_timer_cb;
		break;
		case PWGEN_B:
		gen->led = &LED_mode_B;
		gen->enable = PWGEN_ENABLE_B;
		gen->error = PWGEN_ERROR_B;
		gen->in1 = PWGEN_IN1_B;
		gen->in2 = PWGEN_IN2_B;
		_pwgenB = gen;
		gen->task.cb = _pwgenB_timer_cb;
		break;
	}
	gpio_set_pin_level(gen->enable, false);
	gpio_set_pin_level(gen->in1, false);
	gpio_set_pin_level(gen->in2, false);
}

void pwgen_set_config(struct pwgen_t* const gen, const struct pwgen_config_t* const config, struct timer_descriptor* const timer)
{
	gen->mode = config->mode;
	if (config->divider == 0)
		gen->mode = PWGEN_MODE_DISABLED;

	if (gen->mode == PWGEN_MODE_DISABLED)
	{
		if (gen->state != _PWGEN_STATE_IDLE)
		{
			gpio_set_pin_level(gen->enable, false);
			gpio_set_pin_level(gen->in1, false);
			gpio_set_pin_level(gen->in2, false);
			timer_remove_task(timer, &gen->task);
			gen->state = _PWGEN_STATE_IDLE;
		}
	}
	else
	{
		if (gen->state == _PWGEN_STATE_IDLE)
		{
			gpio_set_pin_level(gen->enable, true);
			gpio_set_pin_level(gen->in1, true);
			gpio_set_pin_level(gen->in2, false);
			gen->state = _PWGEN_STATE_RUN;
			
			gen->task.interval = config->divider;
			timer_add_task(timer, &gen->task);
		}
	}
}

static inline void pwgen_check(struct pwgen_t* const gen)
{
	if (gen->state == _PWGEN_STATE_IDLE)
	{
		*gen->led = LM_OFF;
	}
	else
	{
		if (gpio_get_pin_level(gen->error))
		{
			// No eror
			*gen->led = LM_ON;
		}
		else
		{
			// Overcurrent
			*gen->led = LM_PULSE_1;
		}
	}
}

void pwgen_task(void)
{
	pwgen_check(_pwgenA);
	pwgen_check(_pwgenB);
}