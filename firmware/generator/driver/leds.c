
#include <stdint.h>
#include <stdbool.h>
#include "atmel_start_pins.h"
#include "leds.h"

#define LED_ON_LEVEL false
#define LED_OFF_LEVEL true
uint8_t pulse_counter;

void LED_init(void)
{
	pulse_counter = 0;	
}

static inline void LED_A_set_color(LedColor color)
{
	gpio_set_pin_level(LED1_R, ((color & LC_Red) != 0) ? LED_ON_LEVEL : LED_OFF_LEVEL);
	gpio_set_pin_level(LED1_G, ((color & LC_Green) != 0) ? LED_ON_LEVEL : LED_OFF_LEVEL);
	gpio_set_pin_level(LED1_B, ((color & LC_Blue) != 0) ? LED_ON_LEVEL : LED_OFF_LEVEL);
}

static inline void LED_B_set_color(LedColor color)
{
	gpio_set_pin_level(LED2_R, ((color & LC_Red) != 0) ? LED_ON_LEVEL : LED_OFF_LEVEL);
	gpio_set_pin_level(LED2_G, ((color & LC_Green) != 0) ? LED_ON_LEVEL : LED_OFF_LEVEL);
	gpio_set_pin_level(LED2_B, ((color & LC_Blue) != 0) ? LED_ON_LEVEL : LED_OFF_LEVEL);
}

void LED_task_100ms(const struct timer_task *const timer_task)
{
	static uint8_t counter = 0;
	if (counter == 0)
	{
		pulse_counter++;
		if (pulse_counter >= LM_BLINK)
			pulse_counter = 0;

		if (LED_mode_A.pulse == LM_ON)
			LED_A_set_color(LED_mode_A.color);
		else
			LED_A_set_color(LC_Off);

		if (LED_mode_B.pulse == LM_ON)
			LED_B_set_color(LED_mode_B.color);
		else
			LED_B_set_color(LC_Off);
	}
	
	if (counter == 5)
	{
		if (pulse_counter < LED_mode_A.pulse)
			LED_A_set_color(LED_mode_A.color);
		if (pulse_counter < LED_mode_B.pulse)
			LED_B_set_color(LED_mode_B.color);
	}
	
	counter ++;
	if (counter > 5)
		counter = 0;
		
}
