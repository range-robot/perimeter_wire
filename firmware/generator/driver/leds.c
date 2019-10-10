
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

void LED_task_100ms()
{
	static uint8_t counter = 0;
	if (counter == 0)
	{
		pulse_counter++;
		if (pulse_counter >= LM_BLINK)
			pulse_counter = 0;

		gpio_set_pin_level(LED_A, LED_mode_A == LM_ON ? LED_ON_LEVEL : LED_OFF_LEVEL);
		gpio_set_pin_level(LED_B, LED_mode_B == LM_ON ? LED_ON_LEVEL : LED_OFF_LEVEL);
	}
	
	if (counter == 5)
	{
		if (pulse_counter < LED_mode_A)
			gpio_set_pin_level(LED_A, LED_ON_LEVEL);
		if (pulse_counter < LED_mode_B)
			gpio_set_pin_level(LED_B, LED_ON_LEVEL);
	}
	
	counter ++;
	if (counter > 5)
		counter = 0;
		
}
