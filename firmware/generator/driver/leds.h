
#ifndef __DRIVERS_LEDS_H_
#define __DRIVERS_LEDS_H_

#include "config/board.h"

void LED_init(void);
void LED_task_100ms(void);

typedef enum {
	LM_OFF = 0,
	LM_PULSE_1 = 1,
	LM_PULSE_2 = 2,
	LM_PULSE_3 = 3,
	LM_PULSE_4 = 4,
	LM_PULSE_5 = 5,
	LM_BLINK = 5,
	LM_ON = 5
} LedModes;

LedModes LED_mode_A;
LedModes LED_mode_B;

#endif