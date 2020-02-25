
#ifndef __DRIVERS_LEDS_H_
#define __DRIVERS_LEDS_H_

#include "config/board.h"
#include "hal_timer.h"

void LED_init(void);
void LED_task_100ms(const struct timer_task *const timer_task);

typedef enum {
	LM_OFF = 0,
	LM_PULSE_1 = 1,
	LM_PULSE_2 = 2,
	LM_PULSE_3 = 3,
	LM_PULSE_4 = 4,
	LM_PULSE_5 = 5,
	LM_BLINK = 5,
	LM_ON = 5
} LedPulse;

typedef enum {
	LC_Off = 0,
	LC_Red = 0x1,
	LC_Green = 0x2,
	LC_Blue = 0x4,
	LC_White = LC_Red | LC_Green | LC_Blue
} LedColor;

typedef struct {
	LedColor color;
	LedPulse pulse;
} LedMode;
LedMode LED_mode_A;
LedMode LED_mode_B;

#endif