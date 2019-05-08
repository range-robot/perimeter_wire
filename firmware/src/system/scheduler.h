
#ifndef SYSTEM_SCHEDULER_H_
#define SYSTEM_SCHEDULER_H_

// depends on clock timer.
// time slice length can be changed by CLOCK_MAX
#include <system/clock.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>

struct scheduler_state {
	volatile uint8_t flag:1;
};

struct scheduler_state scheduler_state_;

static inline void scheduler_init(void) {
	clock_init(true);
	scheduler_state_.flag = FALSE;
}

static inline uint8_t scheduler_next_slice(void) {
	if (scheduler_state_.flag) {
		scheduler_state_.flag = FALSE;
		return TRUE;
	}
	return FALSE;
}

static inline uint8_t scheduler_end_slice(uint8_t sleep) {
	// Warning: RACE CONDITION
	// A interrupt setting flas_1ms can happen anywhere in this method
	// sleeping in that case would cause an overslept slice.

	if (sleep)  {
		// NOTE: when adding a condition, see sleep.h
		set_sleep_mode(SLEEP_MODE_IDLE);

		// check flag (keep RACE CONDITION short)
		if (scheduler_state_.flag)
			return FALSE;
			
		sleep_mode();
	}
	else {
		if (scheduler_state_.flag)
			return FALSE;
	}

	return TRUE;
}

#if defined (TIMER0_COMPA_vect)
ISR(TIMER0_COMPA_vect) {
	scheduler_state_.flag = TRUE;
}
#elif defined (TIMER0_COMP_vect)
ISR(TIMER0_COMP_vect) {
	scheduler_state_.flag = TRUE;
}
#else
#error not supported
#endif

#endif /* SYSTEM_H_ */