
#ifndef CLOCK_H_
#define CLOCK_H_

#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>
#include <board.h>
#include <system/timer.h>

#ifndef CLOCK_PRESCALER
#error clock not configured
#endif 
#ifndef CLOCK_MAX
#error clock not configured
#endif
/* 
 * example config: (max == 1ms)
 * #define CLOCK_PRESCALER 64
 * #define CLOCK_MAX (F_CPU / CLOCK_PRESCALER / 1000)
 */
#define CLOCK_US (1000000 / (F_CPU / CLOCK_PRESCALER))


static inline void clock_init(bool enable_interrupt) {
	// Setup scheduler
	// Timer 0 initialisieren, CTC
	OCR0A = CLOCK_MAX;
	if (enable_interrupt)
		timer0_enable_interrupt_ocA();
	timer0_set_wgm(TWGM0_CTC_OCR0A);
	timer0_set_prescaler(TIMER0_PRESCALER_TO_FLAG(CLOCK_PRESCALER));
	
}

/* 
 * get clock value. 
 * Use clock_get() * CLOCK_US to get microseconds
 * resolution depends on F_CPU
 */
static inline uint8_t clock_get(void) {
	return TCNT0;
}

#define clock_diff(start, end, delta) delta = end - start; if (start > end) delta += CLOCK_MAX;

#endif /* SYSTEM_H_ */