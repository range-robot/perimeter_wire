
#ifndef BOARD_H_
#define BOARD_H_

#define FIRMWARE_VERSION (0x0001)

// Use interal 8MHz calibrated oscillator
// CKSEL = 0100
#define F_CPU	8000000

// config for system clock and scheduler (time slice is 1 ms)
#define CLOCK_PRESCALER 64
#define CLOCK_MAX (F_CPU / CLOCK_PRESCALER / 1000)

#define TRUE 1
#define FALSE 0

#endif /* BOARD_H_ */
