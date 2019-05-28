/**
 * \file
 *
 * \brief
 *
 * \copyright Copyright (c) 2014 Stoll technologies. All rights reserved.
 */

/*
 * timer.h
 *
 * Created: 12.05.2014 10:44:58
 *  Author: Michael Stoll
 */ 


#ifndef SYSTEM_TIMER_H_
#define SYSTEM_TIMER_H_

#include <avr/io.h>
/*
 * Clock select values for timer 
 * Ref: AT90CAN p.111 / ATmega32U4 p.105 / ATmega64M1 p.94
 */
enum TimerClockSelect {
	TCS0_Stopped = 0x00,
	TCS0_CLK_1 = 0x01,
	TCS0_CLK_8 = 0x02,
	TCS0_CLK_64 = 0x03,
	TCS0_CLK_256 = 0x04,
	TCS0_CLK_1024 = 0x05,
	TCS0_EXT_HL = 0x06,
	TCS0_EXT_LH = 0x07,

	TCS1_Stopped = 0x00,
	TCS1_CLK_1 = 0x01,
	TCS1_CLK_8 = 0x02,
	TCS1_CLK_64 = 0x03,
	TCS1_CLK_256 = 0x04,
	TCS1_CLK_1024 = 0x05,
	TCS1_EXT_HL = 0x06,
	TCS1_EXT_LH = 0x07,

	TCS2_Stopped = 0x00,
	TCS2_CLK_1 = 0x01,
	TCS2_CLK_8 = 0x02,
	TCS2_CLK_32 = 0x03,
	TCS2_CLK_64 = 0x04,
	TCS2_CLK_128 = 0x05,
	TCS2_CLK_256 = 0x06,
	TCS2_CLK_1024 = 0x07
};


#define TIMER_TOKENPASTE(x, y) x ## y
#define TIMER0_PRESCALER_TO_FLAG(ps) (TIMER_TOKENPASTE(TCS0_CLK_ ,ps))
#define TIMER1_PRESCALER_TO_FLAG(ps) (TIMER_TOKENPASTE(TCS1_CLK_ ,ps))
#define TIMER2_PRESCALER_TO_FLAG(ps) (TIMER_TOKENPASTE(TCS2_CLK_ ,ps))

/*
 * Waveform generation modes
 * Ref: 
 * * AT90CAN Table 12-1. p.110 
 * * ATmega32U4 Table 13-8 p.104
 * * ATmega64M1 p.93
 * * Atmega328p p.140
 */
enum TimerWaveformGenerationModes {
#if defined (__AVR_ATmega32U4__) || defined (__AVR_ATmega64M1__) || defined (__AVR_AT90CAN128__) || defined (__AVR_ATmega328P__)
	TWGM0_Normal = 0x00,
	TWGM0_PWM_0xFF = 0x01,
	/*
	 * Clear Timer on Compare Match
	 */
	TWGM0_CTC_OCR0A = 0x02,
	TWGM0_FastPWM_0xFF = 0x03,

	TWGM1_Normal = 0x00,
	/*
	 * Clear Timer on Compare Match
	 */
	TWGM1_CTC_OCR1A = 0x04,	
#else
#error Not supported
#endif
};

/*
 * Compare output mode
 * Ref: AT90CAN p.110 / ATmega32U4 p.103
 */
enum TimerCompareOutputMode {
	TCOM_Disconnected = 0x00,
	TCOM_Toggle = 0x01,
	TCOM_Clear = 0x02,
	TCOM_NonInverted = TCOM_Clear,
	TCOM_Set = 0x03,
	TCOM_Inverted = TCOM_Set
};



static inline void timer0_set_prescaler(uint8_t cs)  
{
	#if defined (__AVR_ATmega32U4__) || defined (__AVR_ATmega64M1__) || defined (__AVR_ATmega328P__)
	TCCR0B = (TCCR0B & ~(1<<CS02 | 1<<CS01 | 1<<CS00)) | ((cs & 0x07) << CS00); 
	#elif defined (__AVR_AT90CAN128__)
	TCCR0A = (TCCR0A & ~(1<<CS02 | 1<<CS01 | 1<<CS00)) | ((cs & 0x07) << CS00);
	#else
	#error "not supported"
	#endif
};

static inline void timer0_set_wgm(uint8_t wgm)
{
	#if defined (__AVR_ATmega32U4__) || defined (__AVR_ATmega64M1__) || defined (__AVR_ATmega328P__)
	TCCR0A = (TCCR0A & ~(1<<WGM01 | 1<<WGM00)) | ((wgm & 0x03) << WGM00);
	TCCR0B = (TCCR0B & ~(1<<WGM02)) | ((wgm & 0x04) << (WGM02 - 2));
	#elif defined (__AVR_AT90CAN128__)
	TCCR0A = (TCCR0A & ~(1<<WGM01 | 1<<WGM00) | ((wgm & 0x01) << WGM00) | ((wgm & 0x02) << (WGM01 - 1));
	#else
	#error "not supported"
	#endif
};

#ifdef OCR0A
static inline void timer0_set_comA(uint8_t com)  
{
	TCCR0A = (TCCR0A & ~(1 << COM0A0 | 1 << COM0A1)) | ((com & 0x3) << COM0A0);
}
static inline void timer0_enable_interrupt_ocA(void) {
	TIMSK0 |= (1<<OCIE0A);	
}
#endif

#ifdef OCR0B
static inline void timer0_set_comB(uint8_t com)
{
	TCCR0A = (TCCR0A & ~(1<<COM0B0 | 1<<COM0B1)) | ((com & 0x3) << COM0B0);	
}
#endif


static inline void timer1_set_prescaler(uint8_t cs)
{
#if defined (__AVR_ATmega32U4__) || defined (__AVR_ATmega328P__)
	TCCR1B = (TCCR1B & ~(1<<CS12 | 1<<CS11 | 1<<CS10)) | ((cs & 0x07) << CS10);
#else
#error "not supported"
#endif
};
static inline void timer1_set_wgm(uint8_t wgm)
{
#if defined (__AVR_ATmega32U4__) || defined (__AVR_ATmega328P__)
	TCCR1A = (TCCR1A & ~(1<<WGM11 | 1<<WGM10)) | ((wgm & 0x03) << WGM10);
	TCCR1B = (TCCR1B & ~(1<<WGM13 | 1<<WGM12)) | ((wgm & 0x0C) << (WGM12 - 2));
#else
#error "not supported"
#endif
};

static inline void timer1_set_comA(uint8_t com)
{
	TCCR1A = (TCCR1A & ~(1<<COM1A0 | 1<<COM1A1)) | ((com & 0x3) << COM1A0);
}

static inline uint16_t timer1_get_value(void) {
#ifdef TCNT1 
	return TCNT1;
#else
	#error "not supported"
#endif
}

static inline void timer1_set_value(uint16_t value) {
#ifdef TCNT1
	TCNT1 = value;
#else
	#error "not supported"
#endif
}

static inline void timer1_enable_interrupt_ovf(void) {
	TIMSK1 |= (1<<TOIE1);
}

#if defined (__AVR_ATmega328P__)
static inline void timer2_set_prescaler(uint8_t cs)
{
	TCCR2B = (TCCR2B & ~(1<<CS22 | 1<<CS21 | 1<<CS20)) | ((cs & 0x07) << CS20);	
};
#endif

#endif /* TIMER_H_ */