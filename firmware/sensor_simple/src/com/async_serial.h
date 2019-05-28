/*
 * uart.h
 *
 * Created: 31.07.2017 08:48:09
 *  Author: Michael
 */ 


#ifndef SERIAL_H_
#define SERIAL_H_

#include <avr/io.h>
#include "../board.h"

/* CONFIG */
#define BAUD 9600
#define ASYNC_SERIAL_TX_BUFFER_SIZE 32
#define ASYNC_SERIAL_RX_BUFFER_SIZE 8


/* Init */
#include <util/setbaud.h>
#include <stdio.h>
#include "../lib/circular_buffer.h"
#include "../error.h"

enum parity_t {
	PARITY_DISABLED = 0,
	PARITY_EVEN = (1 << UPM01),
	PARITY_ODD = (1 << UPM01) | (1 << UPM00)
};

enum stopbit_t {
	STOPBIT_1BIT = 0,
	STOPBIT_2BIT = (1<<USBS0)
};

CIRCULAR_BUFFER_NAMED_STRUCT(async_serial_tx_buffer, ASYNC_SERIAL_TX_BUFFER_SIZE);
CIRCULAR_BUFFER_NAMED_STRUCT(async_serial_rx_buffer, ASYNC_SERIAL_RX_BUFFER_SIZE);

extern struct async_serial_tx_buffer circular_buffer_async_serial_tx;
extern struct async_serial_rx_buffer circular_buffer_async_serial_rx;

static inline void async_serial_init(enum stopbit_t stopBits, enum parity_t parity) {
	/* set baudrate (values from set_baudrate.h) */
	UBRR0L = UBRRL_VALUE;
	UBRR0H = UBRRH_VALUE;

	/* disable U2X-Mode */
	UCSR0A &= ~(1 << U2X0);

	/* set async mode */
	const uint8_t charsize = 8;
	UCSR0C = /* ~UMSEL1 & ~UMSEL0 | */ parity | stopBits | (((charsize - 5) & 0x3) << UCSZ00);

	/* enable tx, disable interrupts */
	UCSR0B = (((charsize - 8) & 0x01) << UCSZ02) | (1 << TXEN0) | (1 << RXEN0) | (1 << RXCIE0);
}

/* 
 * Transmit data
*/

static void async_serial_put( const uint8_t data )
{
	uint8_t wresult;
	CIRCULAR_BUFFER_WRITE(circular_buffer_async_serial_tx, ASYNC_SERIAL_TX_BUFFER_SIZE, data, wresult);
	if (!wresult) {
		system_throw_error(ERROR_BUFFER_OVERFLOW);
		return;
	}
	
	// begin send mode
	UCSR0B |= (1<< UDRIE0);
}



/*
 * Receive data
*/
static int16_t async_serial_get( void )
{
	if (!CIRCULAR_BUFFER_IS_EMPTY(circular_buffer_async_serial_rx.header)) {
		uint8_t result;
		CIRCULAR_BUFFER_READ(circular_buffer_async_serial_rx, ASYNC_SERIAL_RX_BUFFER_SIZE, result);
		return result;
	}
	else 
		return EOF;
}

#endif /* SERIAL_H_ */