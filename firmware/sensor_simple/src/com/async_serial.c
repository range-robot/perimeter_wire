/*
 * async_serial.c
 *
 * Created: 19.08.2017 21:00:55
 *  Author: Michael
 */ 

#include <avr/interrupt.h>
#include "async_serial.h"

struct async_serial_tx_buffer circular_buffer_async_serial_tx = CIRCULAR_BUFFER_INIT;
struct async_serial_rx_buffer circular_buffer_async_serial_rx = CIRCULAR_BUFFER_INIT;

ISR(USART_UDRE_vect) {
	struct circular_buffer_header header = circular_buffer_async_serial_tx.header;
	
	if (CIRCULAR_BUFFER_IS_EMPTY((header)))
	{
		// stop send mode
		UCSR0B &= ~(1<< UDRIE0);
		return;
	}

	// If there is data in the transmit buffer, then send
	while (! CIRCULAR_BUFFER_IS_EMPTY((header)) && UCSR0A & (1<<UDRE0))
	{
		 CIRCULAR_BUFFER_READ2(circular_buffer_async_serial_tx.buffer, header, ASYNC_SERIAL_TX_BUFFER_SIZE, UDR0);
	}
	circular_buffer_async_serial_tx.header = header;
}

 
ISR(USART_RX_vect) {
	// copy receive register to buffer
	struct circular_buffer_header header = circular_buffer_async_serial_rx.header;
	uint8_t wresult;
	while ((UCSR0A & (1<<RXC0))) {
		uint8_t value = UDR0;
		CIRCULAR_BUFFER_WRITE2(circular_buffer_async_serial_rx.buffer, header, ASYNC_SERIAL_RX_BUFFER_SIZE, value, wresult);
		if (!wresult) {
			system_throw_error(ERROR_BUFFER_OVERFLOW);
		}
	}
	circular_buffer_async_serial_rx.header = header;
}
