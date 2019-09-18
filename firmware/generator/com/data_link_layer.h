/*
 * Data link layer
 * Version 25.09.2019
 *
 * 25.09.2019: Buffer out messages
 */

#ifndef DATA_LINK_LAYER_H_
#define DATA_LINK_LAYER_H_

/*
 * Handles
 * Stream to Message
 * Error checking
 */

#include <stdio.h>
#include <stdint.h>
#include "error.h"

// Ref: http://eli.thegreenplace.net/2009/08/12/framing-in-serial-communications/
/*
 * Required configuration:
 * #define DATALINK_START_FLAG (0x7E)
 * #define DATALINK_END_FLAG (0x7E)
 * #define DATALINK_ESC (0x7D)
 * #define DATALINK_ESC_MASK (0x20)
 * #define DATALINK_CHECKSUM ODD
 * #define DATALINK_RECEIVE_BUFFER_SIZE (8) 
 * #define DATALINK_READ() async_serial_get()
 * #define DATALINK_WRITE(ch) async_serial_put(ch)
 */
#include "data_link_layer_config.h"


struct datalink_state {
	// Buffers must be aligned!
	uint8_t send_buffer[DATALINK_SEND_BUFFER_SIZE];
	uint8_t receive_buffer[DATALINK_RECEIVE_BUFFER_SIZE];
	uint8_t send_checksum;
	uint8_t send_buffer_length;
	uint8_t receive_checksum;
	uint8_t receive_state;
	uint8_t receive_buffer_length;
};
struct datalink_state datalink_state;


enum datalink_receive_states {
	DATALINK_STATE_IDLE,
	DATALINK_STATE_MESSAGE,
	DATALINK_STATE_ESC
};

static inline void datalink_init() {
	datalink_state.receive_buffer_length = 0;
	datalink_state.receive_state = DATALINK_STATE_IDLE;
}

static inline void datalink_begin_message() {
	datalink_state.send_checksum = 0;
	datalink_state.send_buffer[0] = DATALINK_START_FLAG;
	datalink_state.send_buffer_length = 1;
}



static inline void _datalink_put_message_char(const uint8_t c, uint8_t* buffer, uint8_t* len, uint8_t* checksum) {
	*checksum ^= c;
	/*if (*len >= DATALINK_SEND_BUFFER_SIZE - 2) {
		system_throw_error(ERR_OVERFLOW);
		return;
	}*/
	if (c == DATALINK_START_FLAG ||
		c == DATALINK_END_FLAG ||
		c == DATALINK_ESC) {
		buffer[(*len)++] = DATALINK_ESC;
		buffer[(*len)++] = c ^ DATALINK_ESC_MASK;
	} else {
		buffer[(*len)++] = c;
	}
}

static inline void datalink_put_message_char(const uint8_t c) {
	uint8_t* buffer = datalink_state.send_buffer;
	uint8_t len = datalink_state.send_buffer_length;
	uint8_t checksum = datalink_state.send_checksum;
	_datalink_put_message_char(c, buffer, &len, &checksum);
	datalink_state.send_buffer_length = len;
	datalink_state.send_checksum = checksum;
}

static inline void datalink_put_message_data(const uint8_t* data, uint8_t count) {
	uint8_t* buffer = datalink_state.send_buffer;
	uint8_t len = datalink_state.send_buffer_length;
	uint8_t checksum = datalink_state.send_checksum;
	for (uint8_t i = 0; i < count; i++) {
		_datalink_put_message_char(data[i], buffer, &len, &checksum);
	}
	datalink_state.send_buffer_length = len;
	datalink_state.send_checksum = checksum;
}

static inline void datalink_end_message() {
	uint8_t* buffer = datalink_state.send_buffer;
	uint8_t len = datalink_state.send_buffer_length;
	uint8_t checksum = datalink_state.send_checksum;
	_datalink_put_message_char(checksum, buffer, &len, &checksum);
	datalink_state.send_buffer[len++] = DATALINK_END_FLAG;
	
	DATALINK_WRITE(buffer, len);
}

static inline uint8_t datalink_put_message(const uint8_t* data, uint8_t len) {
	datalink_begin_message();
	datalink_put_message_data(data, len);
	datalink_end_message();
	return 0;
}

static inline uint8_t datalink_get_message(void) {
	int ic;
	uint8_t c;

	while (1) {
		ic = DATALINK_READ();
		if (ic == EOF)
			return 0;
		c = (uint8_t)ic;
	
		switch (datalink_state.receive_state) {
			case DATALINK_STATE_IDLE:
				if (c == DATALINK_START_FLAG) {
					// reset receive buffer
					datalink_state.receive_buffer_length = 0;
					datalink_state.receive_checksum = 0;
					datalink_state.receive_state = DATALINK_STATE_MESSAGE;
				} else {
					system_throw_error(ERR_PROTOCOL);
				}
				break;
			case DATALINK_STATE_MESSAGE:
				if (c == DATALINK_END_FLAG
#if DATALINK_END_FLAG == DATALINK_START_FLAG
					&& datalink_state.receive_buffer_length != 0
#endif
				) {
					if (datalink_state.receive_checksum == 0) {
						datalink_state.receive_state = DATALINK_STATE_IDLE;
						return datalink_state.receive_buffer_length -1;
					} else {
						system_throw_error(ERR_INVALID_DATA);
					}
				} else if (c == DATALINK_START_FLAG) {
					// reset receive buffer
					datalink_state.receive_buffer_length = 0;
					datalink_state.receive_checksum = 0;
					datalink_state.receive_state = DATALINK_STATE_MESSAGE;
				} else if (c == DATALINK_ESC) {
					datalink_state.receive_state = DATALINK_STATE_ESC;
				} else {
					datalink_state.receive_checksum ^= c;
					datalink_state.receive_buffer[datalink_state.receive_buffer_length] = c;
					datalink_state.receive_buffer_length++;
				}
				break;
			case DATALINK_STATE_ESC:
#if DATALINK_ESC_MASK != 0
				if (c == DATALINK_START_FLAG) {
					// This makes resynchronization quicker.
					// reset receive buffer
					datalink_state.receive_checksum = 0;
					datalink_state.receive_buffer_length = 0;
					datalink_state.receive_state = DATALINK_STATE_MESSAGE;
				} else {
#else
				{
#endif
					c = DATALINK_ESC_MASK ^ c;
					datalink_state.receive_checksum ^= c;
					datalink_state.receive_buffer[datalink_state.receive_buffer_length] = c;
					datalink_state.receive_buffer_length++;
					datalink_state.receive_state = DATALINK_STATE_MESSAGE;
				}
				break;
		}
	}
}

#endif /* DATA_LINK_LAYER_H_ */