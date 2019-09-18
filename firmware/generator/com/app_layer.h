/*
 * protocol.h
 *
 * Created: 03.08.2017 15:16:37
 *  Author: Michael
 */ 


#ifndef APP_LAYER_H_
#define APP_LAYER_H_

#include "app_layer_config.h"
#include <perimeter_wire_generator/messages.h>
#include "../com/data_link_layer.h"
#include "board.h"
#include "error.h"

// REQUIRED Macros:
// APP_LAYER_MAX_REGISTER
// APP_LAYER_MAX_COMMAND
// APP_LAYER_REGISTERS
// APP_RESET()
// Example config:
// #define APP_LAYER_MAX_REGISTER 10
// uint8_t registers[APP_LAYER_MAX_REGISTER];
// #define APP_LAYER_REGISTERS registers
// #define APP_LAYER_MAX_COMMAND 10
// const void (*)() cmds = { &cb, ..};
// #define APP_LAYER_COMMANDS cmds

extern uint8_t APP_LAYER_REGISTERS[];
extern void (* const APP_LAYER_COMMANDS[])(void);


void app_layer_send_message(uint8_t header, const uint8_t* data_buffer, uint8_t len);

static inline void app_layer_send_hello_message(uint16_t version) {
	app_layer_send_message(MESSAGE_HELLO, (uint8_t*)&version, sizeof(version));
}

static inline void app_layer_send_error_message(uint8_t error) {
	app_layer_send_message(MESSAGE_ERROR, &error, sizeof(error));
}

static inline void app_layer_send_version_message(uint16_t version) {
	app_layer_send_message(MESSAGE_VERSION, (uint8_t*)&version, sizeof(version));
}

static inline void app_layer_send_command_result_message(uint8_t cmd, uint8_t result) {
	uint8_t buffer[2] = {cmd, result};
	app_layer_send_message(MESSAGE_COMMAND_RESULT, buffer, 2);
}

static inline void app_layer_handle_get_version(uint8_t* message, uint8_t len) {
	if (len != 1) {
		system_throw_error(ERR_PROTOCOL);
	}
	else {
		app_layer_send_version_message(FIRMWARE_VERSION);		
	}
}

static inline void app_layer_handle_reset(uint8_t* message, uint8_t len) {
	if (len != 1) {
		system_throw_error(ERR_PROTOCOL);
	}
	else {
		APP_RESET();
	}
}

static inline void app_layer_handle_set_reg(uint8_t* message, uint8_t len) {
	if (len != 3) {
		system_throw_error(ERR_PROTOCOL);
	}
	else {
		uint8_t adr = message[1];
		uint8_t val = message[2];
		if (adr >= APP_LAYER_MAX_REGISTER)
			system_throw_error(ERR_BAD_ADDRESS);
		else {
			APP_LAYER_REGISTERS[adr] = val;
			APP_LAYER_SET_REG_HOOK(adr);
		}
	}
}

static inline void app_layer_handle_set_reg16(uint8_t* message, uint8_t len) {
	if (len != 4) {
		system_throw_error(ERR_PROTOCOL);
	}
	else {
		uint8_t adr = message[1];
		uint8_t val_h = message[2];
		uint8_t val_l = message[3];
		if (adr+1 >= APP_LAYER_MAX_REGISTER)
			system_throw_error(ERR_BAD_ADDRESS);
		else {
			APP_LAYER_REGISTERS[adr] = val_h;
			APP_LAYER_REGISTERS[adr +1] = val_l;
			APP_LAYER_SET_REG_HOOK(adr);
		}
	}
}

static inline void app_layer_handle_get_reg(uint8_t* message, uint8_t len) {
	if (len != 2) {
		system_throw_error(ERR_PROTOCOL);
	}
	else {
		uint8_t adr = message[1];
		if (adr >= APP_LAYER_MAX_REGISTER)
			system_throw_error(ERR_BAD_ADDRESS);
		else {
			uint8_t buffer[2] = {
				adr,
				APP_LAYER_REGISTERS[adr]
			};
			app_layer_send_message(MESSAGE_REG_VALUE, buffer, 2);
		}
	}
}

static inline void app_layer_handle_get_reg16(uint8_t* message, uint8_t len) {
	if (len != 2) {
		system_throw_error(ERR_PROTOCOL);
	}
	else {
		uint8_t adr = message[1];
		if (adr +1 >= APP_LAYER_MAX_REGISTER)
			system_throw_error(ERR_BAD_ADDRESS);
		else {
			uint8_t buffer[3] = {
				adr,
				APP_LAYER_REGISTERS[adr],
				APP_LAYER_REGISTERS[adr+1]
			};
			app_layer_send_message(MESSAGE_REG_VALUE_16, buffer, 3);
		}
	}
}

static inline void app_layer_handle_command(uint8_t* message, uint8_t len) {
	if (len != 2) {
		system_throw_error(ERR_PROTOCOL);
	}
	else {
		uint8_t adr = message[1];
		if (adr >= APP_LAYER_MAX_COMMAND)
			system_throw_error(ERR_BAD_ADDRESS);
		else {
			void (* const cmd)(void) = APP_LAYER_COMMANDS[adr];
			cmd();
		}
	}
}



static inline void app_layer_cycle(void) {
	uint8_t system_error = system_status;
	if (system_error != ERR_NONE) {
		app_layer_send_error_message(system_error);
	}
	system_status = ERR_NONE;


	uint8_t len = datalink_get_message();
	if (len > 0) {
		uint8_t* message = datalink_state.receive_buffer;
		uint8_t header = message[0];
		switch (header) {
			case MESSAGE_GET_VERSION:
				app_layer_handle_get_version(message, len);
				break;
			case MESSAGE_RESET:
				app_layer_handle_reset(message, len);
				break;
			case MESSAGE_SET_REG:
				app_layer_handle_set_reg(message, len);
				break;
			case MESSAGE_SET_REG_16:
				app_layer_handle_set_reg16(message, len);
				break;
			case MESSAGE_GET_REG:
				app_layer_handle_get_reg(message, len);
				break;
			case MESSAGE_GET_REG_16:
				app_layer_handle_get_reg16(message, len);
				break;
			case MESSAGE_COMMAND:
				app_layer_handle_command(message, len);
			break;
			default:
				system_throw_error(ERR_PROTOCOL);
				break;
		}
	}
	
}




#endif /* PROTOCOL_H_ */