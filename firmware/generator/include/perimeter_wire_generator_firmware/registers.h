
#ifndef PERIMETER_WIRE_GENERATOR_REGISTERS_H_
#define PERIMETER_WIRE_GENERATOR_REGISTERS_H_

#include <inttypes.h>

enum {
	REGISTER_CONTROL = 0x00,
	REGISTER_A_MODE = 0x02,
	REGISTER_A_DIV = 0x03,
	REGISTER_A_CODE = 0x04,
	REGISTER_B_MODE = 0x06,
	REGISTER_B_DIV = 0x07,
	REGISTER_B_CODE = 0x08,
	REGISTER_VOLTAGE = 0x10,
	REGISTER_TEMP = 0x11,
	REGISTER_COUNT = 0x12,

	REGISTER_SAVE_START = REGISTER_A_MODE,
	REGISTER_SAVE_END = 0x0A
};
typedef union {
	uint16_t value;
	struct {
		uint8_t l;
		uint8_t h;
	};
} reg_uint16_t;

struct app_registers_t {
	union {
		uint8_t value;
		struct {
			uint8_t run : 1;
			uint8_t reserved : 6;
			uint8_t save : 1;
		};
	} control;

	uint8_t reserved1;

	struct {
		uint8_t mode;
		uint8_t divider;
		reg_uint16_t code;
	} channel_a;
	
	struct {
		uint8_t mode;
		uint8_t divider;
		reg_uint16_t code;
	} channel_b;

	uint8_t reserved2[6];
	uint8_t voltage;
	uint8_t temp;
	
};

enum {
	COMMAND_COUNT = 0
};
#endif /* PERIMETER_WIRE_FIRMWARE_CONFIG_H_ */