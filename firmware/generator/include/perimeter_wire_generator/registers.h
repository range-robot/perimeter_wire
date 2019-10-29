
#ifndef PERIMETER_WIRE_GENERATOR_REGISTERS_H_
#define PERIMETER_WIRE_GENERATOR_REGISTERS_H_

#include <inttypes.h>

enum {
	REGISTER_CONTROL = 0x00,
	REGISTER_TEMP = 0x01,
	REGISTER_A_MODE = 0x02,
	REGISTER_A_DIV = 0x04,
	REGISTER_B_MODE = 0x06,
	REGISTER_B_DIV = 0x08,
	REGISTER_COUNT = 0x0A
};
typedef union {
	uint16_t value;
	struct {
		uint8_t l;
		uint8_t h;
	};
} regu16_t;

struct app_registers_t {
	union {
		uint8_t value;
		struct {
			uint8_t run : 1;
			uint8_t reserved : 7;
		};
	} control;

	uint8_t temp;

	struct {
		uint8_t mode;
		uint8_t res1;
		uint8_t divider;
		uint8_t res2;
	} channel_a;
	
	struct {
		uint8_t mode;
		uint8_t res1;
		uint8_t divider;
		uint8_t res2;
	} channel_b;
};

enum {
	COMMAND_COUNT = 0
};
#endif /* PERIMETER_WIRE_FIRMWARE_CONFIG_H_ */