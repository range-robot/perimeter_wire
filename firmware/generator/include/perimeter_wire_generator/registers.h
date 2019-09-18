
#ifndef PERIMETER_WIRE_GENERATOR_REGISTERS_H_
#define PERIMETER_WIRE_GENERATOR_REGISTERS_H_

#include <inttypes.h>

enum {
	REGISTER_CONTROL = 0x00,
	REGISTER_RES1 = 0x01,
	REGISTER_A_MODE = 0x02,
	REGISTER_A_FREQ_L = 0x04,
	REGISTER_A_FREQ_H = 0x05,
	REGISTER_A_FREQ = REGISTER_A_FREQ_L,
	REGISTER_B_MODE = 0x06,
	REGISTER_B_FREQ_L = 0x08,
	REGISTER_B_FREQ_H = 0x09,
	REGISTER_B_FREQ = REGISTER_B_FREQ_L,
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
	
	struct {
		uint8_t mode;
		uint8_t res;
		regu16_t frequency;
	} channel_a;
	
	struct {
		uint8_t mode;
		uint8_t res;
		regu16_t frequency;
	} channel_b;
};

enum {
	COMMAND_COUNT = 0
};
#endif /* PERIMETER_WIRE_FIRMWARE_CONFIG_H_ */