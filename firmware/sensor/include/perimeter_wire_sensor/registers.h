
#ifndef PERIMETER_WIRE_SENSOR_REGISTERS_H_
#define PERIMETER_WIRE_SENSOR_REGISTERS_H_

#include <inttypes.h>

enum {
	REGISTER_CONTROL = 0x00,
	REGISTER_ENABLED = 0x01,
	REGISTER_CHANNEL_A_DIVIDER = 0x04,
	REGISTER_CHANNEL_A_L = 0x06,
	REGISTER_CHANNEL_A_H = 0x07,
	REGISTER_CHANNEL_A = REGISTER_CHANNEL_A_L,
	REGISTER_CHANNEL_B_DIVIDER = 0x08,
	REGISTER_CHANNEL_B_L = 0x0A,
	REGISTER_CHANNEL_B_H = 0x0B,
	REGISTER_CHANNEL_B = REGISTER_CHANNEL_B_L,
	REGISTER_CHANNEL_C_DIVIDER = 0x0C,
	REGISTER_CHANNEL_C_L = 0x0E,
	REGISTER_CHANNEL_C_H = 0x0F,
	REGISTER_CHANNEL_C = REGISTER_CHANNEL_C_L,
	REGISTER_CHANNEL_D_DIVIDER = 0x10,
	REGISTER_CHANNEL_D_L = 0x12,
	REGISTER_CHANNEL_D_H = 0x13,
	REGISTER_CHANNEL_D = REGISTER_CHANNEL_D_L,
	REGISTER_COUNT = 0x14
};

typedef union {
	uint16_t value;
	struct {
		uint8_t l;
		uint8_t h;
	};
} regu16_t;

typedef struct {
	uint8_t divider;
	uint8_t reserved;
	regu16_t mag;
} channel_reg_t;

struct app_registers_t {
	union {
		uint8_t value;
		struct {
			uint8_t run : 1;
			uint8_t reserved : 7;
		};
	} control;
	uint8_t enabled;
	uint8_t reserved[2];

	channel_reg_t channel[4];
};

enum {
	COMMAND_COUNT = 0
};

#endif /* PERIMETER_WIRE_FIRMWARE_CONFIG_H_ */