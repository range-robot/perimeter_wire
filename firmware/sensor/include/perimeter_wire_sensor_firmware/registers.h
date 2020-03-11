
#ifndef PERIMETER_WIRE_SENSOR_REGISTERS_H_
#define PERIMETER_WIRE_SENSOR_REGISTERS_H_

#include <inttypes.h>

enum {
	REGISTER_CONTROL = 0x00,
	REGISTER_FLAGS = 0x01,
	REGISTER_CHANNEL_A_DIVIDER = 0x04,
	REGISTER_CHANNEL_A_REPEAT = 0x05,
	REGISTER_CHANNEL_A_CODE = 0x06,
	REGISTER_CHANNEL_A_L = 0x08,
	REGISTER_CHANNEL_A_H = 0x09,
	REGISTER_CHANNEL_A = REGISTER_CHANNEL_A_L,
	REGISTER_CHANNEL_B_DIVIDER = 0x0A,
	REGISTER_CHANNEL_B_REPEAT = 0x0B,
	REGISTER_CHANNEL_B_CODE = 0x0C,
	REGISTER_CHANNEL_B_L = 0x0E,
	REGISTER_CHANNEL_B_H = 0x0F,
	REGISTER_CHANNEL_B = REGISTER_CHANNEL_B_L,
	REGISTER_CHANNEL_C_DIVIDER = 0x10,
	REGISTER_CHANNEL_C_REPEAT = 0x11,
	REGISTER_CHANNEL_C_CODE = 0x12,
	REGISTER_CHANNEL_C_L = 0x14,
	REGISTER_CHANNEL_C_H = 0x15,
	REGISTER_CHANNEL_C = REGISTER_CHANNEL_C_L,
	REGISTER_CHANNEL_D_DIVIDER = 0x16,
	REGISTER_CHANNEL_D_REPEAT = 0x17,
	REGISTER_CHANNEL_D_CODE = 0x18,
	REGISTER_CHANNEL_D_L = 0x1A,
	REGISTER_CHANNEL_D_H = 0x1B,
	REGISTER_CHANNEL_D = REGISTER_CHANNEL_D_L,
	REGISTER_BUFFER_LENGTH = 0x1C,  // 16-bit
	REGISTER_BUFFER_INDEX = 0x1E,	// 16-bit auto-increment
	REGISTER_BUFFER_VALUE = 0x20,	// 16-bit
	REGISTER_COUNT = 0x22
};

typedef union {
	int16_t value;
	struct {
		uint8_t l;
		uint8_t h;
	};
} reg_int16_t;

typedef struct {
	uint8_t divider;
	uint8_t repeat;
	uint16_t code;
	reg_int16_t mag;
} channel_reg_t;

typedef struct {
	uint16_t length;
	uint16_t index;
	uint16_t value;
} buffer_reg_t;

struct app_registers_t {
	union {
		uint8_t value;
		struct {
			uint8_t run : 1;
			uint8_t reserved : 7;
		};
	} control;
	uint8_t flags;
	uint8_t reserved[2];

	channel_reg_t channel[4];
	buffer_reg_t buffer;
};

enum {
	COMMAND_COUNT = 0
};

#endif /* PERIMETER_WIRE_FIRMWARE_CONFIG_H_ */