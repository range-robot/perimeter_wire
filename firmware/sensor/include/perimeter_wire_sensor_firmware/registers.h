
#ifndef PERIMETER_WIRE_SENSOR_REGISTERS_H_
#define PERIMETER_WIRE_SENSOR_REGISTERS_H_

#include <inttypes.h>

enum {
	REGISTER_CONTROL = 0x00,
	REGISTER_FLAGS = 0x01,
	REGISTER_CHANNEL_DIVIDER = 0x04,
	REGISTER_CHANNEL_REPEAT = 0x05,
	REGISTER_CHANNEL_CODE = 0x06,	// 16-bit
	REGISTER_CHANNEL_A = 0x08, 		// 16-bit
	REGISTER_CHANNEL_A_QUAL = 0x0A,	// 16-bit
	REGISTER_CHANNEL_B = 0x0C, 		// 16-bit
	REGISTER_CHANNEL_B_QUAL = 0x0E, // 16-bit
	REGISTER_CHANNEL_C = 0x10,		// 16-bit
	REGISTER_CHANNEL_C_QUAL = 0x12, // 16-bit
	REGISTER_BUFFER_LENGTH = 0x14,  // 16-bit
	REGISTER_BUFFER_INDEX = 0x16,	// 16-bit auto-increment
	REGISTER_BUFFER_VALUE = 0x18,	// 16-bit
	REGISTER_COUNT = 0x20
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
} channel_config_reg_t;

typedef struct {
	reg_int16_t mag;
	reg_int16_t qual;
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

	channel_config_reg_t config;
	channel_reg_t channel[3];
	buffer_reg_t buffer;
};

enum {
	COMMAND_COUNT = 0
};

enum
{
	PWSENS_FLAGS_ENABLE_CH_A = 0x01,
	PWSENS_FLAGS_ENABLE_CH_B = 0x02,
	PWSENS_FLAGS_ENABLE_CH_C = 0x04,
	PWSENS_FLAGS_ENABLE = PWSENS_FLAGS_ENABLE_CH_A | PWSENS_FLAGS_ENABLE_CH_B | PWSENS_FLAGS_ENABLE_CH_C,
	PWSENS_FLAGS_START = 0x10,
	PWSENS_FLAGS_CONTINUOUS = 0x20,
	PWSENS_FLAGS_SYNC_MODE = 0x40,
	PWSENS_FLAGS_DIFFERENTIAL = 0x80
};

#endif /* PERIMETER_WIRE_FIRMWARE_CONFIG_H_ */