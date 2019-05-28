
#ifndef PERIMETER_WIRE_FIRMWARE_CONFIG_H_
#define PERIMETER_WIRE_FIRMWARE_CONFIG_H_


#define APP_LAYER_REGISTERS registers
#define APP_LAYER_COMMANDS cmds

enum {
	REGISTER_CONTROL = 0x00,
	REGISTER_ENABLED = 0x01,
	REGISTER_CHANNEL_A_L = 0x02,
	REGISTER_CHANNEL_A_H = 0x03,
	REGISTER_CHANNEL_A = REGISTER_CHANNEL_A_L,
	REGISTER_CHANNEL_B_L = 0x04,
	REGISTER_CHANNEL_B_H = 0x05,
	REGISTER_CHANNEL_B = REGISTER_CHANNEL_B_L,
	REGISTER_CHANNEL_C_L = 0x06,
	REGISTER_CHANNEL_C_H = 0x07,
	REGISTER_CHANNEL_C = REGISTER_CHANNEL_C_L,
	REGISTER_CHANNEL_D_L = 0x08,
	REGISTER_CHANNEL_D_H = 0x09,
	REGISTER_CHANNEL_D = REGISTER_CHANNEL_D_L,
	APP_LAYER_MAX_REGISTER = 0x0A
};

struct app_registers_t {
	union {
		uint8_t value;
		struct {
			uint8_t run : 1;
			uint8_t reserved : 7;
		};
	} control;
	uint8_t enabled;
	uint8_t channel_a_l;
	uint8_t channel_a_h;
	uint8_t channel_b_l;
	uint8_t channel_b_h;
	uint8_t channel_c_l;
	uint8_t channel_c_h;
	uint8_t channel_d_l;
	uint8_t channel_d_h;
};

enum {
    APP_LAYER_MAX_COMMAND = 0
};
#endif /* PERIMETER_WIRE_FIRMWARE_CONFIG_H_ */