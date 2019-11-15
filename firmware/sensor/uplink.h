#ifndef UPLINK_H_
#define UPLINK_H_

#include <perimeter_wire_sensor_firmware/registers.h>

extern uint8_t registers[REGISTER_COUNT];
extern void (* const cmds[COMMAND_COUNT])(void);

extern struct app_registers_t * app_registers;

void uplink_set_reg_callback(uint8_t adr);

inline static void uplink_get_flags(uint8_t* flags)
{
	*flags = app_registers->flags;
}

inline static uint8_t uplink_get_divider(uint8_t channel)
{
	return app_registers->channel[channel].divider;
}

inline static uint16_t uplink_get_code(uint8_t channel)
{
	return app_registers->channel[channel].code;
}

inline static uint8_t uplink_get_repeat(uint8_t channel)
{
	return app_registers->channel[channel].repeat;
}

inline static void uplink_set_channel(uint8_t channel, int16_t value)
{
	app_registers->channel[channel].mag.value = value;
}

#endif /* UPLINK_H_ */