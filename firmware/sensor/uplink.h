#ifndef UPLINK_H_
#define UPLINK_H_

#include <perimeter_wire_sensor_firmware/registers.h>

extern uint8_t registers[REGISTER_COUNT];
extern void (* const cmds[COMMAND_COUNT])(void);

extern struct app_registers_t * app_registers;

void uplink_get_reg_callback(uint8_t adr);
void uplink_set_reg_callback(uint8_t adr);

inline static void uplink_get_flags(uint8_t* flags)
{
	*flags = app_registers->flags;
}

inline static uint8_t uplink_get_divider()
{
	return app_registers->config.divider;
}

inline static uint16_t uplink_get_code()
{
	return app_registers->config.code;
}

inline static uint8_t uplink_get_repeat()
{
	return app_registers->config.repeat;
}

inline static void uplink_set_channel(uint8_t channel, int16_t value, uint16_t quality)
{
	app_registers->channel[channel].mag.value = value;
	app_registers->channel[channel].qual.value = quality;
}

inline static void uplink_set_buffer_length(uint16_t length)
{
	app_registers->buffer.length = length;
}

inline static uint16_t uplink_get_buffer_index(void)
{
	return app_registers->buffer.index;
}

inline static void uplink_set_buffer_index(uint16_t index)
{
	app_registers->buffer.index = index;
}

inline static void uplink_set_buffer_value(uint16_t value)
{
	app_registers->buffer.value = value;
}

#endif /* UPLINK_H_ */