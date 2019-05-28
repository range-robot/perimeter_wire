#ifndef UPLINK_H_
#define UPLINK_H_

#include <perimeter_wire_sensor/registers.h>

extern uint8_t registers[REGISTER_COUNT];
extern void (* const cmds[COMMAND_COUNT])(void);

extern struct app_registers_t * app_registers;

void uplink_set_reg_callback(uint8_t adr);

inline static void uplink_get_enabled(uint8_t* enabled)
{
	*enabled = app_registers->enabled;
}

inline static void uplink_set_channel_a(uint16_t value)
{
	app_registers->channel_a.value = value;
}

inline static void uplink_set_channel_b(uint16_t value)
{
	app_registers->channel_b.value = value;
}

inline static void uplink_set_channel_c(uint16_t value)
{
	app_registers->channel_c.value = value;
}

inline static void uplink_set_channel_d(uint16_t value)
{
	app_registers->channel_d.value = value;
}

#endif /* UPLINK_H_ */