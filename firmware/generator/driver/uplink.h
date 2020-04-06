#ifndef UPLINK_H_
#define UPLINK_H_

#include "perimeter_wire_generator.h"
#include <perimeter_wire_generator_firmware/registers.h>

extern uint8_t registers[REGISTER_COUNT];
extern void (* const cmds[COMMAND_COUNT])(void);

extern struct app_registers_t * app_registers;

void uplink_set_reg_callback(uint8_t adr);
void uplink_init(void);
void uplink_task(void);

inline static void uplink_read_generatorA_config(struct pwgen_config_t *const config)
{
	config->divider = app_registers->channel_a.divider;
	config->mode = app_registers->channel_a.mode;
	config->code = app_registers->channel_a.code.value;
}

inline static void uplink_read_generatorB_config(struct pwgen_config_t *const config)
{
	config->divider = app_registers->channel_b.divider;
	config->mode = app_registers->channel_b.mode;
	config->code = app_registers->channel_b.code.value;
}

inline static void uplink_write_temp(uint8_t temp)
{
	app_registers->temp = temp;
}

inline static void uplink_write_voltage(uint8_t voltage)
{
	app_registers->voltage = voltage;
}

#endif /* UPLINK_H_ */