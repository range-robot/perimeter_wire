
#include <perimeter_wire_generator/registers.h>

// hookup app layer
uint8_t registers[REGISTER_COUNT] = {
	0,  // REGISTER_CONTROL
	0,  // REGISTER_TEMP
	1,  // REGISTER_A_MODE
	0,  // reserved
	5,  // REGISTER_A_DIV
	0,  // reserved
	0, 	// REGISTER_B_MODE
	0,  // reserved
	0,  // REGISTER_B_DIV
	0   // reserved
};

struct app_registers_t * app_registers = (struct app_registers_t*)registers;
void (* const cmds[COMMAND_COUNT])(void)  = {};