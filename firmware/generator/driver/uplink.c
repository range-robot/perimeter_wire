
#include <perimeter_wire_generator/registers.h>

// hookup app layer
uint8_t registers[REGISTER_COUNT] = {
	0,  // REGISTER_CONTROL
	0,  // REGISTER_TEMP
	0x07,  // REGISTER_A_MODE
	4,  // REGISTER_A_DIV
	0x53,  // code
	0x35,  // code
	0x07, 	// REGISTER_B_MODE
	4,  // REGISTER_B_DIV
	0x53,  // code
	0x35   // code
};

struct app_registers_t * app_registers = (struct app_registers_t*)registers;
void (* const cmds[COMMAND_COUNT])(void)  = {};