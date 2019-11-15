
#include <perimeter_wire_sensor_firmware/registers.h>

// hookup app layer
uint8_t registers[REGISTER_COUNT] = {};
struct app_registers_t * app_registers = (struct app_registers_t*)registers;
void (* const cmds[COMMAND_COUNT])(void)  = {};