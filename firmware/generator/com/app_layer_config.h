#ifndef APP_LAYER_CONFIG_H_
#define APP_LAYER_CONFIG_H_

#include "hpl_reset.h"
#include "driver/uplink.h"
#include <perimeter_wire_generator/registers.h>

// macro for software reset
#define APP_RESET(bootload) _reset_mcu()

#define APP_LAYER_GET_REG_HOOK(adr)
#define APP_LAYER_SET_REG_HOOK(adr) uplink_set_reg_callback(adr);

#define APP_LAYER_REGISTERS registers
#define APP_LAYER_MAX_REGISTER REGISTER_COUNT
#define APP_LAYER_COMMANDS cmds
#define APP_LAYER_MAX_COMMAND COMMAND_COUNT


#endif /* APP_LAYER_CONFIG_H_ */