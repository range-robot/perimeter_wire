#ifndef APP_LAYER_CONFIG_H_
#define APP_LAYER_CONFIG_H_

#include "hpl_reset.h"
#include "uplink.h"
#include <perimeter_wire_sensor_firmware/registers.h>
#include <perimeter_wire_sensor_firmware/messages.h>


#define BOOTLOADER_VECTORS_ADDRESS 0

static void start_bootloader()
{
  uint32_t* vectors_ptr = BOOTLOADER_VECTORS_ADDRESS;

  uint32_t start_adr = (uint32_t)vectors_ptr ;
  start_adr++ ;

  /* Rebase the Stack Pointer */
  __set_MSP( (uint32_t)(*vectors_ptr) );

  /* Rebase the vector table base address */
  SCB->VTOR = ((uint32_t)(vectors_ptr) & SCB_VTOR_TBLOFF_Msk);

  /* Jump to application Reset Handler in the application */
  asm("bx %0"::"r"(start_adr));
}
// macro for software reset
#define APP_RESET(bootload) if (bootload) start_bootloader(); else _reset_mcu();

#define APP_LAYER_SET_REG_HOOK(adr) uplink_set_reg_callback(adr);
#define APP_LAYER_GET_REG_HOOK(adr) uplink_get_reg_callback(adr);

#define APP_LAYER_REGISTERS registers
#define APP_LAYER_MAX_REGISTER REGISTER_COUNT
#define APP_LAYER_COMMANDS cmds
#define APP_LAYER_MAX_COMMAND COMMAND_COUNT


#endif /* APP_LAYER_CONFIG_H_ */