

#include "board.h"
#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/power.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>
#include <perimeter_wire_firmware/config.h>
#include "com/async_serial.h"
#include "com/data_link_layer.h"
#include "com/app_layer.h"
#include "system/scheduler.h"
#include "driver/perimeter_wire.h"

// hookup app layer
uint8_t registers[APP_LAYER_MAX_REGISTER] = {};
struct app_registers_t * app_registers = (struct app_registers_t*)registers;
void (* const APP_LAYER_COMMANDS[APP_LAYER_MAX_COMMAND])(void)  = {};

int main(void)
{
	/* Disable watchdog if enabled by bootloader/fuses */
	MCUSR &= ~(1 << WDRF);
	wdt_disable();

	/* Disable clock division */
	clock_prescale_set(clock_div_1);

	// Setup periphery
	scheduler_init();
	async_serial_init(STOPBIT_1BIT, PARITY_DISABLED);
	datalink_init();
	
	set_sleep_mode(SLEEP_MODE_IDLE);

	sei();

	app_layer_send_hello_message(FIRMWARE_VERSION);
	
	while (TRUE)
	{
		// this code is called in a loop as fast as possible
		app_layer_cycle();

		if (scheduler_next_slice()) {
			// this code is called every 1ms
			wdt_reset();
		
			pw_task_1ms();
			if (!scheduler_end_slice(FALSE)) {
				system_throw_error(ERROR_SCHEDULER_OVERTIME);
			}
		}
		else {
			sleep_mode();
		}
	}
}
