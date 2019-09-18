#include <atmel_start.h>
#include <hal_timer.h>
#include <perimeter_wire_generator.h>
#include <uplink.h>
#include "com/usb_serial.h"
#include "com/app_layer.h"

static bool send_hello = false;
static struct pwgen_config_t genA_config, genB_config;
static struct pwgen_t genA, genB;


void uplink_set_reg_callback(uint8_t adr) {
	uplink_read_generatorA_config(&genA_config);
	uplink_read_generatorB_config(&genB_config);
	
	pwgen_set_config(&genA, &genA_config, &TIMER_0);
	pwgen_set_config(&genB, &genB_config, &TIMER_0);
}

void usb_connect(void)
{
	send_hello = true;
}

int main(void)
{
	/* Initializes MCU, drivers and middleware */
	atmel_start_init();

	usb_serial_init(usb_connect);

	pwgen_init(&genA, PWGEN_A);
	pwgen_init(&genB, PWGEN_B);

	timer_start(&TIMER_0); 

	uplink_read_generatorA_config(&genA_config);
	uplink_read_generatorB_config(&genB_config);
	
	pwgen_set_config(&genA, &genA_config, &TIMER_0);
	pwgen_set_config(&genB, &genB_config, &TIMER_0);

	while (1)
	{
		app_layer_cycle();
		if (send_hello)
		{
			send_hello = false;
			app_layer_send_hello_message(FIRMWARE_VERSION);
		}
	}
}
