#include <atmel_start.h>
#include <hal_timer.h>
#include <perimeter_wire_sensor.h>
#include <uplink.h>
#include "com/usb_serial.h"
#include "com/app_layer.h"

static bool send_hello = false;


void uplink_set_reg_callback(uint8_t adr) {
	uint8_t enabled;
	uplink_get_enabled(&enabled);
	pwsens_set_enable(enabled);
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

	pwsens_init();
	//timer_start(&TIMER_0);

	while (1)
	{
		app_layer_cycle();
		pwsens_task();
		if (send_hello)
		{
			send_hello = false;
			app_layer_send_hello_message(FIRMWARE_VERSION);
		}
	}
}
