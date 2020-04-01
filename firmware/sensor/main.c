#include <atmel_start.h>
#include <perimeter_wire_sensor.h>
#include <uplink.h>
#include "com/usb_serial.h"
#include "com/app_layer.h"

static bool send_hello = false;

void uplink_get_reg_callback(uint8_t adr) {
	if (adr == REGISTER_BUFFER_VALUE) {
		uint16_t index = uplink_get_buffer_index();
		if (index >= pwsens_get_buffer_length())
			index = 0;
		uplink_set_buffer_value(pwsens_get_buffer_value(index));
		uplink_set_buffer_index(index + 1);
	}
}
void uplink_set_reg_callback(uint8_t adr) {
	if (adr >= REGISTER_FLAGS && adr <= REGISTER_CHANNEL_CODE)
	{
		pwsens_set_config(uplink_get_flags(), uplink_get_divider(), uplink_get_code(), uplink_get_repeat(), uplink_get_filter());
	}
}

void usb_connect(void)
{
//	send_hello = true;
}

int main(void)
{
	/* Initializes MCU, drivers and middleware */
	atmel_start_init();

	usb_serial_init(usb_connect);

	pwsens_init();
	//timer_start(&TIMER_0);

	uplink_set_buffer_length(pwsens_get_buffer_length());

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
