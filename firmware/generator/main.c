#include <atmel_start.h>
#include <hal_timer.h>
#include <hpl_adc_config.h>
#include "driver/uplink.h"
#include "driver/perimeter_wire_generator.h"
#include "com/usb_serial.h"
#include "com/app_layer.h"
#include "driver/leds.h"

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

static struct timer_task led_task;
static struct timer_task adc_task;

enum {
	ADC_READ_TEMP,
	ADC_READ_VOLTAGE
} adc_state = ADC_READ_TEMP;

static void adc_convert_cb(const struct adc_async_descriptor *const descr, const uint8_t channel)
{
	uint8_t value;
	if (adc_async_read_channel(&ADC_0, channel, &value, 1) <= 0)
		return;
	switch(adc_state)
	{
		case ADC_READ_TEMP:
			uplink_write_temp(value);
			adc_state = ADC_READ_VOLTAGE;
			break;
		case ADC_READ_VOLTAGE:
			uplink_write_voltage(value);
			adc_state = ADC_READ_TEMP;
	}
}

static void adc_task_cb()
{
	switch (adc_state)
	{
		case ADC_READ_TEMP:  // start temp reading
			adc_async_set_inputs(&ADC_0, CONF_ADC_0_MUX_TEMP, CONF_ADC_0_MUXNEG, 0);
			adc_async_start_conversion(&ADC_0);
			break;
		case ADC_READ_VOLTAGE: // start voltage reading
			adc_async_set_inputs(&ADC_0, CONF_ADC_0_MUX_VOLTAGE, CONF_ADC_0_MUXNEG, 0);
			adc_async_start_conversion(&ADC_0);
			break;
	}
}

int main(void)
{
	/* Initializes MCU, drivers and middleware */
	atmel_start_init();

	usb_serial_init(usb_connect);
	
	pwgen_init(&genA, PWGEN_A);
	pwgen_init(&genB, PWGEN_B);

	timer_start(&TIMER_0); 

	// setup LED (with startup signal)
	LED_mode_A.color = LC_Blue;
	LED_mode_A.pulse = LM_ON;
	LED_mode_B.color = LC_Blue;
	LED_mode_B.pulse = LM_ON;
	LED_init();
	led_task.cb = &LED_task_100ms;
	led_task.interval = TIMER_FREQUENCY / 10; // 10 Hz
	led_task.mode = TIMER_TASK_REPEAT;
	timer_add_task(&TIMER_0, &led_task);
	
	// setup adc
	adc_async_register_callback(&ADC_0, 0, ADC_ASYNC_CONVERT_CB, adc_convert_cb);
	adc_async_enable_channel(&ADC_0, 0);
	adc_task.cb = adc_task_cb;
	adc_task.interval = TIMER_FREQUENCY / 1; // 1 Hz
	adc_task.mode = TIMER_TASK_REPEAT;
	timer_add_task(&TIMER_0, &adc_task);

	uplink_read_generatorA_config(&genA_config);
	uplink_read_generatorB_config(&genB_config);
	
	pwgen_set_config(&genA, &genA_config, &TIMER_0);
	pwgen_set_config(&genB, &genB_config, &TIMER_0);

	while (1)
	{
		usb_serial_task();
		app_layer_cycle();
		if (send_hello)
		{
			send_hello = false;
			app_layer_send_hello_message(FIRMWARE_VERSION);
		}
		pwgen_task();
	}
}
