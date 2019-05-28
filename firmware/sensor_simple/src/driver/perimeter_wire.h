
#ifndef PERIMETER_WIRE_H_
#define PERIMETER_WIRE_H_

#include <perimeter_wire_firmware/config.h>

extern struct app_registers_t * app_registers;

/*
 * Enable ADC
 * Start conversion immediately
 * Enable AD interrupt
 * Prescaler: 64
 */
#define ADCSRA_ENABLE ((1<<ADEN) | (1<<ADSC) | (1<<ADIE) | (1<<ADPS2) | (1 << ADPS1))
/*
 * Reference: Vcc
 * Align right
 */
#define ADMUX_BASE ((1<<REFS0) | (0<<ADLAR))

#define PW_MAX_CHANNELS (4)

struct {
	uint8_t current_channel;
	uint8_t next_channel;
	uint8_t is_running:1;
	uint8_t reserverd:7;
} pw_status_;


static inline void switch_init(void) {
	// Clear ADCSRB for 'free running' mode
	ADCSRB = 0;
	
	// configure input pins
	DIDR0 = (1<< ADC5D) | (1<< ADC4D) | (1<< ADC3D) | (1<< ADC2D);

	pw_status_.next_channel = 0xff;
	pw_status_.current_channel = 0xff;
	pw_status_.is_running = FALSE;
}

static inline uint8_t pw_get_next_channel_(uint8_t startat) {
	uint8_t enabled = app_registers->enabled;
	for (; startat < PW_MAX_CHANNELS; startat++) {
		if ((enabled & (1 << startat)) != 0) {
			return startat;
		}
	}
	return 0xff;
}

static inline void pw_prepare_next_channel_(uint8_t startat) {
	// setup mux for next channel
	// there should be some time between prepare and start conversion
	uint8_t channel = pw_get_next_channel_(startat);
	switch (channel) {
		default:
		case 0:
			ADMUX = ADMUX_BASE | (2<<MUX0);
			break;
		case 1:
			ADMUX = ADMUX_BASE | (3<<MUX0);
			break;
		case 2:
			ADMUX = ADMUX_BASE | (4<<MUX0);
			break;
		case 3:
			ADMUX = ADMUX_BASE | (5<<MUX0);
			break;
	}
	pw_status_.next_channel = channel;
}

static inline void pw_read_next_channel_() {
	// trigger read
	uint8_t channel = pw_status_.next_channel;
	if (channel == 0xff)
	{
		pw_status_.current_channel = 0xff;
		pw_status_.is_running = FALSE;
		ADCSRA = 0; // Disable adc
	}
	else
	{
		pw_status_.current_channel = channel;
		pw_status_.is_running = TRUE;
		ADCSRA = ADCSRA_ENABLE;
	}
}

static inline void pw_task_1ms(void) {
	if (pw_status_.is_running)
		return;
	
	if (app_registers->control.run) {
		pw_prepare_next_channel_(0);
		pw_read_next_channel_();
		pw_prepare_next_channel_(pw_status_.next_channel + 1);
	} 
}

ISR(ADC_vect)
{
	// Read the ADC Result
	uint8_t channel = pw_status_.current_channel;
	switch (channel) {
		case 0:
 			app_registers->channel_a_l = ADCL;
			app_registers->channel_a_h = ADCH;
			break;
		case 1:
			app_registers->channel_b_l = ADCL;
			app_registers->channel_b_h = ADCH;
			break;
		case 2:
			app_registers->channel_c_l = ADCL;
			app_registers->channel_c_h = ADCH;
			break;
		case 3:
			app_registers->channel_d_l = ADCL;
			app_registers->channel_d_h = ADCH;
			break;
		default:
			break;
	}
	
	// get next channel
	pw_read_next_channel_();
	
	// set mux to next channel
	pw_prepare_next_channel_(pw_status_.next_channel + 1);
}

#endif /* PERIMETER_WIRE_H_ */