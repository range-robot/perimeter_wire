
#include <usbd_config.h>
#include <cdcdf_acm.h>
#include <inttypes.h>
#include <stdio.h>
#include "error.h"
#include "usb_serial.h"

#define USB_SERIAL_RX_SIZE CONF_USB_CDCD_ACM_DATA_BULKIN_MAXPKSZ_HS

static FUNC_PTR connect_callback;

// Multiple buffers
// one buffer for udb driver
// the other for data link layer
// rest is for async
// toggled on event.

struct rx_buffer {
	uint8_t buffer[USB_SERIAL_RX_SIZE];
	uint32_t start;
	uint32_t count;
};

#define RXBUFFER_COUNT 4
uint32_t rx_buffer_write_index, rx_buffer_read_index;

struct rx_buffer rx_buffer[RXBUFFER_COUNT];

/**
 * \brief Callback invoked when bulk IN data received
 */
static bool usb_device_cb_read(const uint8_t ep, const enum usb_xfer_code rc, const uint32_t count)
{
	if (rc == USB_XFER_DONE)
	{
		uint32_t index = rx_buffer_write_index;
		rx_buffer[index].start = 0;
		rx_buffer[index].count = count;
	
		index ++;
		if (index == RXBUFFER_COUNT)
			index = 0;

		if (rx_buffer[index].start != rx_buffer[index].count) {
			system_throw_error(ERR_OVERFLOW);
			// drop buffer
			rx_buffer[index].start = rx_buffer[index].count = 0;
		}
		rx_buffer_write_index = index;
		return (ERR_NONE != cdcdf_acm_read(rx_buffer[index].buffer, USB_SERIAL_RX_SIZE));
	}
	return false;
}

/**
 * \brief Callback invoked when usb state changes
 */
static bool usb_device_cb_state_c(usb_cdc_control_signal_t state)
{
	if (state.rs232.RTS)
	{
		/* Callbacks must be registered after endpoint allocation */
		cdcdf_acm_register_callback(CDCDF_ACM_CB_READ, (FUNC_PTR)usb_device_cb_read);
		//cdcdf_acm_register_callback(CDCDF_ACM_CB_WRITE, (FUNC_PTR)usb_device_cb_write);
		
		/* stop ongoing transmissions */
		cdcdf_acm_stop_xfer();		
		/* Start RX */
		rx_buffer_read_index = 0;
		rx_buffer_write_index = 0;
		rx_buffer[0].start = rx_buffer[0].count = 0;

		int32_t err =  cdcdf_acm_read(rx_buffer[0].buffer, USB_SERIAL_RX_SIZE);
		if (ERR_NONE != err)
			return true;

		if (connect_callback)
			connect_callback();
	}

	/* No error. */
	return false;
}

/**
 * \brief Hookup usb serial
 */
int32_t usb_serial_init(FUNC_PTR connect_cb)
{
	while (!cdcdf_acm_is_enabled())
	{
		// wait cdc acm to be installed
	};

	connect_callback = connect_cb;
	return cdcdf_acm_register_callback(CDCDF_ACM_CB_STATE_C, (FUNC_PTR)usb_device_cb_state_c);
}

int16_t async_serial_get(void)
{
	uint32_t index = rx_buffer_read_index;
	
	while (index != rx_buffer_write_index)
	{
		if (rx_buffer[index].count == rx_buffer[index].start)
		{
			uint32_t current = index;
			index ++;
			if (index == RXBUFFER_COUNT)
				index = 0;
			
			// detect resets here (from connect callback)
			bool reset = true;
			CRITICAL_SECTION_ENTER()
			__disable_irq(); //disable all interrupts
			if (rx_buffer_read_index == current)
				rx_buffer_read_index = index;
			reset = false;
			CRITICAL_SECTION_LEAVE()

			if (reset)
				return EOF;
		}
		else
		{
			uint32_t start = rx_buffer[index].start;
			uint8_t c = rx_buffer[index].buffer[start++];
			rx_buffer[index].start = start;
			return c;
		}
	}
	return EOF;
}

void async_serial_write(const uint8_t* buf, uint8_t count)
{	
	int32_t res = cdcdf_acm_write(buf, count);
	ASSERT(res == count);
}
