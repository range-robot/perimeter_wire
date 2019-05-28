#ifndef USB_SERIAL_H_
#define USB_SERIAL_H_

#include <utils.h>
#include <inttypes.h>

/*
 * usb async serial interface for SAM devices
 * Uses ASF4 CDC ACM driver
 */

int32_t usb_serial_init(FUNC_PTR connect_cb);

/* 
 * Transmit data
*/
void async_serial_write(const uint8_t* buffer, uint8_t count);

/*
 * Receive data
*/
int16_t async_serial_get(void);


#endif /* USB_SERIAL_H_ */