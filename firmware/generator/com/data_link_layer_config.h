/*
 * datalink_config.h
 *
 * Created: 28.08.2017 09:52:22
 *  Author: Michael
 */ 


#ifndef DATALINK_CONFIG_H_
#define DATALINK_CONFIG_H_

#include "usb_serial.h"
#include <perimeter_wire_generator/messages.h>

#define DATALINK_START_FLAG (0x7E)
#define DATALINK_END_FLAG (0x7E)
#define DATALINK_ESC (0x7D)
#define DATALINK_ESC_MASK (0x20)
#define DATALINK_CHECKSUM ODD
#define DATALINK_RECEIVE_BUFFER_SIZE (16)
#define DATALINK_SEND_BUFFER_SIZE (16)

#define DATALINK_READ() async_serial_get()
#define DATALINK_WRITE(buffer, count) async_serial_write(buffer, count)



#endif /* DATALINK_CONFIG_H_ */