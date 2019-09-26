#ifndef SRC_COM_CONFIG_H_
#define SRC_COM_CONFIG_H_

// Configuration file for communication classes

// common
#include "console.h"

// app-layer
#include <perimeter_wire_generator/messages.h>
#include <perimeter_wire_generator/registers.h>
#define APP_LAYER_MAX_REGISTER REGISTER_COUNT

// data-link-layer
#define DATALINK_START_FLAG (0x7E)
#define DATALINK_END_FLAG (0x7E)
#define DATALINK_ESC (0x7D)
#define DATALINK_ESC_MASK (0x20)

//#define DEBUG_DDL

// async-serial
// Recevive buffer must keep all bytes received during one cycle
#define RECEIVE_BUFFER_SIZE 2048

#endif  // SRC_COM_CONFIG_H_