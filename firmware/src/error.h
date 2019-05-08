
#ifndef ERROR_H_
#define ERROR_H_

#include <stdint.h>

#define ERROR_OK 0
#define ERROR_SCHEDULER_OVERTIME 1

#define ERROR_BUFFER_OVERFLOW 3
#define ERROR_INVALID_MESSAGE 4
#define ERROR_INVALID_LENGTH 5
#define ERROR_INVALID_CHECKSUM 6
#define ERROR_INVALID_ADDRESS 7
#define ERROR_INVALID_COMMAND 8
#define ERROR_EXECUTE_BUSY 9

extern uint8_t system_status;

static void system_throw_error(uint8_t error) {
	if (system_status == ERROR_OK)
	system_status = error;
}
#endif /* ERROR_H_ */