
#ifndef ERROR_H_
#define ERROR_H_

#include <stdint.h>
#include <err_codes.h>

extern uint8_t system_status;

static void system_throw_error(uint8_t error) {
	if (system_status == ERR_NONE)
		system_status = error;
}
#endif /* ERROR_H_ */