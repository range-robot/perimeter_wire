
#include "uplink.h"
#include "driver_init.h"
#include <perimeter_wire_generator_firmware/registers.h>
#include <err_codes.h>
#include <utils_assert.h>
#include <string.h>

// hookup app layer
uint8_t registers[REGISTER_COUNT] = {0};

struct app_registers_t * app_registers = (struct app_registers_t*)registers;
void (* const cmds[COMMAND_COUNT])(void)  = {};

#define SAVE_SIZE (REGISTER_SAVE_END - REGISTER_SAVE_START)

typedef struct {
	uint8_t reg[SAVE_SIZE];
	uint16_t crc;
} storage_t;

static uint16_t calculate_checksum(uint8_t* buffer, size_t size)
{
	uint16_t crc = 0xFFFF;
	uint8_t  i   = 0;

	for (; i < size; i++) {
		uint8_t value;

		value = (uint8_t)((crc >> 8) ^ buffer[i]);
		value = value ^ (value >> 4);
		crc   = (crc << 8) ^ ((uint16_t)value << 12) ^ ((uint16_t)value << 5) ^ (uint16_t)value;
	}
	return crc;
}

inline static void uplink_read_nv(void)
{
	storage_t buffer;

	/* Read data from RWWEE flash */
	int32_t res = _rww_flash_read(&FLASH_INSTANCE.dev, NVMCTRL_RWW_EEPROM_ADDR, (uint8_t*)&buffer, sizeof(storage_t));
	ASSERT(res == ERR_NONE);

	uint16_t crc = calculate_checksum(buffer.reg, sizeof(buffer.reg));
	if (buffer.crc == crc)
	{
		memcpy(registers + REGISTER_SAVE_START, &buffer.reg, SAVE_SIZE);
	}
}

inline static void uplink_write_nv(void)
{
	storage_t buffer;
	memcpy(&buffer.reg, registers + REGISTER_SAVE_START, SAVE_SIZE);
	buffer.crc = calculate_checksum(buffer.reg, SAVE_SIZE);

	/* Write data to RWWEE flash */
	int32_t res = _rww_flash_write(&FLASH_INSTANCE.dev, NVMCTRL_RWW_EEPROM_ADDR, (uint8_t*)&buffer, sizeof(storage_t));
	ASSERT(res == ERR_NONE);
}

void uplink_init(void)
{
	uplink_read_nv();
}

void uplink_task(void)
{
	if (app_registers->control.save != 0)
	{
		uplink_write_nv();
		app_registers->control.save = 0;
	}
}
