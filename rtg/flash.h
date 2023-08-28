#include <inttypes.h>

#ifndef FLASH_H_
#define FLASH_H_

typedef struct AddressToFlash {
	uint8_t bank_id;
	uint8_t idx_bank;
	uint8_t idx_sector;
	uint32_t address;
} AddressToFlash_t;

int8_t flash_address2sector(uint32_t address, AddressToFlash_t* const mapping);

uint8_t flash_is_same_sector(const AddressToFlash_t* const data1, const AddressToFlash_t* const data2);

int8_t flash_read(
	uint32_t address, uint32_t* const buffer, uint32_t len,
	uint32_t* const red
);

int8_t flash_write(
	uint32_t address, const uint32_t* const data, uint32_t len,
	uint32_t* const written
);

int8_t flash_clear_sector(uint32_t address, uint32_t bytes);

#endif
