
#include <stdio.h>

#include "util.h"
#include "flash.h"
// Including only "stm32f7xx_hal_flash.h" creates problems while compilation
#include "stm32f7xx_hal.h"


typedef struct FlashRange {
	const uint32_t start;
	const uint32_t end;
} FlashRange_t;

typedef struct FlashBank {
	const FlashRange_t* const sectors;
	const uint32_t len;
	const uint32_t id;
} FlashBank_t;

#ifdef __STM32F7xx_HAL_H
// see:
// #define FLASHITCM_BASE         0x00200000UL /*!< Base address of: (up to 1 MB) embedded FLASH memory accessible over ITCM              */
// #define FLASHAXI_BASE          0x08000000UL /*!< Base address of: (up to 1 MB) embedded FLASH memory accessible over AXI                */

// base address on AXIM interface (because FLASH_BASE and FLASH_END)
static const FlashRange_t sector_ranges[] = {
		{ .start = 0x08000000, .end = 0x08007FFF },
		{ .start = 0x08008000, .end = 0x0800FFFF },
		{ .start = 0x08010000, .end = 0x08017FFF },
		{ .start = 0x08018000, .end = 0x0801FFFF },
		{ .start = 0x08020000, .end = 0x0803FFFF },
		{ .start = 0x08040000, .end = 0x0807FFFF },
		{ .start = 0x08080000, .end = 0x080BFFFF },
		{ .start = 0x080C0000, .end = 0x080FFFFF },
};

static const FlashBank_t banks[] = {
		{ .sectors = sector_ranges, .len = LEN_ARRAY(sector_ranges), .id = 0 },
};
#elif defined(STM32H7xx_HAL_H)
static const FlashRange_t sectors_bank1[] = {
		{ .start = 0x08000000, .end = 0x08020000 },
		{ .start = 0x08020000, .end = 0x08040000 },
		{ .start = 0x08040000, .end = 0x08060000 },
		{ .start = 0x08060000, .end = 0x08080000 },
		{ .start = 0x08080000, .end = 0x080A0000 },
		{ .start = 0x080A0000, .end = 0x080C0000 },
		{ .start = 0x080C0000, .end = 0x080E0000 },
		{ .start = 0x080E0000, .end = 0x08100000 },
};

static const FlashRange_t sectors_bank2[] = {
		{ .start = 0x08100000, .end = 0x08120000 },
		{ .start = 0x08120000, .end = 0x08140000 },
		{ .start = 0x08140000, .end = 0x08160000 },
		{ .start = 0x08160000, .end = 0x08180000 },
		{ .start = 0x08180000, .end = 0x081A0000 },
		{ .start = 0x081A0000, .end = 0x081C0000 },
		{ .start = 0x081C0000, .end = 0x081E0000 },
		{ .start = 0x081E0000, .end = 0x08200000 },
};

static const FlashBank_t banks[] = {
		{ .sectors = sectors_bank1, .len = LEN_ARRAY(sectors_bank1), .id = FLASH_BANK_1 },
		{ .sectors = sectors_bank2, .len = LEN_ARRAY(sectors_bank2), .id = FLASH_BANK_2 }
};
#endif


uint8_t flash_is_out_of_range(uint32_t address) {
	return address < FLASH_BASE || address > FLASH_END;
}

uint8_t flash_is_same_sector(const AddressToFlash_t* const data1, const AddressToFlash_t* const data2) {
	return data1 != NULL && data2 != NULL
			&& data1->idx_bank == data2->idx_bank
			&& data1->idx_sector == data2->idx_sector;
}

int8_t flash_address2sector(uint32_t address, AddressToFlash_t* const mapping) {
	if (flash_is_out_of_range(address)) {
		printf("Failed to map address to sector: address is out of range\r\n");
		return -1;
	}
	for (int idx_bank = 0; idx_bank < LEN_ARRAY(banks); idx_bank++) {
		const FlashBank_t* const bank = banks + idx_bank;
		for (int idx_sector = 0; idx_sector < bank->len; idx_sector++) {
			const FlashRange_t* const range = bank->sectors + idx_sector;
			if (range->start <= address && address < range->end) {
				mapping->bank_id = bank->id;
				mapping->idx_bank = idx_bank;
				mapping->idx_sector = idx_sector;
				mapping->address = address;
				return 0;
			}
		}
	}
	printf("Failed to map address to sector: bank for address or sector not found\r\n");
	return -1;
}

int8_t flash_read(
	uint32_t address, uint32_t* const buffer, uint32_t len, uint32_t* const red
) {
	if (red == NULL || buffer == NULL) {
		return -1;
	}
	*red = 0;
	if (len == 0) {
		return 0;
	}
	if (flash_is_out_of_range(address)) {
		printf("Failed to read from flash address: address is out of range\r\n");
		return -1;
	}
	uint32_t address_end = address + len * sizeof(*buffer);
	if (flash_is_out_of_range(address_end)) {
		printf("Failed to read from flash address: end address is out of range\r\n");
		return -1;
	}
	__IO uint32_t* tmp = (__IO uint32_t*) address;
	while ((*red) < len) {
		buffer[*red] = *tmp;
		tmp++;
		*red = *red + 1;
	}
    return 0;
}

int8_t flash_write(
	uint32_t address, const uint32_t* const data, uint32_t len,
	uint32_t* const written
) {
	int8_t status = 0;
	HAL_StatusTypeDef result = HAL_OK;
	if (written == NULL || data == NULL) {
		return -1;
	}
	*written = 0;
	if (len == 0) {
		return status;
	}
	do {
		// validate address range
		AddressToFlash_t flash_start = { 0 };
		if (flash_address2sector(address, &flash_start)) {
			break;
		}
		AddressToFlash_t flash_end = { 0 };
		if (flash_address2sector(address + len * sizeof(*data), &flash_end)) {
			break;
		}

		HAL_StatusTypeDef result = HAL_FLASH_Unlock();
		if (result != HAL_OK) {
			printf("Failed to unlock flash\r\n");
			status = -1;
			break;
		}
		while (*written < len) {
			if (
			// WARNING! SOME IMPLEMENTATIONS REQUIRE POINTER TO DATA!
#ifdef __STM32F7xx_HAL_H
			HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address, *(data + (*written)))
#elif defined(STM32H7xx_HAL_H)
			HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, address, (uint32_t) (data + (*written)))
#endif
			 == HAL_OK){
				address += sizeof(*data);
				*written = *written + 1;
			}
			else {
				printf("Failed to write to flash on address 0x%lX\r\n", address);
				status = -1;
				break;
			}
		}
	} while(0);

    result = HAL_FLASH_Lock();
    if (result != HAL_OK) {
		printf("Failed to write lock flash\r\n");
		status = -1;
	}
    return status;
}

int8_t flash_clear_sector(uint32_t address, uint32_t bytes) {
	int8_t status = 0;
	HAL_StatusTypeDef result = HAL_OK;
	do {
		AddressToFlash_t flash_start = { 0 };
		if (flash_address2sector(address, &flash_start)) {
			break;
		}
		AddressToFlash_t flash_end = { 0 };
		if (flash_address2sector(address + bytes, &flash_start)) {
			break;
		}

		HAL_StatusTypeDef result = HAL_FLASH_Unlock();
		if (result != HAL_OK) {
			printf("Failed to unlock flash\r\n");
			status = -1;
			break;
		}
		static FLASH_EraseInitTypeDef erese_settings;
		erese_settings.TypeErase = FLASH_TYPEERASE_SECTORS;
		erese_settings.VoltageRange = FLASH_VOLTAGE_RANGE_1;
		erese_settings.Sector = flash_start.idx_sector;
		// erese_settings.Banks = flash_start.bank_id;

		if (flash_start.bank_id == flash_end.bank_id) {
			erese_settings.NbSectors = flash_end.idx_sector - flash_start.idx_sector;
		}
		else {
			// TODO: test
			const uint8_t len_banks = flash_end.idx_bank;
			for (uint8_t i = flash_start.idx_bank; i <= len_banks; i++) {
				if (i == flash_start.idx_bank) {
					erese_settings.NbSectors = banks[i].len - flash_start.idx_sector;
				}
				if (i == flash_end.idx_bank) {
					erese_settings.NbSectors += flash_end.idx_sector;
				}
				if (i != flash_start.idx_bank && i != flash_end.idx_bank) {
					erese_settings.NbSectors += banks[i].len;
				}
			}
		}
		erese_settings.NbSectors += 1;

		uint32_t status_erase = 0;
		result = HAL_FLASHEx_Erase(&erese_settings, &status_erase);
		if (result != HAL_OK) {
			printf("Failed to erase sectors. Status 0x%lX\r\n", status_erase);
			status = -1;
			break;
		}
	} while(0);

    result = HAL_FLASH_Lock();
    if (result != HAL_OK) {
		printf("Failed to write lock flash\r\n");
		status = -1;
	}
    return status;
}
