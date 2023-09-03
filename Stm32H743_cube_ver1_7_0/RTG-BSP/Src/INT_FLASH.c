/*
 * INT_FLASH.c
 *
 *  Created on: 21 Feb 2022
 *      Author: itzhaki
 */

#include <bit.h>
#include <INT_FLASH.h>
#include <stm32h7xx_hal_def.h>
#include <stm32h7xx_hal_flash.h>
#include <stm32h7xx_hal_flash_ex.h>
#include <string.h>
#include <timer.h>


// Find to witch sector this address belongs
static uint32_t GetSector(uint32_t Address)
{
	uint32_t sector = 0;

	/* BANK 1 */
	if ((Address >= 0x08000000) && (Address < 0x08020000))
	{
		sector = FLASH_SECTOR_0;
	}

	else if ((Address >= 0x08020000) && (Address < 0x08040000))
	{
		sector = FLASH_SECTOR_1;
	}

	else if ((Address >= 0x08040000) && (Address < 0x08060000))
	{
		sector = FLASH_SECTOR_2;
	}

	else if ((Address >= 0x08060000) && (Address < 0x08080000))
	{
		sector = FLASH_SECTOR_3;
	}

	else if ((Address >= 0x08080000) && (Address < 0x080A0000))
	{
		sector = FLASH_SECTOR_4;
	}

	else if ((Address >= 0x080A0000) && (Address < 0x080C0000))
	{
		sector = FLASH_SECTOR_5;
	}

	else if ((Address >= 0x080C0000) && (Address < 0x080E0000))
	{
		sector = FLASH_SECTOR_6;
	}

	else if ((Address >= 0x080E0000) && (Address < 0x08100000))
	{
		sector = FLASH_SECTOR_7;
	}
	else
		sector = FLASH_SECTOR_7;

	/* BANK 2 */
	if ((Address >= 0x08100000) && (Address < 0x08120000))
	{
		sector = FLASH_SECTOR_0;
	}

	else if ((Address >= 0x08120000) && (Address < 0x08140000))
	{
		sector = FLASH_SECTOR_1;
	}

	else if ((Address >= 0x08140000) && (Address < 0x08160000))
	{
		sector = FLASH_SECTOR_2;
	}

	else if ((Address >= 0x08160000) && (Address < 0x08180000))
	{
		sector = FLASH_SECTOR_3;
	}

	else if ((Address >= 0x08180000) && (Address < 0x081A0000))
	{
		sector = FLASH_SECTOR_4;
	}

	else if ((Address >= 0x081A0000) && (Address < 0x081C0000))
	{
		sector = FLASH_SECTOR_5;
	}

	else if ((Address >= 0x081C0000) && (Address < 0x081E0000))
	{
		sector = FLASH_SECTOR_6;
	}

	else if ((Address >= 0x081E0000) && (Address < 0x08200000))
	{
		sector = FLASH_SECTOR_7;
	}
	else
		sector = FLASH_SECTOR_7;

	return sector;
}

// read from internal flash
uint8_t RTG_ReadIntFlash(uint32_t add)
{
	return (*((uint8_t*) (INTERNAL_FLASH_BASE_ADD + add)));
}

// erase sector and write data to it
uint32_t RTG_WriteIntFlash(uint32_t StartSectorAddress, uint32_t *data,
		uint16_t numberofwords)
{

	static FLASH_EraseInitTypeDef EraseInitStruct;
	uint32_t SECTORError;
	int sofar = 0;

	/* Unlock the Flash to enable the flash control register access *************/
	HAL_FLASH_Unlock();

	/* Erase the user Flash area */

	/* Get the number of sector to erase from 1st sector */

	uint32_t StartSector = GetSector(StartSectorAddress);
	uint32_t EndSectorAddress = StartSectorAddress + numberofwords * 4;
	uint32_t EndSector = GetSector(EndSectorAddress);

	/* Fill EraseInit structure*/
	EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
	EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
	EraseInitStruct.Sector = StartSector;

	// The the proper BANK to erase the Sector
	if (StartSectorAddress < 0x08100000)
		EraseInitStruct.Banks = FLASH_BANK_1;
	else
		EraseInitStruct.Banks = FLASH_BANK_2;

	EraseInitStruct.NbSectors = (EndSector - StartSector) + 1;

	if (HAL_FLASHEx_Erase(&EraseInitStruct, &SECTORError) != HAL_OK)
	{
		return HAL_FLASH_GetError();
	}

	/* Program the user Flash area 8 WORDS at a time
	 * (area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR) ***********/

	while (sofar < numberofwords)
	{
		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, StartSectorAddress,
				(uint32_t) &data[sofar]) == HAL_OK)
		{
			StartSectorAddress += 4 * FLASHWORD;  //
			sofar += FLASHWORD;
		}
		else
		{
			/* Error occurred while writing data in Flash memory*/
			return HAL_FLASH_GetError();
		}
	}

	/* Lock the Flash to disable the flash control register access (recommended
	 to protect the FLASH memory against possible unwanted operation) *********/
	HAL_FLASH_Lock();

	return 0;
}

// calculate the check sum of all flash except the last 2Byte (checksum to compare)
uint16_t RTG_IntFlashCalculateChecksum(void)
{
	uint32_t checksum = 0;

	// Calculate checksum of all internal flash except the last two Bytes
	for (uint32_t i = 0; i < (INTERNAL_FLASH_SIZE - 2); i++)
		checksum += RTG_ReadIntFlash(i);

	return (checksum & 0xFFFF);
}

//Add checksum to Internal flash last to bytes (0x081FFFFC)
void RTG_AddCecksumToIntFlash(UDP_t *debug_udp)
{
	// 256Bit (8x32Bit) array write to flash
	uint32_t data[FLASHWORD];

	// Set all data to 0xFF
	memset(data, 0xff, sizeof(data));

	// Add checksum to the last WORD of the array data
	data[FLASHWORD - 1] = ((debug_udp->eth_Rbuf[5] * 0x100
			+ debug_udp->eth_Rbuf[6]) << 16) + 0xFFFF;

	// Write checksum(16Bit) to the last to bytes of the internal flash;
	RTG_WriteIntFlash((uint32_t) (0x081FFFE0), data, FLASHWORD);
}

// Check checksum
uint8_t RTG_CheckChecksum(void)
{
	uint16_t calculateChecksum;
	uint16_t checkSum;

	// result of checksum calculation
	calculateChecksum = RTG_IntFlashCalculateChecksum();

	// read checksum from internal flash
	memcpy(&checkSum,
			(uint8_t*) (INTERNAL_FLASH_BASE_ADD + INTERNAL_FLASH_SIZE - 2), 2);

	//remark: checksum (burn in the internal Flash should
	// be the 2' compliment of calculateChecksum
	return (((calculateChecksum + checkSum) & 0xFFFF) ? 1 : 0);
}

/****************************************************************************************************/
// This test check the program in the internal Flash, it calculate the checksum of the 2MByte of the
// internal flash except last two Bytes, that last two Bytes contains the checksum for the program
// the 2' compliments of checksum, when we add this value to the calculated checksum value it should
// be '0' - PASS other value if - FAIL.
//
// INT FLASH MESSAGE STRUCTURE
// -------------------------------
// | TIME TAG(32) | Pass Fail(8) |
// -------------------------------
//
// Pass Fail register: 1 - FAIL, 0 - PASS
//
//   7   6   5   4   3   2   1     0
// ----------------------------------------
// | 0 | 0 | 0 | 0 | 0 | 0 | 0 |Pass/Fail |
// ----------------------------------------
void  RTG_INT_Flash_BIT(CIB_PBIT_INTERNAL_FLASH *mes)
{
	// Add TIME TAG
	mes->time_tag = GetTimeTag;
	// check the SW checksum
	mes->pass_or_fail = RTG_CheckChecksum();
	pPbitMes->error_bits.INT_FLASH = mes->pass_or_fail;
}
