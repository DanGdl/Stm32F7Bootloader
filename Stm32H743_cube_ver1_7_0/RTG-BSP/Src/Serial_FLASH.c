/*
 * Serial_FLASH.c
 *
 *  Created on: 10 Nov 2021
 *      Author: itzhaki
 */

#ifndef SRC_SERIAL_FLASH_C_
#define SRC_SERIAL_FLASH_C_

#include <bit.h>
#include <main.h>
#include <RTG_main.h>
#include <spi.h>
#include <stdlib.h>
#include <stm32h7xx.h>
#include <stm32h7xx_hal_def.h>
#include <string.h>
#include <Serial_FLASH.h>
#include <timer.h>

extern SPI_HandleTypeDef hspi1;
extern HSPI sSpi1;

char flashFlag = 0;

char flasherrFlag = 0;

uint8_t flashIDbuf [24];

/******************************* static functions ************************************/

static void RTG_FlashResetEnable(void) {
	RTG_SendToFlash( RESET_ENABLE, FLASH_ADDRESS_D, 0, 0, 0);
}

static void RTG_FlashResetMemory(void) {
	RTG_SendToFlash( RESET_MEMORY, FLASH_ADDRESS_D, 0, 0, 0);
}

static uint8_t RTG_checkIfDummyNeed(uint8_t cmd) {
	if ((cmd == READ_SERIAL_FLASH_DISCOVERY_PARAMETER) || (cmd == FAST_READ) || (cmd == ADD_4_BYTE_FAST_READ)
			|| (cmd == READ_GENERAL_PURPOSE_READ_REGISTER) || (cmd == READ_OTP_ARRAY)) return 1;
	else return 0;
}

static uint8_t* RTG_convertU16ToU8(uint16_t data) {
	static uint8_t temp [2];

	temp [0] = (data >> 8) & 0xFF;
	temp [1] = data & 0xFF;

	return temp;
}
/***************************** static functions end **********************************/

void RTG_SendToFlash( uint8_t cmd, uint8_t addEnable, uint32_t add, uint8_t *TXBuff,
		uint16_t size) {
	uint16_t sendSize = 1;     // 1 for cmd
	uint8_t *Data;
	HAL_StatusTypeDef error;

	// Check burst max size
	if (size > MAX_BURST_SIZE) return;

	// 4-byte address
	if (addEnable) sendSize += 4;

	//check if this command need dummy cycles
	if (RTG_checkIfDummyNeed(cmd)) sendSize++;

	Data = (uint8_t*) malloc(size + sendSize + 1);
	if (Data == NULL) return;

	// Add command and address to the buffer
	Data [0] = cmd;
	if (addEnable == FLASH_ADDRESS_E) {
		Data [1] = (add >> 24) & 0xFF;
		Data [2] = (add >> 16) & 0xFF;
		Data [3] = (add >> 8) & 0xFF;
		Data [4] = add & 0xFF;

		Data [5] = 0x00;
	}
	else Data [1] = 0x00;

	free(Data);

	if (size > 0) {
		memcpy(Data + sendSize, TXBuff, size);
		memcpy(TXBuff, Data, size);
	}

//	error = HAL_SPI_Transmit_DMA(SPIFLASHHANDLER, Data, size + sendSize);
	error = HAL_SPI_Transmit_IT(SPIFLASHHANDLER, Data, size + sendSize);

	if ((error != HAL_OK) || (hspi1.ErrorCode)) {
		RTG_SPI_Error(&sSpi1);
		return;
	}
}

void RTG_RecivefromFlash_(
	uint8_t cmd, uint8_t addEnable,
	uint32_t add, uint8_t *TXBuff,
	uint8_t *RXBuffff, uint16_t size) {
	uint16_t sendSize = 1;     // 1 for cmd
	uint8_t Data[100]={0};
	HAL_StatusTypeDef error;

	// Check burst max size
	if (size > MAX_BURST_SIZE) return;

	// 4-byte address
	if (addEnable) sendSize += 4;

	//check if this command need dummy cycles
	if (RTG_checkIfDummyNeed(cmd)) sendSize++;

	// Add command and address to the buffer
	Data [0] = cmd;
	if (addEnable == FLASH_ADDRESS_E) {
		Data [1] = (add >> 24) & 0xFF;
		Data [2] = (add >> 16) & 0xFF;
		Data [3] = (add >> 8) & 0xFF;
		Data [4] = add & 0xFF;

		Data [5] = 0x00;
	}
	else Data [1] = 0x00;

	if (size > 0) {
		memcpy(Data + sendSize, TXBuff, size);
		memcpy(TXBuff, Data, size);
	}
	else return;


	error = HAL_SPI_TransmitReceive_IT(SPIFLASHHANDLER, TXBuff, sSpi1.RxBuff, size + sendSize);
	if ((error != HAL_OK) || (hspi1.ErrorCode)) {
		RTG_SPI_Error(&sSpi1);
		return;
	}
	RT_DELAY_MS(1);
}

void RTG_RecivefromFlash(
		uint8_t cmd, uint8_t addEnable,
		uint32_t add, uint8_t *TXBuff,
		uint8_t *RXBuffff, uint16_t size) {
	uint16_t sendSize = 1;     // 1 for cmd
	uint8_t *Data;
	HAL_StatusTypeDef error;

	// Check burst max size
	if (size > MAX_BURST_SIZE) return;

	// 4-byte address
	if (addEnable) sendSize += 4;

	//check if this command need dummy cycles
	if (RTG_checkIfDummyNeed(cmd)) sendSize++;

	Data = (uint8_t*) malloc(size + sendSize + 1);
	if (Data == NULL) return;

	// Add command and address to the buffer
	Data [0] = cmd;
	if (addEnable == FLASH_ADDRESS_E) {
		Data [1] = (add >> 24) & 0xFF;
		Data [2] = (add >> 16) & 0xFF;
		Data [3] = (add >> 8) & 0xFF;
		Data [4] = add & 0xFF;

		Data [5] = 0x00;
	}
	else Data [1] = 0x00;

	if (size > 0) {
		memcpy(Data + sendSize, TXBuff, size);
		memcpy(TXBuff, Data, size);
	}
	else return;

	free(Data);

	error = HAL_SPI_TransmitReceive(SPIFLASHHANDLER, TXBuff, sSpi1.RxBuff, size + sendSize,0xffff);
	if ((error != HAL_OK) || (hspi1.ErrorCode)) {
		RTG_SPI_Error(&sSpi1);
		return;
	}
}

/*************************************************************************************/
void RTG_intFlash(void) {
	uint8_t RXBuff [20];

	SPI_HandleTypeDef *hspi = SPIFLASHHANDLER;

	// Configure SPI1 to transmit in 8bit mode
	MODIFY_REG(hspi->Instance->CFG1, 0x1F, 0x7);

	// Reset serial FLASH
	RTG_FlashSoftwareReset();

	// Read Serial FLASH ID
	RTG_FlashReadID(RXBuff);
}

void RTG_FlashSoftwareReset(void) {
	RTG_FlashResetEnable();

	// must be at least 50ns after reset enable
//	HAL_Delay(1);
	RT_DELAY_MS(1);

	RTG_FlashResetMemory();

	// must be at least 40ns after reset memory
//	HAL_Delay(1);
	RT_DELAY_MS(1);
}

void RTG_FlashReadID(uint8_t *RXBuff) {
	uint8_t txbuf [6] = { 0 };
	RTG_RecivefromFlash( READ_ID1, FLASH_ADDRESS_D, 0, txbuf, RXBuff, 6);
}
// evgeny change

uint8_t RTG_FlashRead_ID(uint8_t *pID) {
	//**************************************************************
	uint16_t sendSize = 9; // 1 for cmd
	HAL_StatusTypeDef error;
	uint8_t  TXBuff[10];

	TXBuff[0] = READ_ID1;
	TXBuff[1] = 0x00;
	sSpi1.Rx_ready = 0;
	error = HAL_SPI_TransmitReceive_IT(SPIFLASHHANDLER, TXBuff, sSpi1.RxBuff, sendSize);

	if (( error != HAL_OK)|| (hspi1.ErrorCode))
	{
			RTG_SPI_Error(&sSpi1);
			return 0;
	}
	RT_DELAY_TIC(1);
	if(sSpi1.Rx_ready != 1) return 0;
	memcpy(pID, sSpi1.RxBuff + 1, 6);
	return 6;
	//**************************************************************
}

void RTG_FlashMultipeReadID(uint8_t *RXBuff) {
	uint8_t txbuf [20] = { 0 };
	RTG_RecivefromFlash( MULTIPLE_I_O_READ_ID, FLASH_ADDRESS_D, 0, txbuf, RXBuff, 20);
}

void RTG_FlashReadDiscoveryParm(uint32_t add, uint8_t *TXBuff, uint8_t *RXBuff, uint16_t size) {
	RTG_RecivefromFlash( READ_SERIAL_FLASH_DISCOVERY_PARAMETER,
	FLASH_ADDRESS_E, add, TXBuff, RXBuff, size);
}

void RTG_FlashRead(uint32_t add, uint8_t *TXBuff, uint8_t *RXBuff, uint16_t size) {
	RTG_RecivefromFlash( READ, FLASH_ADDRESS_E, add, TXBuff, RXBuff, size);
}

void RTG_FlashFastRead(uint32_t add, uint8_t *TXBuff, uint8_t *RXBuff, uint16_t size) {
	RTG_RecivefromFlash( FAST_READ, FLASH_ADDRESS_E, add, TXBuff, RXBuff, size);
}

void RTG_Flash4ByteRead(uint32_t add, uint8_t *TXBuff, uint8_t *RXBuff, uint16_t size) {
	RTG_RecivefromFlash( ADD_4_BYTE_READ, FLASH_ADDRESS_E, add, TXBuff, RXBuff, size);
}

void RTG_Flash4ByteFastRead(uint32_t add, uint8_t *TXBuff, uint8_t *RXBuff, uint16_t size) {
	RTG_RecivefromFlash( ADD_4_BYTE_FAST_READ, FLASH_ADDRESS_E, add, TXBuff, RXBuff, size);
}

void RTG_FlashWriteEnable() {
	RTG_SendToFlash( WRITE_ENABLE, FLASH_ADDRESS_D, 0, 0, 0);
}

void RTG_FlashWriteDisable() {
	RTG_SendToFlash( WRITE_DISABLE, FLASH_ADDRESS_D, 0, 0, 0);
}

void RTG_FlashReadStatusRegister(uint8_t *RXBuff) {
	uint8_t temp = 0;
	RTG_RecivefromFlash( READ_STATUS_REGISTER, FLASH_ADDRESS_D, 0, &temp, RXBuff, 1);
}

void RTG_FlashReadFlagStatusRegister(uint8_t *RXBuff) {
	uint8_t temp = 0;
	RTG_RecivefromFlash( READ_FLAG_STATUS_REGISTER,
	FLASH_ADDRESS_D, 0, &temp, RXBuff, 1);
}

void RTG_FlashReadNonVolatileConfigurationRegister(uint8_t *RXBuff) {
	uint8_t temp = 0;
	RTG_RecivefromFlash(
	READ_NONVOLATILE_CONFIGURATION_REGISTER, FLASH_ADDRESS_D, 0, &temp, RXBuff, 1);
}

void RTG_FlashReadVolatileConfigurationRegister(uint8_t *RXBuff) {
	uint8_t temp = 0;
	RTG_RecivefromFlash( READ_VOLATILE_CONFIGURATION_REGISTER,
	FLASH_ADDRESS_D, 0, &temp, RXBuff, 1);
}

void RTG_FlashReadEnhancedVollatileConfigurationRegister(uint8_t *RXBuff) {
	uint8_t temp = 0;
	RTG_RecivefromFlash(
	READ_ENHANCED_VOLATILE_CONFIGURATION_REGISTER, FLASH_ADDRESS_D, 0, &temp, RXBuff, 1);
}

void RTG_FlashReadEnhancedAddressRegister(uint8_t *RXBuff) {
	uint8_t temp = 0;
	RTG_RecivefromFlash( READ_EXTENDED_ADDRESS_REGISTER,
	FLASH_ADDRESS_D, 0, &temp, RXBuff, 1);
}

void RTG_FlashReadGeneralPurposeRegister(uint8_t *RXBuff) {
	uint8_t temp [8] = { 0 };
	RTG_RecivefromFlash( READ_GENERAL_PURPOSE_READ_REGISTER,
	FLASH_ADDRESS_D, 0, temp, RXBuff, 8);
}

void RTG_FlashWriteStatusRegister(uint16_t data) {
	RTG_SendToFlash( WRITE_STATUS_REGISTER, FLASH_ADDRESS_D, 0, RTG_convertU16ToU8(data), 2);
}

void RTG_FlashWriteNonVolatileConfigurationRegister(uint16_t data) {
	RTG_SendToFlash( WRITE_NONVOLATILE_CONFIGURATION_REGISTER,
	FLASH_ADDRESS_D, 0, RTG_convertU16ToU8(data), 2);
}

void RTG_FlashWriteVolatileConfigurationRegister(uint16_t data) {
	RTG_SendToFlash( WRITE_VOLATILE_CONFIGURATION_REGISTER,
	FLASH_ADDRESS_D, 0, RTG_convertU16ToU8(data), 2);
}

void RTG_FlashWriteEnhancedVollatileConfigurationRegister(uint16_t data) {
	RTG_SendToFlash(
	WRITE_ENHANCED_VOLATILE_CONFIGURATION_REGISTER, FLASH_ADDRESS_D, 0, RTG_convertU16ToU8(data), 2);
}

void RTG_FlashWriteEnhancedAddressRegisterRegister(uint16_t data) {
	RTG_SendToFlash( WRITE_EXTENDED_ADDRESS_REGISTER,
	FLASH_ADDRESS_D, 0, RTG_convertU16ToU8(data), 2);
}

void RTG_FlashClearFlageStatusRegister(void) {
	RTG_SendToFlash( CLEAR_FLAG_STATUS_REGISTER,
	FLASH_ADDRESS_D, 0, 0, 0);
}

void RTG_FlashPageProgram(uint32_t add, uint8_t *TXBuff, uint16_t size) {
	if (size > 256) return;

	RTG_SendToFlash( PAGE_PROGRAM, FLASH_ADDRESS_E, add, TXBuff, size);
}

void RTG_Flash4BytePageProgram(uint32_t add, uint8_t *TXBuff, uint16_t size) {
	if (size > 256) return;

	RTG_SendToFlash( ADD_4_BYTE_PAGE_PROGRAM, FLASH_ADDRESS_E, add, TXBuff, size);
}

void RTG_Flash32KBSubectorErase(uint32_t add) {
	RTG_SendToFlash( SIZE_32KB_SUBSECTOR_ERASE, FLASH_ADDRESS_E, add, 0, 0);
}

void RTG_Flash4KBSubectorErase(uint32_t add) {
	RTG_SendToFlash( SIZE_4KB_SUBSECTOR_ERASE, FLASH_ADDRESS_E, add, 0, 0);
}

void RTG_FlasSectorErase(uint32_t add) {
	RTG_SendToFlash( SECTOR_ERASE, FLASH_ADDRESS_E, add, 0, 0);
}

void RTG_FlasDieErase(uint32_t add) {
	RTG_SendToFlash( DIE_ERASE, FLASH_ADDRESS_E, add, 0, 0);
}

void RTG_Flas4ByteSectorErase(uint32_t add) {
	RTG_SendToFlash( ADD_4_BYTE_SECTOR_ERASE, FLASH_ADDRESS_E, add, 0, 0);
}

void RTG_Flas4Byte4KSectorErase(uint32_t add) {
	RTG_SendToFlash( ADD_4_BYTE_4KB_SUBSECTOR_ERASE,
	FLASH_ADDRESS_E, add, 0, 0);
}

void RTG_Flas4Byte32KSectorErase(uint32_t add) {
	RTG_SendToFlash( ADD_4_BYTE_32KB_SUBSECTOR_ERASE,
	FLASH_ADDRESS_E, add, 0, 0);
}

void RTG_FlasProgramEraseSuspend(void) {
	RTG_SendToFlash( PROGRAM_ERASE_SUSPEND, FLASH_ADDRESS_D, 0, 0, 0);
}

void RTG_FlasProgramEraseResume(void) {
	RTG_SendToFlash( PROGRAM_ERASE_RESUME, FLASH_ADDRESS_D, 0, 0, 0);
}

void RTG_FlashReadOtpArray(uint32_t add, uint8_t *RXBuff, uint16_t size) {
	uint8_t temp [64] = { 0 };

	if (size > 64) return;

	RTG_RecivefromFlash( READ_OTP_ARRAY, FLASH_ADDRESS_E, add, temp, RXBuff, size);
}

void RTG_FlashProgramOtpArray(uint32_t add, uint8_t *TXBuff, uint16_t size) {
	if (size > 64) return;

	RTG_SendToFlash( PROGRAM_OTP_ARRAY, FLASH_ADDRESS_E, add, TXBuff, size);
}

void RTG_FlasEnter4ByteMode(void) {
	RTG_SendToFlash( ENTER_4_BYTE_ADDRESS_MODE, FLASH_ADDRESS_D, 0, 0, 0);
}

void RTG_FlasExit4ByteMode(void) {
	RTG_SendToFlash( EXIT_4_BYTE_ADDRESS_MODE, FLASH_ADDRESS_D, 0, 0, 0);
}

void RTG_FlasEnterDeepPowerDown(void) {
	RTG_SendToFlash( ENTER_DEEP_POWER_DOWN, FLASH_ADDRESS_D, 0, 0, 0);
}

void RTG_FlasReleasFromDeepPowerDown(void) {
	RTG_SendToFlash( RELEASE_FROM_DEEP_POWERDOWN,
	FLASH_ADDRESS_D, 0, 0, 0);
}

void RTG_FlashReadSectorProtection(uint8_t *RXBuff) {
	uint8_t temp [] = { 0, 0 };

	RTG_RecivefromFlash( READ_SECTOR_PROTECTION,
	FLASH_ADDRESS_D, 0, temp, RXBuff, 2);
}

void RTG_FlashProgramSectorProtection(uint16_t data) {
	RTG_SendToFlash( PROGRAM_SECTOR_PROTECTION, FLASH_ADDRESS_D, 0, RTG_convertU16ToU8(data), 2);
}

void RTG_FlashReadVolatileLockBits(uint32_t add, uint8_t data) {
	uint8_t temp = 0;

	RTG_RecivefromFlash( READ_VOLATILE_LOCK_BITS,
	FLASH_ADDRESS_E, add, &temp, &data, 1);
}

void RTG_FlashWriteVolatileLockBits(uint32_t add, uint8_t data) {
	RTG_SendToFlash( WRITE_VOLATILE_LOCK_BITS, FLASH_ADDRESS_E, add, &data, 1);
}

void RTG_FlashReadNonvolatileLockBits(uint32_t add, uint8_t data) {
	uint8_t temp = 0;

	RTG_RecivefromFlash( READ_NONVOLATILE_LOCK_BITS,
	FLASH_ADDRESS_E, add, &temp, &data, 1);
}

void RTG_FlashWriteNonvolatileLockBits(uint32_t add) {
	RTG_SendToFlash( WRITE_NONVOLATILE_LOCK_BITS,
	FLASH_ADDRESS_E, add, 0, 0);
}

void RTG_FlashEraseNonvolatileLockBits(void) {
	RTG_SendToFlash( ERASE_NONVOLATILE_LOCK_BITS,
	FLASH_ADDRESS_D, 0, 0, 0);
}

void RTG_FlashReadGlobalFreezeBits(uint8_t data) {
	uint8_t temp = 0;

	RTG_RecivefromFlash( READ_GLOBAL_FREEZE_BIT,
	FLASH_ADDRESS_D, 0, &temp, &data, 1);
}

void RTG_FlashWriteGlobalFreezeBits(void) {
	RTG_SendToFlash( WRITE_GLOBAL_FREEZE_BIT, FLASH_ADDRESS_D, 0, 0, 0);
}

void RTG_FlashReadPassword(uint8_t *password) {
	uint8_t temp [8] = { 0 };

	RTG_RecivefromFlash( READ_PASSWORD, FLASH_ADDRESS_D, 0, temp, password, 8);
}

void RTG_FlashWritePassword(uint8_t *password) {
	RTG_SendToFlash( WRITE_PASSWORD, FLASH_ADDRESS_D, 0, password, 8);
}

void RTG_FlashUnlockPassword(uint8_t *password) {
	RTG_SendToFlash( UNLOCK_PASSWORD, FLASH_ADDRESS_D, 0, password, 8);
}

void RTG_Flash4ByteReadVolatileLockBit(uint32_t add, uint8_t data) {
	uint8_t temp = 0;
	RTG_RecivefromFlash( ADD_4_BYTE_READ_VOLATILE_LOCK_BITS,
	FLASH_ADDRESS_E, add, &temp, &data, 1);
}

void RTG_Flash4ByteWriteVolatileLockBit(uint32_t add, uint8_t data) {
	RTG_SendToFlash( ADD_4_BYTE_WRITE_VOLATILE_LOCK_BITS,
	FLASH_ADDRESS_E, add, &data, 1);
}

void RTG_FlashInterfaceActivation(void) {
	RTG_SendToFlash( INTERFACE_ACTIVATION, FLASH_ADDRESS_D, 0, 0, 0);
}

//#define FLASH_DEBUG_BIT

/****************************************************************************************************/
// This test check the EXT FLASH communication with the device by reading and comparing the DEVICE ID
//
// EXT FLASH MESSAGE STRUCTURE
// ----------------------------------------------------------------------------------------------------------->
// | Time Tag(32) | Pass Fail(8) | DEV ID BYTE 1(8) | DEV ID BYTE 2(8) | DEV ID BYTE 3(8) | DEV ID BYTE 4(8) |
// ----------------------------------------------------------------------------------------------------------->
// >-----------------------------------
//   DEV ID BYTE 5(8) | DEV ID BYTE 6 |
// >-----------------------------------
//
// Pass Fail register: 1 - FAIL, 0 - PASS
//
//   7   6   5   4   3   2   1       0
// ------------------------------------------
// |  0 | 0 | 0 | 0 | 0 | 0 | 0 | Pass/Fail |
// ------------------------------------------
void RTG_intFlash_(uint8_t *ret) {
	uint8_t RXBuff [20];

	SPI_HandleTypeDef *hspi = SPIFLASHHANDLER;

	// Configure SPI1 to transmit in 8bit mode
	MODIFY_REG(hspi->Instance->CFG1, 0x1F, 0x7);

	// Reset serial FLASH
	RTG_FlashSoftwareReset();

	// Read Serial FLASH ID
	RTG_FlashReadID(RXBuff);
	memcpy(ret,RXBuff,6);
}

uint16_t RTG_EXT_FLASH_bit(CIB_PBIT_EXTERNAL_FLASH *mes) {
	uint8_t ID [] = SERIAL_FLASH_ID;
	uint16_t IDSize = sizeof(ID);

	memset(mes->dev_id,0xff,6);
	// Read Flash ID

	// Add TIME TAG
	mes->time_tag = GetTimeTag;
	if(!RTG_FlashRead_ID(mes->dev_id))
	{
		mes->pass_or_fail = 1;
	}
	else{
		for (int i = 0; i < IDSize; i++) {
			// Compare DEVICE ID received
	//		if (ID [i] != mes->dev_id [i]) {
			if ( 0xff == mes->dev_id [i] ) {
				mes->pass_or_fail = 1;     //ERROR
				break;
			}
		}
	}
	pPbitMes->error_bits.EXT_FLASH = mes->pass_or_fail;
	return IDSize;     // numbers of bytes
}

#endif /* SRC_SERIAL_FLASH_C_ */
