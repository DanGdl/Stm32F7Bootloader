/*
 * Serial_FLASH.h
 *
 *  Created on: 10 Nov 2021
 *      Author: itzhaki
 */

#ifndef INC_SERIAL_FLASH_H_
#define INC_SERIAL_FLASH_H_

#include <stm32h7xx_hal_spi.h>
#include <sys/_stdint.h>

#define SERIAL_FLASH_ID {0x20,0xbb,0x21,0x10,0x44,0x00}
#define SERIAL_FLASH_ID_SIZE 8

#define MAX_BURST_SIZE 256

#define SPIFLASHHANDLER &hspi1

#define FLASH_ADDRESS_D 0 //Disable
#define FLASH_ADDRESS_E 1 //Enable

/************************** Serial flash commands define *****************/
//Software RESET Operations
#define RESET_ENABLE 0x66
#define RESET_MEMORY 0x99

//READ ID Operations
#define READ_ID1 0x9E
#define READ_ID2 0x9F
#define MULTIPLE_I_O_READ_ID 0xAF
#define READ_SERIAL_FLASH_DISCOVERY_PARAMETER 0x5A

//READ MEMORY Operations
#define READ 0x03
#define FAST_READ 0x0B

//READ MEMORY Operations with 4-Byte Address
#define ADD_4_BYTE_READ 0x13
#define ADD_4_BYTE_FAST_READ 0x0C

//WRITE Operations
#define WRITE_ENABLE 0x06
#define WRITE_DISABLE 0x04

//READ REGISTER Operations
#define READ_STATUS_REGISTER 0x05
#define READ_FLAG_STATUS_REGISTER 0x70
#define READ_NONVOLATILE_CONFIGURATION_REGISTER 0xB5
#define READ_VOLATILE_CONFIGURATION_REGISTER 0x85
#define READ_ENHANCED_VOLATILE_CONFIGURATION_REGISTER 0x65
#define READ_EXTENDED_ADDRESS_REGISTER 0xC8
#define READ_GENERAL_PURPOSE_READ_REGISTER 0x96

//WRITE REGISTER Operations
#define WRITE_STATUS_REGISTER 0x01
#define WRITE_NONVOLATILE_CONFIGURATION_REGISTER 0xB1
#define WRITE_VOLATILE_CONFIGURATION_REGISTER 0x81
#define WRITE_ENHANCED_VOLATILE_CONFIGURATION_REGISTER 0x61
#define WRITE_EXTENDED_ADDRESS_REGISTER 0xC5

//CLEAR FLAG STATUS REGISTER Operation
#define CLEAR_FLAG_STATUS_REGISTER 0x50

//PROGRAM Operations
#define PAGE_PROGRAM 0x02

//PROGRAM Operations with 4-Byte Address
#define ADD_4_BYTE_PAGE_PROGRAM 0x12

//ERASE Operations
#define SIZE_32KB_SUBSECTOR_ERASE 0x52
#define SIZE_4KB_SUBSECTOR_ERASE 0x20
#define SECTOR_ERASE 0xD8
#define DIE_ERASE 0xC4

//ERASE Operations with 4-Byte Address
#define ADD_4_BYTE_SECTOR_ERASE 0xDC
#define ADD_4_BYTE_4KB_SUBSECTOR_ERASE 0x21
#define ADD_4_BYTE_32KB_SUBSECTOR_ERASE 0x5C

//SUSPEND/RESUME Operations
#define PROGRAM_ERASE_SUSPEND 0x75
#define PROGRAM_ERASE_RESUME 0x7A

//ONE-TIME PROGRAMMABLE (OTP) Operations
#define READ_OTP_ARRAY 0x4B
#define PROGRAM_OTP_ARRAY 0x42

//4-BYTE ADDRESS MODE Operations
#define ENTER_4_BYTE_ADDRESS_MODE 0xB7
#define EXIT_4_BYTE_ADDRESS_MODE 0xE9

//Deep Power-Down Operations
#define ENTER_DEEP_POWER_DOWN 0xB9
#define RELEASE_FROM_DEEP_POWERDOWN 0xAB

//ADVANCED SECTOR PROTECTION Operations
#define READ_SECTOR_PROTECTION 0x2D
#define PROGRAM_SECTOR_PROTECTION 0x2C
#define READ_VOLATILE_LOCK_BITS 0xE8
#define WRITE_VOLATILE_LOCK_BITS 0xE5
#define READ_NONVOLATILE_LOCK_BITS 0xE2
#define WRITE_NONVOLATILE_LOCK_BITS 0xE3
#define ERASE_NONVOLATILE_LOCK_BITS 0xE4
#define READ_GLOBAL_FREEZE_BIT 0xA7
#define WRITE_GLOBAL_FREEZE_BIT 0xA6
#define READ_PASSWORD 0x27
#define WRITE_PASSWORD 0x28
#define UNLOCK_PASSWORD 0x29

//ADVANCED SECTOR PROTECTION Operations with 4-Byte Address
#define ADD_4_BYTE_READ_VOLATILE_LOCK_BITS 0xE0
#define ADD_4_BYTE_WRITE_VOLATILE_LOCK_BITS 0xE1

//ADVANCED FUNCTION INTERFACE Operations
#define INTERFACE_ACTIVATION 0x9B
#define CYCLIC_REDUNDANCY_CHECK1 0x9B
#define CYCLIC_REDUNDANCY_CHECK2 0x27

//#pragma pack (push,1)
//
//typedef struct
//{
//	uint32_t TimeTag;
//	uint8_t ID[6];
//	uint8_t Error;
//} EXT_Flash_t;
//
//#pragma pack ( pop )

/********************** Serial flash commands define end *******************/

/************************** Serial flash commands functions *****************/
/******* below is the list of commands that supported by the hardware *******/
/******************************************************************/
void RTG_SendToFlash( uint8_t cmd, uint8_t addEnable,
		uint32_t add, uint8_t *TXBuff, uint16_t size);

void RTG_RecivefromFlash( uint8_t cmd,
		uint8_t addEnable, uint32_t add, uint8_t *TXBuff, uint8_t *RXBuff,
		uint16_t size);
void RTG_intFlash(void);
void RTG_FlashSoftwareReset(void);

void RTG_FlashReadID(uint8_t *RXBuff);
uint8_t RTG_FlashRead_ID(uint8_t *pID);

void RTG_FlashMultipeReadID(uint8_t *RXBuff);
void RTG_FlashReadDiscoveryParm(uint32_t add, uint8_t *TXBuff, uint8_t *RXBuff,
		uint16_t size);
void RTG_FlashRead(uint32_t add, uint8_t *TXBuff, uint8_t *RXBuff,
		uint16_t size);
void RTG_FlashFastRead(uint32_t add, uint8_t *TXBuff, uint8_t *RXBuff,
		uint16_t size);
void RTG_Flash4ByteRead(uint32_t add, uint8_t *TXBuff, uint8_t *RXBuff,
		uint16_t size);
void RTG_Flash4ByteFastRead(uint32_t add, uint8_t *TXBuff, uint8_t *RXBuff,
		uint16_t size);
void RTG_FlashWriteEnable(void);
void RTG_FlashWriteDisable(void);
void RTG_FlashReadStatusRegister(uint8_t *RXBuff);
void RTG_FlashReadFlagStatusRegister(uint8_t *RXBuff);
void RTG_FlashReadNonVolatileConfigurationRegister(uint8_t *RXBuff);
void RTG_FlashReadVolatileConfigurationRegister(uint8_t *RXBuff);
void RTG_FlashReadEnhancedVollatileConfigurationRegister(uint8_t *RXBuff);
void RTG_FlashReadEnhancedAddressRegister(uint8_t *RXBuff);
void RTG_FlashReadGeneralPurposeRegister(uint8_t *RXBuff);
void RTG_FlashWriteStatusRegister(uint16_t data);
void RTG_FlashWriteNonVolatileConfigurationRegister(uint16_t data);
void RTG_FlashWriteVolatileConfigurationRegister(uint16_t data);
void RTG_FlashWriteEnhancedVollatileConfigurationRegister(uint16_t data);
void RTG_FlashWriteEnhancedAddressRegisterRegister(uint16_t data);
void RTG_FlashClearFlageStatusRegister(void);
void RTG_FlashPageProgram(uint32_t add, uint8_t *TXBuff, uint16_t size);
void RTG_Flash4BytePageProgram(uint32_t add, uint8_t *TXBuff, uint16_t size);
void RTG_Flash32KBSubectorErase(uint32_t add);
void RTG_Flash4KBSubectorErase(uint32_t add);
void RTG_FlasSectorErase(uint32_t add);
void RTG_FlasDieErase(uint32_t add);
void RTG_Flas4ByteSectorErase(uint32_t add);
void RTG_Flas4Byte4KSectorErase(uint32_t add);
void RTG_Flas4Byte32KSectorErase(uint32_t add);
void RTG_FlasProgramEraseSuspend(void);
void RTG_FlasProgramEraseResume(void);
void RTG_FlashReadOtpArray(uint32_t add, uint8_t *TXBuff, uint16_t size);
void RTG_FlashProgramOtpArray(uint32_t add, uint8_t *TXBuff, uint16_t size);
void RTG_FlasEnter4ByteMode(void);
void RTG_FlasExit4ByteMode(void);
void RTG_FlasEnterDeepPowerDown(void);
void RTG_FlasReleasFromDeepPowerDown(void);
void RTG_FlashReadSectorProtection(uint8_t *RXBuff);
void RTG_FlashProgramSectorProtection(uint16_t data);
void RTG_FlashReadVolatileLockBits(uint32_t add, uint8_t data);
void RTG_FlashWriteVolatileLockBits(uint32_t add, uint8_t data);
void RTG_FlashReadNonvolatileLockBits(uint32_t add, uint8_t data);
void RTG_FlashWriteNonvolatileLockBits(uint32_t add);
void RTG_FlashEraseNonvolatileLockBits(void);
void RTG_FlashReadGlobalFreezeBits(uint8_t data);
void RTG_FlashWriteGlobalFreezeBits(void);
void RTG_FlashReadPassword(uint8_t *password);
void RTG_FlashWritePassword(uint8_t *password);
void RTG_FlashUnlockPassword(uint8_t *password);
void RTG_Flash4ByteReadVolatileLockBit(uint32_t add, uint8_t data);
void RTG_Flash4ByteWriteVolatileLockBit(uint32_t add, uint8_t data);
void RTG_FlashInterfaceActivation(void);
/************************** Serial flash commands functions end *****************/

uint16_t RTG_EXT_FLASH_bit(CIB_PBIT_EXTERNAL_FLASH *mes);

#endif /* INC_SERIAL_FLASH_H_ */
