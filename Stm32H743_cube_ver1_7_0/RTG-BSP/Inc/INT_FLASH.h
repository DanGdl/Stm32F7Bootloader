/*
 * INT_FLASH.h
 *
 *  Created on: 21 Feb 2022
 *      Author: itzhaki
 */

#ifndef INC_INT_FLASH_H_
#define INC_INT_FLASH_H_

#include <eth_rtg.h>
//#include <sys/_stdint.h>
#include <CIB_Protocol.h>

#define INTERNAL_FLASH_BASE_ADD (uint8_t*)(0x8000000)
#define INTERNAL_FLASH_SIZE 	(0x200000) // 0x200000

#define FLASHWORD	8
#pragma pack (push,1)

//typedef struct
//{
//	uint32_t TimeTag;
//	uint8_t error;
//} BIT_INT_FLASH_t;

#pragma pack ( pop )

uint8_t RTG_ReadIntFlash(uint32_t add);
uint32_t RTG_WriteIntFlash(uint32_t StartSectorAddress, uint32_t *data,
		uint16_t numberofwords);
uint16_t RTG_IntFlashCalculateChecksum(void);
void RTG_AddCecksumToIntFlash(UDP_t *debug_udp);
uint8_t RTG_CheckChecksum(void);
void RTG_INT_Flash_BIT(CIB_PBIT_INTERNAL_FLASH *mes);

#endif /* INC_INT_FLASH_H_ */
