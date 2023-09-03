/*
 * Safe_And_Arm.c
 *
 *  Created on: May 15, 2022
 *      Author: Sharon
 */

#include <bit.h>
#include <string.h>
#include <Safe_And_Arm.h>
#include <timer.h>
#include <uarts.h>

SAA_PBIT_t SAA_PBIT = { 0 };

detailed_status_message_reply_struct 	SAA_detStaMes 		= { 0 };
versions_message_reply_struct 			SAA_verMes 				= { 0 };
global_error_message_reply_struct 		SSA_errorMessage 		= { 0 };

static uint32_t RTG_Safe_And_Arm_Checksum(uint32_t inputData [], uint32_t len) {
	uint16_t bitIndex;
	uint32_t crc = 0xFFFFFFFF, i, polynomial = 0x04C11DB7, current;
	for (i = 0; i < len; i++) {
		current = inputData [i];
		crc ^= current;
		// Process all the bits in input data.
		for (bitIndex = 0; (bitIndex < 32); ++bitIndex) {
			// If the MSB for CRC == 1
			if ((crc & 0x80000000) != 0) {
				crc = ((crc << 1) ^ polynomial);
			}
			else {
				crc <<= 1;
			}
		}
	}
	return crc;
}

void RTG_DetailedStatusRxCbit( uint8_t *buffer ) {

//	SAA_CBIT_t *cbit = &(CBIT_CURRENT_BUFF_W.SAFE_AND_ARM);
	SAA_CBIT_t Cbit = {0};
	SAA_CBIT_t *cbit = &Cbit;
	detailed_status_message_reply_struct *reply = (detailed_status_message_reply_struct*) buffer;

	uint8_t crcTxBuf [68] = { 0 };
	uint32_t crc32;

	if ( !(		(reply->hdr.preamble == 0xcc6c) && (reply->hdr.sync == 0xA5)
			 && (reply->hdr.length == 0x43) && (reply->hdr.address == SAA_ADDRESS)
			 && (reply->hdr.opcode == 0x50) && (reply->state_machine > 0x13)
			 ) ) return;

	// Because we have 5 byte we need 8 byte to calculate CRC32
	memcpy(crcTxBuf, (uint8_t*) &reply->hdr.message_ID, reply->hdr.length);
	// Calculate  CRC
	crc32 = RTG_Safe_And_Arm_Checksum((uint32_t*) &crcTxBuf, (sizeof(crcTxBuf) / 4));
	if (crc32 != reply->crc32) return;

	cbit->MCU_state_machine = reply->state_machine;
	cbit->mcu_bit = reply->mcu_bit;
	cbit->hv_capacitor = reply->hv_capacitor;
	cbit->logic_dc = reply->logic_dc;
	cbit->vdd = reply->vdd;
	cbit->fpga_1_seq_fail = reply->fpga_1_seq_fail;
	cbit->fpga_2_seq_fail = reply->fpga_2_seq_fail;
	cbit->fpga_1_debug = reply->fpga_1_debug;
	cbit->fpga_2_debug = reply->fpga_2_debug;
	cbit->status_bits_1 = reply->status_bits_1;
	cbit->status_bits_2 = reply->status_bits_2;
	cbit->status_bits_3 = reply->status_bits_3;
	cbit->status_bits_4 = reply->status_bits_4;
	cbit->status_bits_5 = reply->status_bits_5;
	cbit->status_bits_6 = reply->status_bits_6;
	cbit->status_bits_7 = reply->status_bits_7;
	cbit->status_bits_8_spare = reply->status_bits_8_spare;
	cbit->TimeTag = GetTimeTag;
}

void RTG_Detailed_StatusRxPbit(uint8_t *buffer) {
//	static SAA_PBIT_t *pbit = &pPbitMes->SAFE_AND_ARM;
	static SAA_PBIT_t Pbit;
	static SAA_PBIT_t *pbit = &Pbit;
	detailed_status_message_reply_struct *reply = (detailed_status_message_reply_struct*) buffer;

	uint8_t crcTxBuf [68] = { 0 };
	uint32_t crc32;

	if ( !(	   (reply->hdr.preamble == 0xcc6c) 	&& (reply->hdr.sync == 0xA5)
			&& (reply->hdr.length == 0x43)		&& (reply->hdr.address == SAA_ADDRESS)
			&& (reply->hdr.opcode == SnA_DETAILED_STATUS_MSG)
			&& (reply->state_machine >= 0x13 )
			) ) return;
	// Because we have 5 byte we need 8 byte to calculate CRC32
	memcpy(crcTxBuf, (uint8_t*) &reply->hdr.message_ID, reply->hdr.length);

	// Calculate  CRC
	crc32 = RTG_Safe_And_Arm_Checksum((uint32_t*) &crcTxBuf, sizeof(crcTxBuf) / 4);

	if ( crc32 != reply->crc32 ) return;

	pbit->mcu_bit 				= reply->mcu_bit;
	pbit->hv_capacitor 			= reply->hv_capacitor;
	pbit->logic_dc 				= reply->logic_dc;
	pbit->vdd 					= reply->vdd;
	pbit->fpga_1_seq_fail 		= reply->fpga_1_seq_fail;
	pbit->fpga_2_seq_fail 		= reply->fpga_2_seq_fail;
	pbit->fpga_1_debug 			= reply->fpga_1_debug;
	pbit->fpga_2_debug 			= reply->fpga_2_debug;
	pbit->status_bits_1 		= reply->status_bits_1;
	pbit->status_bits_2 		= reply->status_bits_2;
	pbit->status_bits_3 		= reply->status_bits_3;
	pbit->status_bits_4 		= reply->status_bits_4;
	pbit->status_bits_5 		= reply->status_bits_5;
	pbit->status_bits_6 		= reply->status_bits_6;
	pbit->status_bits_7 		= reply->status_bits_7;
	pbit->status_bits_8_spare 	= reply->status_bits_8_spare;
	pbit->TimeTag = GetTimeTag;
}

void RTG_VersionsRxPbit(uint8_t *buffer) {
//	static SAA_PBIT_t *pbit = &pPbitMes->SAFE_AND_ARM;
	static SAA_PBIT_t Pbit ;
	static SAA_PBIT_t *pbit = &Pbit;
	versions_message_reply_struct *reply = (versions_message_reply_struct*) buffer;

	uint8_t crcTxBuf [24] = { 0 };
	uint32_t crc32;

	if ( !(    (reply->hdr.preamble == 0xcc6c) && (reply->hdr.sync == 0xA5) && (reply->hdr.length == 0x18)
			&& (reply->hdr.address == SAA_ADDRESS) && (reply->hdr.opcode == SnA_VERSIONS_MSG)
			) ) return;
	// Because we have 5 byte we need 8 byte to calculate CRC32
	memcpy(crcTxBuf, (uint8_t*) &reply->hdr.message_ID, reply->hdr.length);
	// Calculate  CRC
	crc32 = RTG_Safe_And_Arm_Checksum((uint32_t*) &crcTxBuf, sizeof(crcTxBuf) / 4);
	if (crc32 != reply->crc32) return;
	pbit->mcu_version_low = reply->mcu_version_low;
	pbit->mcu_version_high = reply->mcu_version_high;
	pbit->fpga_1_version_low = reply->fpga_1_version_low;
	pbit->fpga_1_version_high = reply->fpga_1_version_high;
	pbit->fpga_2_version_low = reply->fpga_2_version_low;
	pbit->fpga_2_version_high = reply->fpga_2_version_high;
	pbit->TimeTag = GetTimeTag;
}

void RTG_detailed_status_tx() {
	detailed_status_message_struct buffer = { 0 };

	uint8_t crcTxBuf [8] = { 0 };

	buffer.hdr.preamble = 0xcc6c;
	buffer.hdr.sync = 0xA5;
	buffer.hdr.length = 0x5;
	buffer.hdr.message_ID = 0x0000;			//check
	buffer.hdr.address = SAA_ADDRESS;				//check
	buffer.hdr.opcode = SnA_DETAILED_STATUS_MSG;
	buffer.stream_mode = 0x00;				//check

	// Because we have 5 byte we need 8 byte to calculate CRC32
	memcpy(crcTxBuf, (uint8_t*) &buffer.hdr.message_ID, 5);

	buffer.crc32 = RTG_Safe_And_Arm_Checksum((uint32_t*) &crcTxBuf, sizeof(crcTxBuf) / 4);

	sUart [UART_SAA].dataTx_length = sizeof(buffer);
	memcpy((uint8_t*) sUart [UART_SAA].TXbuf, (uint8_t*) &buffer, sUart [UART_SAA].dataTx_length);
	RTG_Uart_SendMessege(&sUart [UART_SAA]);
}

void RTG_versions_tx() {
	versions_message_struct buffer = { 0 };
	buffer.hdr.preamble = 0xcc6c;
	buffer.hdr.sync = 0xA5;
	buffer.hdr.length = 0x4;
	buffer.hdr.message_ID = 0x0000;			//check
	buffer.hdr.address = SAA_ADDRESS;				//check
	buffer.hdr.opcode = SnA_VERSIONS_MSG;
	buffer.crc32 = RTG_Safe_And_Arm_Checksum((uint32_t*) &buffer.hdr.message_ID, 1);

	sUart [UART_SAA].dataTx_length = sizeof(buffer);
	memcpy((uint8_t*) sUart [UART_SAA].TXbuf, (uint8_t*) &buffer, sUart [UART_SAA].dataTx_length);
	RTG_Uart_SendMessege(&sUart [UART_SAA]);
}

