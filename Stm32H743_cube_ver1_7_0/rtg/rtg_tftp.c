#include "settings.h"
#ifndef TFTP_CLIENT


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <ctype.h>
#include <pbuf.h>

#include "lwip.h"
#include "tftp.h"
#include "util.h"
#include "usart.h"
#include "flash.h"
#include "rtg_main.h"
#include "tftp_server.h"
#include "parser_intel_hex.h"


extern struct netif gnetif;
extern UART_HandleTypeDef huart1;


void* tftp_open(const char* fname, const char* mode, u8_t write);
void tftp_close(void* handle);
int tftp_read(void* handle, void* buf, int bytes);
int tftp_write(void* handle, struct pbuf* p);


const struct tftp_context ctx = {
	.open = tftp_open,
	.close = tftp_close,
	.read = tftp_read,
	.write = tftp_write,
};


typedef struct TftpFileHandle {
	char mode[9];
	uint16_t idx_block;
} TftpFileHandle_t;

TftpFileHandle_t f_handle = { 0 };



void rtg_main(void) {
	err_t status = tftp_init(&ctx);
	if (status != ERR_OK) {
		printf("Failed to setup TFTP server\n\r");
	}
	printf("TFTP server running\r\n");
	// BOOT_ADD
	uint32_t ticks_prev = 0;
	const uint32_t diff = pdMS_TO_TICKS(1000);
	while(1) {
		ethernetif_input(&gnetif);
		sys_check_timeouts();

		uint32_t ticks = osKernelGetTickCount();
		if (diff >= (ticks - ticks_prev)) {
			char address[IP4ADDR_STRLEN_MAX] = { '\0' };
			printf("Bootloader to flash a program on card. IP: %s, port 69\r\n",
					ip4addr_ntoa_r(&gnetif.ip_addr, address, IP4ADDR_STRLEN_MAX));
			ticks_prev = ticks;
		}
	}

	tftp_cleanup();
}



void tftp_close(void* handle) {
	printf("tftp_close\r\n");
	memset(handle, 0, sizeof(*handle));
}

/**
* Open file for read/write.
* @param fname Filename
* @param mode Mode string from TFTP RFC 1350 (netascii, octet, mail)
* @param write Flag indicating read (0) or write (!= 0) access
* @returns File handle supplied to other functions
*/
void* tftp_open(const char* fname, const char* mode, u8_t write) {
	printf("tftp_open\r\n");
	if (!write) {
		// not supported
		return NULL;
	}
	memcpy(f_handle.mode, mode, strlen(mode));
	f_handle.idx_block = 0;
	return &f_handle;
}

int tftp_read(void* handle, void* buf, int bytes) {
	return -1; // not supported
}


void handle_hex_command(const IntelHexData_t* const command) {
	static uint16_t address_base = 0;
	if (command->operation == 0) {
		uint32_t address = 0;
		memcpy(&address, &command->address, sizeof(command->address));
		address |= (address_base << 16);

		printf("Write to address to 0x%08lX: 0x", address);
		for (int i = 0; i < command->size; i++) {
			printf("%02X", command->data[i]);
		}
		printf("\r\n");
		uint32_t written = 0;
		if (flash_write(
				address, (uint32_t*) command->data,
				command->size/sizeof(uint32_t), &written
		)) {
			printf("Failed to write data to flash on address 0x%08lX\r\n", address);
		}
	}
	else if (command->operation == 1) {
		printf("End of file\r\n");
	}
//	else if (command->operation == 2) {
//
//	}
//	else if (command->operation == 3) {
//
//	}
	else if (command->operation == 4) {
		memcpy(&address_base, command->data, sizeof(address_base));
		printf("Set base address to 0x%04X\r\n", address_base);
		const uint32_t address = (address_base << 16);

		AddressToFlash_t flash = { 0 };
		if (flash_address2sector(address, &flash)) {
			printf("Failed to get flash data for address 0x%08lX\r\n", address);
			return;
		}
		static AddressToFlash_t flash_prev = { 0 };
		if (flash_is_same_sector(&flash, &flash_prev)) {
			printf("Sector with address 0x%08lX already cleared\r\n", address);
			return;
		}
		if (flash_clear_sector(address, 0)) {
			printf("Failed to clear sector on address 0x%08lX\r\n", address);
		}
		memcpy(&flash_prev, &flash, sizeof(flash_prev));
	}
	else if (command->operation == 5) {
		printf("Program's start address 0x%08lX\r\n", *((uint32_t*) command->data));
	}
	else {
		printf("Unsupported HEX command %d!!\r\n", command->operation);
	}
}


void handle_hex_buffer(const char* const buffer, uint16_t len) {
	// arrived buffer can contain a part of hex-command.
	// Temporary store till second part arrives
	static char buffer_parts[SIZE_TFTP_DATA] = { '\0' };
	static size_t last_symbol_idx = 0;

	IntelHexData_t storage = { 0 };
	uint8_t data[SIZE_TFTP_DATA] = { '\0' };

	uint8_t start = 1;
	const char* p_symbol = strchr_n(buffer, ':', len);
	while (p_symbol) {
		storage.data = (uint8_t*) data;

		if (start && p_symbol != buffer) {
			// add second part to stored part of command and handle it
			if (buffer > p_symbol) {
				printf("WEIRD FUCKUP!! Must not happen\r\n");
			}

			size_t size = p_symbol - buffer;
			memcpy(buffer_parts + last_symbol_idx, buffer, size * sizeof(*p_symbol));
			last_symbol_idx += size;
			parse_intel_hex(buffer_parts, last_symbol_idx, &storage);

			memset(buffer_parts, '\0', last_symbol_idx * sizeof(buffer_parts[0]));
			last_symbol_idx = 0;
			p_symbol = buffer;
		}
		else {
			parse_intel_hex(p_symbol, len - (p_symbol - buffer), &storage);
		}
		start = 0;

		// find next command
		const char* next = strchr_n(p_symbol + 1, ':', len - (p_symbol + 1 - buffer));
		if (storage.checksum_ok) {
			handle_hex_command(&storage);
		}
		else if (p_symbol == NULL) {
			printf("Checksum NotOK\r\n");
		}
		else {
			size_t size = (next == NULL) ? (len - (p_symbol - buffer)) : (next - p_symbol);
			if (next == NULL) {
				// part of command in the end of received buffer.
				// store and wait for second part
				memcpy(buffer_parts + last_symbol_idx, p_symbol, size * sizeof(*p_symbol));
				last_symbol_idx += size;
			}
			else {
				// buffer with only '\n' - happens a lot.
				if (size != 1 || *p_symbol != '\n') {
					char tmp[512] = { '\0' };
					memcpy(tmp, p_symbol, size * sizeof(*p_symbol));
					printf("Checksum NotOK: 0x%s\r\n", tmp);
				}
			}
		}
		p_symbol = next;
		memset(&storage, 0, sizeof(storage));
		memset(data, 0, sizeof(data));
	}
}


/**
 * Write to file
 * @param handle File handle returned by open()
 * @param pbuf PBUF adjusted such that payload pointer points
 *             to the beginning of write data. In other words,
 *             TFTP headers are stripped off.
 * @returns &gt;= 0: Success; &lt; 0: Error
 */
int tftp_write(void* handle, struct pbuf* p) {
	handle_hex_buffer(p->payload, p->tot_len);
	return 1;
}

#endif
