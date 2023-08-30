#include "settings.h"

#ifdef TFTP_CLIENT

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <ctype.h>
#include <ip_addr.h>
#include <pbuf.h>
#include <udp.h>

#include "util.h"
#include "lwip.h"
#include "tftp.h"
#include "usart.h"
#include "flash.h"
#include "parser_intel_hex.h"
#include "rtg_main.h"


#define PORT_MINE			1025
#define PORT_SERVER			69		// TFTP // 21 FTP

#define MODE				"octet" // binary data

// 192.168.001.010
extern struct netif gnetif;
extern UART_HandleTypeDef huart1;

uint8_t transfer_done = 0;
uint16_t idx_block = 0;

struct udp_pcb* upcb = NULL;
ip_addr_t server_ip = {
	.addr = 0,
};
void udp_receive_callback(void* arg, struct udp_pcb* pcb, struct pbuf* p, const ip_addr_t* addr, u16_t port);



void get_console_message(
		const char* const message,
		char* const buffer, unsigned int len_max
) {
	if (message != NULL) {
		printf(message);
	}
	uint32_t count_symbols = 0;
	while (1) {
		uint8_t c;
		HAL_StatusTypeDef status = HAL_UART_Receive(&huart1, &c, 1, HAL_MAX_DELAY);
		if (c == '\r') {
			HAL_UART_Transmit(&huart1, &c, 1, HAL_MAX_DELAY);
			c = '\n';
			HAL_UART_Transmit(&huart1, &c, 1, HAL_MAX_DELAY);
			buffer[count_symbols] = '\0';
			count_symbols++;
			break;
		}
		else {
			HAL_UART_Transmit(&huart1, &c, 1, HAL_MAX_DELAY);
		}
		if (status == HAL_OK) {
			buffer[count_symbols] = c;
			count_symbols++;
			if (count_symbols == (len_max - 1)) {
				buffer[len_max] = '\0';
				break;
			}
		}
		else {
			printf("Failed to receive data to uart3: %d\r\n", status);
		}
	}
}

uint8_t is_valid_ip(const char* const buffer, unsigned int len_max) {
	uint8_t is_valid_ip = 1;
	uint8_t idx_first_digit = 0;
	uint8_t counter_dots = 0;
	for (int i = 0; i < len_max; i++) {
		if (isdigit((uint8_t) buffer[i])) {
			continue;
		}
		if (buffer[i] == '.' || (buffer[i] == '\0' && counter_dots == 3)) {
			const int len = i - idx_first_digit;
			if (len < 0 || len > 3) {
				is_valid_ip = 0;
				break;
			}
			idx_first_digit = i + 1;
			counter_dots++;
			if (counter_dots == 4) {
				break;
			}
		} else {
			is_valid_ip = 0;
			break;
		}
	}
	return is_valid_ip;
}

uint32_t ip_to_number(const char* const ip, uint8_t* const error_code) {
	// 129.144.50.56 -> 16777216*129 + 65536*144 + 256*50 + 1*56 => 2173710904
	uint32_t result = 0;
	uint8_t idx_first_digit = 0;
	uint8_t counter_dots = 0;
	const int len = strlen(ip);
	for (int i = 0; i <= len; i++) {
		if (i < len && ip[i] != '.') {
			continue;
		}
		char buffer[4] = { '\0' };
		const int dist = i - idx_first_digit;
		if (dist < 0 || dist > 3) {
			*error_code = 1;
			break;
		}
		memcpy(buffer, ip + idx_first_digit, dist);
		uint16_t number = 0;
		const int rc = sscanf(buffer, "%"PRIu16, &number);
		if (rc < 1) {
			printf("Failed to parse token %s as u8 number\n", buffer);
			*error_code = 1;
			break;
		}

		if (counter_dots == 0) {
			result += 16777216 * number;
		}
		else if (counter_dots == 1) {
			result += 65536 * number;
		}
		else if (counter_dots == 2) {
			result += 256 * number;
		}
		else if (counter_dots == 3) {
			result += number;
		}
		else {
			*error_code = 1;
			break;
		}
		counter_dots++;
		idx_first_digit = i + 1;
	}
	return result;
}



int tfpt_request_file(const char* const ip, const char* const file_name) {
	if (upcb != NULL) {
		udp_remove(upcb);
	}
	upcb = udp_new();
	if (upcb == NULL) {
		printf("Failed to create UDP\r\n");
		return -1;
	}

	uint8_t error_code = 0;
	uint32_t n = ip_to_number(ip, &error_code);
	server_ip.addr = ipaddr_addr(ip);
	printf("IP2Num Custom %lu, System %lu\r\n", n, server_ip.addr);

	err_t err = udp_bind(upcb, IP_ADDR_ANY, PORT_MINE);
	if (err == ERR_OK) {
		udp_recv(upcb, udp_receive_callback, NULL);
	} else {
		printf("Failed to bind UDP server server\r\n");
		return -1;
	}

	err_t is_connected = udp_connect(upcb, &server_ip, PORT_SERVER);
	if (is_connected != ERR_OK) {
		printf("Failed to connect to TFTP server\r\n");
		return -1;
	}
	const int len = strlen(file_name);
	struct pbuf* udp_buffer = pbuf_alloc(
			PBUF_TRANSPORT, len + 3 + LEN_ARRAY(MODE), PBUF_RAM
	);
	if (udp_buffer == NULL) {
		printf("Failed to allocate buffer\r\n");
		udp_disconnect(upcb);
		return -1;
	}
	idx_block = 0;
	transfer_done = 0;

	char* tmp = udp_buffer->payload;
	*(int16_t*) tmp = htons(OP_RRQ);
	tmp += 2;
	strcpy(tmp, file_name);
	tmp += len + 1;
	strcpy(tmp, MODE);
	tmp += LEN_ARRAY(MODE);

	int8_t status = 0;
	if (udp_send(upcb, udp_buffer) != ERR_OK) {
		printf("Request transfer start failed\n\r");
		status = -1;
	}
	udp_disconnect(upcb);
	pbuf_free(udp_buffer);
	return status;
}


void rtg_main(void) {
	char buffer[512] = { '\0' };
	char ip[16] = { '\0' };
	uint8_t request_ip = 1;
	uint8_t request_file_name = 0;
	uint8_t request_file = 0;
	while(1) {

		ethernetif_input(&gnetif);
		sys_check_timeouts();

		if (request_ip) {
			get_console_message("Enter TFTP server IP:\r\n", buffer, LEN_ARRAY(buffer));

			if (is_valid_ip(buffer, 15)) {
				request_ip = 0;
				request_file_name = 1;
				request_file = 0;
				memcpy(ip, buffer, LEN_STR(ip));
			} else {
				printf("Provided text %s doesn't looks like IP\r\n", buffer);
			}
			memset(buffer, '\0', LEN_ARRAY(buffer));
		}
		if (request_file_name) {
			get_console_message("Enter a name of file to download (512 symbols max):\r\n", buffer, LEN_ARRAY(buffer));
			if (buffer[0] != '\0') {
				request_ip = 0;
				request_file_name = 0;
				request_file = 1;
			} else {
				printf("There is no file name provided\r\n");
			}
		}
		if (request_file) {
			ethernetif_input(&gnetif);
			if (tfpt_request_file(ip, buffer)) {
				request_ip = 1;
			}
			request_file = 0;
		}

		if (transfer_done) {
			printf("File arrived. Last Block idx %u\r\n", idx_block);
			transfer_done = 0;
			request_ip = 1;
			memset(buffer, 0, LEN_ARRAY(buffer));
		}
	}
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
		printf("Program's start address 0x%08lX. What to do with this?\r\n", *((uint32_t*) command->data));
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

	char tmp[56] = { '\0' };
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

			if (!storage.checksum_ok) {
				memcpy(tmp, buffer_parts + last_symbol_idx, size);
			}
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
					if (tmp[0] != '\0') {
						memcpy(tmp, p_symbol, size * sizeof(*p_symbol));
					}
					printf("Checksum NotOK: 0x%s\r\n", tmp);
				}
			}
		}
		p_symbol = next;
		memset(&storage, 0, sizeof(storage));
		memset(data, 0, sizeof(data));
		memset(tmp, 0, sizeof(tmp));
	}
}

void udp_receive_callback(
	void* arg, struct udp_pcb* upcb, struct pbuf* p,
	const ip_addr_t* addr, u16_t port
) {
	// arrived buffer 516 bytes: data (512) + header (2 bytes OP_CODE, 2 bytes block idx, then data or error message)
	do {
		char* tmp = p->payload;
		if (ntohs(*((int16_t*) p->payload)) == OP_ERROR) {
			printf("Error received: %s\n\r", tmp + 4);
			transfer_done = 1;
			break;
		}
		uint16_t block = ntohs(*((uint16_t*) (p->payload + 2)));
		// printf(((block == idx_block) ? "Same block arrived %d!\r\n" : "Received block idx %u\r\n"), block);
		if (idx_block != block) {
			idx_block = block;
			handle_hex_buffer(tmp + SIZE_TFTP_HEADER, p->tot_len - SIZE_TFTP_HEADER);
		}
		// Send an ACK packet. The block number we want to ACK is
		// already in the buffer so we just need to change the
		// opcode. Note that the ACK is sent to the port number
		// which the server just sent the data from, NOT to port 69
		struct pbuf* txBuf = pbuf_alloc(PBUF_TRANSPORT, 4, PBUF_RAM);
		if (txBuf) {
			*(int16_t*) tmp = htons(OP_ACK);
			pbuf_take(txBuf, tmp, SIZE_TFTP_HEADER); // copy the data into the buffer

			if (udp_connect(upcb, addr, port) == ERR_OK) {
				if (udp_send(upcb, txBuf) != ERR_OK) {
					printf("Send ACK failed\n\r");
				}
				udp_disconnect(upcb);
			} else {
				printf("Failed to connect to TFTP server\r\n");
			}
			pbuf_free(txBuf);
		}
		else {
			printf("Failed to allocate T buffer\r\n");
		}
		if (p->tot_len < SIZE_TFTP_PACKET) {
			transfer_done = 1;
		}
	} while(0);
	// Free the p buffer
	pbuf_free(p);
}

#endif
