#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <ctype.h>
#include <ip_addr.h>
#include <pbuf.h>
#include <udp.h>

#include "lwip.h"
#include "usart.h"
#include "rtg_main.h"


#define PORT_MINE			1025
#define PORT_SERVER			69 // TFTP // 21 FTP
#define LEN_ARRAY(x)		(sizeof(x)/sizeof(x[0]))
#define LEN_STR(x)			(sizeof(x)/sizeof(x[0]) - 1)
#define SIZE_TFTP_PACKET	516

// TFTP op-codes
#define OP_RRQ		1
#define OP_DATA		3
#define OP_ACK		4
#define	OP_ERROR	5
#define MODE		"octet" // ??

// 192.168.001.010
extern struct netif gnetif;
extern UART_HandleTypeDef huart3;

uint8_t transfer_done = 0;
uint16_t idx_block = 0;
uint32_t idx_packet = 0;

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
		HAL_StatusTypeDef status = HAL_UART_Receive(&huart3, &c, 1, HAL_MAX_DELAY);
		if (c == '\r') {
			HAL_UART_Transmit(&huart3, &c, 1, HAL_MAX_DELAY);
			c = '\n';
			HAL_UART_Transmit(&huart3, &c, 1, HAL_MAX_DELAY);
			break;
		}
		else {
			HAL_UART_Transmit(&huart3, &c, 1, HAL_MAX_DELAY);
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
			const int len = i - 1 - idx_first_digit;
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
	printf("Custom %lu, System %lu\r\n", n, server_ip.addr);

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
	idx_packet = 0;
	transfer_done = 0;

	char* tmp = udp_buffer->payload;
	*(int16_t*) tmp = htons(OP_RRQ);
	tmp += 2;
	strcpy(tmp, file_name);
	tmp += len + 1;
	strcpy(tmp, MODE);
	tmp += LEN_ARRAY(MODE);

	if (udp_send(upcb, udp_buffer) != ERR_OK) {
		printf("Request transfer start failed\n\r");
	}
	udp_disconnect(upcb);
	pbuf_free(udp_buffer);
	return 0;
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
			tfpt_request_file(ip, buffer);
			request_file = 0;
		}

		if (transfer_done) {
			printf("File arrived. Last Block idx %u, counter %lu\r\n", idx_block, idx_packet);
		}
	}
}



void udp_receive_callback(
		void* arg, struct udp_pcb* upcb, struct pbuf* p,
		const ip_addr_t* addr, u16_t port
) {
	// arrived buffer 516 bytes (data (512) + header)
	char* tmp = p->payload;
	if (ntohs(*((int16_t*) p->payload)) == OP_ERROR) {
		printf("Error received: %s\n\r", tmp + 4);
		transfer_done = 1;
	}
	else {
		if (p->tot_len < SIZE_TFTP_PACKET) {
			transfer_done = 1;
		}
		uint16_t block = ntohs(*((int16_t*) (p->payload + 2)));
		if (block == idx_block) {
			printf("Same block arrived %d!\r\n", block);
			// End of transfer
			transfer_done = 1;
		}
		idx_block = block;
		printf("Received packet idx %u, counter %lu\r\n", idx_block, idx_packet);
		idx_packet++;
		// write(1, buffer + 4, count - 4);

		// Send an ACK packet. The block number we want to ACK is
		// already in the buffer so we just need to change the
		// opcode. Note that the ACK is sent to the port number
		// which the server just sent the data from, NOT to port 69
		struct pbuf* txBuf = pbuf_alloc(PBUF_TRANSPORT, 4, PBUF_RAM);
		*(int16_t*) tmp = htons(OP_ACK);
		pbuf_take(txBuf, tmp, 4); // copy the data into the buffer

		err_t is_connected = udp_connect(upcb, addr, port);
		if (is_connected == ERR_OK) {
			if (udp_send(upcb, txBuf) != ERR_OK) {
				printf("Send ACK failed\n\r");
			}
			udp_disconnect(upcb);
			pbuf_free(txBuf);
		} else {
			printf("Failed to connect to TFTP server\r\n");
		}
	}
	// Free the p buffer
	pbuf_free(p);
}
