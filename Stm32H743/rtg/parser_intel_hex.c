
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "parser_intel_hex.h"
#include "util.h"

#define TOKEN_SIZE		1
#define TOKEN_ADDRESS 	3
#define TOKEN_COMMAND	7
#define TOKEN_DATA		9


void parse_intel_hex(
	const char* const buffer, uint32_t len, IntelHexData_t* const storage
) {
	if (buffer == NULL || len == 0 || storage == NULL) {
		return;
	}
	char buffer_tmp[28] = { '\0' };
	char* end = NULL;
	uint32_t sum_data = 0;
	uint16_t idx_data_end = 0;
	uint16_t counter = 0;
	for (int idx = 1; ; idx += 2) {
		if (idx >= len) {
			break;
		}
		memcpy(buffer_tmp, buffer + idx, 2);
		buffer_tmp[2] = '\0';

		int8_t value = strtol(buffer_tmp, &end, 16);
		sum_data += value;

		if (idx == TOKEN_SIZE) {
			storage->size = value;
			idx_data_end = TOKEN_DATA + value * 2;
			if ((idx_data_end + 2) >= len) {
				// printf("Buffer is too short, wait for second part!\r\n");
				break;
			}
		}
		else if (idx == TOKEN_ADDRESS) {
			memcpy(buffer_tmp, buffer + idx, 4);
			buffer_tmp[4] = '\0';
			storage->address = strtol(buffer_tmp, &end, 16);
		}
		else if (idx == TOKEN_COMMAND) {
			storage->operation = value;
		}
		else if (idx >= TOKEN_DATA && idx < idx_data_end) {
#ifdef STORE_RAW_DATA
			if (idx == TOKEN_DATA) {
				memcpy(storage->data_raw, buffer + TOKEN_DATA, idx_data_end - TOKEN_DATA);
			}
#endif
			if (storage->operation == 4 || storage->operation == 5) {
				if (idx != TOKEN_DATA) {
					continue;
				}
				uint8_t size = idx_data_end - idx;
				memcpy(buffer_tmp, buffer + idx, size);
				buffer_tmp[size] = '\0';
				uint32_t addr = strtol(buffer_tmp, &end, 16);
				memcpy(storage->data, &addr, storage->size);
				counter += storage->size;
			}
			else {
				storage->data[counter] = value;
				counter++;
			}
		}
		else if (idx == idx_data_end) {
			sum_data -= value;
			uint8_t checksum = value;
			if (checksum) {
				storage->checksum_ok = (sum_data % 256) == (256 - checksum);
			} else {
				storage->checksum_ok = (sum_data % 256) == checksum;
			}
			break;
		}
	}
}
