#include <inttypes.h>

#ifndef PARSER_INTEL_HEX_H_
#define PARSER_INTEL_HEX_H_

// #define STORE_RAW_DATA

typedef struct IntelHexData {
	uint8_t size;
	uint16_t address;
	uint8_t operation;
	uint8_t checksum_ok;
	uint8_t* data;
} IntelHexData_t;

void parse_intel_hex(
	const char* const buffer, uint32_t len, IntelHexData_t* const storage
);

#endif
