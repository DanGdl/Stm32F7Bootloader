#include <stdlib.h>
#include "util.h"

const char* strchr_n(const char* const str, char symbol, unsigned int len) {
	for (unsigned int i = 0; i < len; i++) {
		if (*(str + i) == symbol) {
			return str + i;
		}
	}
	return NULL;
}
