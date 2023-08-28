#ifndef UTIL_H_
#define UTIL_H_


#define LEN_ARRAY(x)		(sizeof(x)/sizeof(x[0]))
#define LEN_STR(x)			(sizeof(x)/sizeof(x[0]) - 1)
#define MAX(x, y)			(x > y ? x : y)
#define MIN(x, y)			(x < y ? x : y)

const char* strchr_n(const char* const str, char symbol, unsigned int len);

#endif
