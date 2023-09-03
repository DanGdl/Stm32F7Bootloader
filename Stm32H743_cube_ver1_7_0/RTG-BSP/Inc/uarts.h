/*
 * uarts.h
 *
 *  Created on: Mar 20, 2021
 *      Author: itzhaki
 */

#ifndef INC_UARTS_H_
#define INC_UARTS_H_

#include <main.h>
#include <eth_rtg.h>

//#include "uarts.h"

#define UART_BUF_SIZE DEFAULT_BUFF_SIZE
/* Dimensions of the buffer that the task being created will use as its stack.
 NOTE:  This is the number of words the stack will hold, not the number of
 bytes.  For example, if each stack item is 32-bits, and this is set to 100,
 then 400 bytes (100 * 32-bits) will be allocated. */
#define UART_STACK_SIZE 1200

// Handles UART  defined in main.c by CubeMX
extern UART_HandleTypeDef hlpuart1;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart6;
//extern UART_HandleTypeDef huart7;
extern UART_HandleTypeDef huart8;
// Mutex handles

typedef struct uartStruct
{
	UART_HandleTypeDef *handle;
	UDP_t *UART_udp;
	char uartTx_ready;
	char uartRx_ready;
	unsigned char *TXbuf;
	unsigned char *RXbuf;
	uint16_t buff_size;
	uint32_t Timeout;
	uint16_t dataRx_length;
	uint16_t dataTx_length;
	uint32_t ErrorRxLength, ErrorRecieve, ErrorRecieveCmpl, ErrorCallback,recieveCount;
} Uarts;

#define UART_HANDLE_SAA 			&huart1
#define UART_HANDLE_MAANASH			&huart3
#define UART_HANDLE_GPS	 			&huart5
#define UART_HANDLE_SERVO 			&huart8
#define UART_HANDLE_MARGALIT 		&huart6
#define UART_HANDLE_CIB2SEEKER 		&hlpuart1


enum
{
	UART_SAA = 0,
	UART_MAANASH,
	UART_GPS,
	UART_MARGALIT,
	UART_SERVO,
	UART_CIB2SEEKER,
	UART_END
};

typedef struct {
	uint8_t sourceId:4;
	uint8_t destinationId:4;
	uint8_t msgLenDwords;
	uint8_t icdVersion:4;
	uint8_t cyclicCounter:4;
	uint8_t messageType;
	uint32_t timeTag;
}WCS_CIB_header_t;
extern Uarts sUart [UART_END];

void RTG_UartInit(void);   //Init UART SubSystem - UartsStartPramInt();
HAL_StatusTypeDef RTG_Uart_SetBaudRate(Uarts *uartStruct, uint32_t BaudRate);
uint32_t RTG_Uart_GetBaudRate(Uarts *uartStruct);

HAL_StatusTypeDef RTG_Uart_SendMessege(Uarts *uartStruct);

void RTG_Task_UART_IDLE(void *argument);

void RTG_uartRxIT(Uarts *uartStruct);
HAL_StatusTypeDef RTG_Uart_ReciveMessegeIdle(Uarts *uartStruct);
HAL_StatusTypeDef RTG_Uart_ReciveMessege(Uarts *uartStruct);

#endif /* INC_UARTS_H_ */
