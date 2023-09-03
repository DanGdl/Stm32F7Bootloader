/*
 * uarts.c
 *
 *  Created on: Mar 20, 2021
 *      Author: itzhaki
 */

#include <ADIS16547.h>
#include <bit.h>
#include <GPS.h>
#include <IMU-TRF-90M.h>
#include <RTG_main.h>
#include <string.h>
#include <Safe_And_Arm.h>
#include <SERVO.h>
#include <uarts.h>
#include <rtg_tasks.h>

extern uint8_t GPS_Buff [60];

/* udp callbak message receive port to task */
extern uint16_t UartCallbackNumber;

extern SERVO_UART_rx_t SERVO_UART_buff;
/* flag get message UART receive SERVO  */
extern  uint8_t uartServoGetMessage;

Uarts sUart [UART_END];

//Allocating Tx\Rx Buffers for all UARTs
// for LPUART1 Buffers another
uint8_t UartTxBuf [UART_END] [UART_BUF_SIZE];
uint8_t UartRxBuf [UART_END] [UART_BUF_SIZE];

uint8_t LPUART1TxBuf [UART_BUF_SIZE] __attribute__((section(".TxLPUART"))); /* LPUART1 TX Buffers */
uint8_t LPUART1RxBuf [UART_BUF_SIZE] __attribute__((section(".RxLPUART"))); /* LPUART1 RX Buffers */
//

void RTG_UartInit(void)
{
	sUart [UART_SAA].handle = UART_HANDLE_SAA;
	sUart [UART_SAA].UART_udp = &UART1_udp;

	sUart [UART_MAANASH].handle = UART_HANDLE_MAANASH;
	sUart [UART_MAANASH].UART_udp = &UART3_udp;

	sUart [UART_GPS].handle = UART_HANDLE_GPS;
	sUart [UART_GPS].UART_udp = &UART4_udp;

	sUart [UART_CIB2SEEKER].handle = UART_HANDLE_CIB2SEEKER;
	sUart [UART_CIB2SEEKER].UART_udp = &UART4_udp;
	sUart [UART_CIB2SEEKER].RXbuf = LPUART1RxBuf;
	sUart [UART_CIB2SEEKER].TXbuf = LPUART1TxBuf;
	sUart [UART_CIB2SEEKER].buff_size = sizeof(LPUART1RxBuf);

	sUart [UART_MARGALIT].handle = UART_HANDLE_MARGALIT;
	sUart [UART_MARGALIT].UART_udp = &UART6_udp;

	sUart [UART_SERVO].handle = UART_HANDLE_SERVO;
	sUart [UART_SERVO].UART_udp = &UART8_udp;

	for (uint32_t i = 0; i < UART_END; i++)
	{
		sUart [i].handle->ErrorCode = HAL_UART_ERROR_NONE;
		sUart [i].uartRx_ready = 0;
		sUart [i].uartTx_ready = 0;
		if( i != UART_CIB2SEEKER)
		{
			sUart [i].RXbuf = UartRxBuf [i];
			sUart [i].TXbuf = UartTxBuf [i];
			sUart [i].buff_size = sizeof(UartTxBuf [i]);
		}
		sUart [i].Timeout = 200;
		RTG_Uart_ReciveMessegeIdle(&sUart [i]);
	}

}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	static portBASE_TYPE xHigherPriorityTaskWoken;
	uint32_t xNVIC;
	static Uart_MessageToQueue_t Message;

	Message.uart_n = 255;
	for (uint32_t i = 0; i < UART_END; i++){
		if( huart == sUart [i].handle ){
				Message.uart_n = i;
				break;
		}
	}
	if( Message.uart_n == 255 ) return;
	xNVIC = taskENTER_CRITICAL_FROM_ISR();
	RTG_Uart_ReciveMessegeIdle(&sUart [Message.uart_n]);
	taskEXIT_CRITICAL_FROM_ISR(xNVIC);
	sUart [Message.uart_n].dataRx_length = Size;
	Message.size = Size;
	if ( xQueueSendFromISR( xUartQueue,( void * ) &Message, &xHigherPriorityTaskWoken ) != pdPASS) {/* Failed to post the message, even after 10 ticks. */
	}

}


HAL_StatusTypeDef RTG_Uart_ReciveMessegeIdle(Uarts *uartStruct)
{
	HAL_StatusTypeDef stat;
	uartStruct->uartRx_ready = 0;
	if (uartStruct->handle->hdmarx)
	{
		stat = HAL_UARTEx_ReceiveToIdle_DMA(uartStruct->handle, uartStruct->RXbuf, uartStruct->buff_size);
		__HAL_DMA_DISABLE_IT(uartStruct->handle->hdmarx, DMA_IT_HT);
	}
	else
	{
		stat = HAL_UARTEx_ReceiveToIdle_IT(uartStruct->handle, uartStruct->RXbuf, uartStruct->buff_size);
	}
	if (stat != HAL_OK){
		uartStruct->handle->Lock = HAL_UNLOCKED;
			uartStruct->ErrorRecieve++;
	}
	return stat;
}

/*
 #define  HAL_UART_ERROR_NONE             (0x00000000U)    /!< No error                * /
 #define  HAL_UART_ERROR_PE               (0x00000001U)    /!< Parity error            * /
 #define  HAL_UART_ERROR_NE               (0x00000002U)    /!< Noise error             * /
 #define  HAL_UART_ERROR_FE               (0x00000004U)    /!< Frame error             * /
 #define  HAL_UART_ERROR_ORE              (0x00000008U)    /!< Overrun error           * /
 #define  HAL_UART_ERROR_DMA              (0x00000010U)    /!< DMA transfer error      * /
 #define  HAL_UART_ERROR_RTO              (0x00000020U)    /!< Receiver Timeout error  * /
 */

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	int i;
	UBaseType_t uxSavedInterruptStatus;
	for (i = 0; i < UART_END; i++)
	{
		if (sUart [i].handle == huart)
		{
			break;
		}
	}
	uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();
	sUart [i].ErrorCallback++;
	__HAL_UART_CLEAR_PEFLAG(huart);
	__HAL_UART_CLEAR_NEFLAG(huart);
	__HAL_UART_CLEAR_FEFLAG(huart);
	__HAL_UART_CLEAR_OREFLAG(huart);
	RTG_Uart_ReciveMessegeIdle(&sUart [i]);
	taskEXIT_CRITICAL_FROM_ISR( uxSavedInterruptStatus );
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	int i;
	for (i = 0; i < UART_END; i++)
	{
		if (sUart [i].handle == huart)
		{
			sUart [i].ErrorRecieveCmpl++;
			RTG_Uart_ReciveMessegeIdle(&sUart [i]);
		}
	}
}

HAL_StatusTypeDef RTG_Uart_SendMessege(Uarts *uartStruct)
{
	HAL_StatusTypeDef stat;
	uartStruct->handle->gState = HAL_UART_STATE_READY;
	if( uartStruct->dataTx_length >= RTG_MAX_ETH_BUFF-10 ) return HAL_ERROR;
	if (uartStruct->handle->hdmatx)
	{
		stat = HAL_UART_Transmit_DMA(uartStruct->handle, uartStruct->TXbuf, uartStruct->dataTx_length);
	}
	else
	{
		stat = HAL_UART_Transmit_IT(uartStruct->handle, uartStruct->TXbuf, uartStruct->dataTx_length);
	}
	if(stat != HAL_OK)
	{
		uartStruct->ErrorRxLength++;
	}
	return stat;
}

HAL_StatusTypeDef RTG_Uart_SetBaudRate(Uarts *uartStruct, uint32_t BaudRate)
{
// Add check that the uart is not busy
	HAL_UART_AbortReceive(uartStruct->handle);
	HAL_UART_AbortTransmit(uartStruct->handle);

	uartStruct->handle->Init.BaudRate = BaudRate;
	return HAL_UART_Init(uartStruct->handle);
}

uint32_t RTG_Uart_GetBaudRate(Uarts *uartStruct)
{
	return uartStruct->handle->Init.BaudRate;
}

// Don't used
HAL_StatusTypeDef RTG_Uart_ReciveMessege(Uarts *uartStruct)
{
	static uint32_t tickstart;
	HAL_StatusTypeDef error;

	/* Init tickstart for timeout management */
	tickstart = HAL_GetTick();

	uartStruct->uartRx_ready = 0;
	if (uartStruct->handle->hdmarx)
	{
		error = HAL_UART_Receive_DMA(uartStruct->handle, uartStruct->RXbuf, uartStruct->buff_size);
	}
	else
	{
		error = HAL_UART_Receive_IT(uartStruct->handle, uartStruct->RXbuf, uartStruct->buff_size);
	}
	if (error != HAL_OK)
	{
		return error;
	}

	while (!uartStruct->uartRx_ready)
	{
		if (((HAL_GetTick() - tickstart) > (uartStruct->Timeout * 2)) || (uartStruct->Timeout == 0U))
		{
			if (HAL_UART_Abort(uartStruct->handle) == HAL_OK)
			{
				uartStruct->uartRx_ready = 1;
				return HAL_TIMEOUT;
			}
		}
	}

	return HAL_OK;
}

