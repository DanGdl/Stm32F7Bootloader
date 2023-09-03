/*
 * rtg_tasks.h
 *
 *  Created on: 10 בדצמ׳ 2022
 *      Author: e_tas
 */
/*
 * ISR DataReady HAL_GPIO_EXTI_Callback GPIOB14 2.5 ms
 * open semaphor onDataReady
 * task onDataReady wait semaphor onDataReady
 *
 * ISR HAL_SPI_TxRxCpltCallback (spi.c)
 * open semaphor onSpiTxRx
 * task onSpiTxRx wait semaphor onSpiTxRx
 * 		copy IMU data, send udp message
 * 		open semaphor onCbit
 *
 * 	RTG_Udp_Receive_Callback
 * 	open semaphor onUdpCallback (eth_rtg.c)
 * 	task onUdpCallback wait semaphor onUdpCallback
 *
 * 	HAL_UARTEx_RxEventCallback
 *  open semaphore onUARTxRxCallback
 *
 */

#ifndef INC_RTG_TASKS_H_
#define INC_RTG_TASKS_H_

#include <stdint.h>
#include <ethernetif.h>
#include <semphr.h>


#define RTG_IS_IRQ()                  (__get_IPSR() != 0U)

#define TASK_STACK_SIZE 1600

#define TASK_PRIORITY_onDataReady		     osPriorityNormal5
#define TASK_PRIORITY_onSpiTxRx			     osPriorityNormal5
#define TASK_PRIORITY_runCbit 			     osPriorityNormal-1
#define TASK_PRIORITY_onUdpCallback		     osPriorityNormal-1
#define TASK_PRIORITY_onUARTxRxCallback	     osPriorityNormal
#define TASK_PRIORITY_onEthSend			     osPriorityNormal3

#define TASK_PRIORITY(NAME) 		TASK_PRIORITY_##NAME
#define T_FUN_HEADER(NAME) 			void TASK_NAME(NAME)(void *argument )
#define TASK_NAME(NAME) 			CIB_TASK_##NAME
#define SEMAPHORE_HDL(NAME) 		xSemaphore_##NAME
#define TASK_HANDLE(NAME) 			xTaskHandle_##NAME


#define INIT_ALL_TASKS						\
		TASK_CREATE(onDataReady);			\
		TASK_CREATE(onSpiTxRx); 			\
		TASK_CREATE(runCbit);				\
		TASK_CREATE(onUdpCallback);			\
		TASK_CREATE(onUARTxRxCallback);		\
		TASK_CREATE(onEthSend);		\
/* INIT_ALL_TASKS */

#define INIT_ALL_TASKS_VAR					\
		TASK_INIT_VARS(onDataReady) 		\
		TASK_INIT_VARS(onSpiTxRx) 			\
		TASK_INIT_VARS(runCbit) 			\
		TASK_INIT_VARS(onUdpCallback)		\
		TASK_INIT_VARS(onUARTxRxCallback)	\
		TASK_INIT_VARS(onEthSend)	\
/* INIT_ALL_TASKS_VAR */

#define INIT_ALL_SEMAPHORE_VARS 			\
		SEMAPHORE_VAR(onDataReady) 			\
		SEMAPHORE_VAR(onSpiTxRx) 			\
		SEMAPHORE_VAR(runCbit) 				\
		SEMAPHORE_VAR(onUdpCallback) 		\
		SEMAPHORE_VAR(onUARTxRxCallback)	\
// INIT_ALL_SEMAPHORE_VARS

#define INIT_ALL_SEMAPHORES 				\
		SEMAPHORE_CREATE(onDataReady) 		\
		SEMAPHORE_CREATE(onSpiTxRx) 		\
		SEMAPHORE_CREATE(runCbit) 			\
		SEMAPHORE_CREATE(onUdpCallback)		\
		SEMAPHORE_CREATE(onUARTxRxCallback)	\
// INIT_ALL_SEMAPHORES


#define SEMAPHORE_TAKE(NAME)  	( xSemaphoreTake ( xSemaphore_##NAME ,portMAX_DELAY ) == pdTRUE )
#define SEMAPHORE_GIVE(NAME)   	xSemaphoreGive( xSemaphore_##NAME  )

#define YIELD_SEMAPHORE_GIVE_ISR(NAME,xHigherPriorityTaskWoken )   \
	do{ xSemaphoreGiveFromISR( xSemaphore_##NAME, &xHigherPriorityTaskWoken ); \
	portYIELD_FROM_ISR( xHigherPriorityTaskWoken ); }while(0)
#define SEMAPHORE_GIVE_ISR(NAME,xHigherPriorityTaskWoken )   \
	do{ xSemaphoreGiveFromISR( xSemaphore_##NAME, &xHigherPriorityTaskWoken ); }while(0)

#define TASK_NAME_ST_(NAME) #NAME
#define TASK_NAME_ST(NAME) TASK_NAME_ST_(TASK_NAME(NAME))

#define TASK_INIT_VARS(NAME) 						\
__EXT	StaticTask_t 	xTaskBuffer_##NAME; 			\
__EXT	StackType_t 	xStack_##NAME [TASK_STACK_SIZE]; \
__EXT	TaskHandle_t 	xTaskHandle_##NAME; 			\
__EXT uint8_t 		task ## NAME ## flag ;			\
//RTG_TASK_NAME


#define TASK_CREATE(NAME) \
	do{ \
	T_FUN_HEADER(NAME) ; \
	xTaskHandle_##NAME = xTaskCreateStatic(	\
			TASK_NAME(NAME), \
			TASK_NAME_ST(NAME) , \
			sizeof(xStack_##NAME) / sizeof(StackType_t), \
			NULL, \
			TASK_PRIORITY(NAME),\
			xStack_##NAME, \
			&xTaskBuffer_##NAME); \
	configASSERT(xTaskHandle_##NAME != NULL); \
	}while(0)


/*==== Semaphores ======*/
/*INIT VAR*/
#define SEMAPHORE_VAR(NAME)	\
__EXT		SemaphoreHandle_t xSemaphore_##NAME;	\
__EXT		StaticSemaphore_t xSemaphoreBuffer_##NAME;

#define SEMAPHORE_CREATE(NAME) \
		xSemaphore_##NAME = xSemaphoreCreateBinaryStatic(&xSemaphoreBuffer_##NAME ); 	\
		configASSERT( xSemaphore_##NAME != NULL );

// extern values
#define __EXT extern // for extern values
INIT_ALL_TASKS_VAR
INIT_ALL_SEMAPHORE_VARS
#undef __EXT
#define __EXT

//************* QUIUE ***********************************************************
#define 	ETH_QUEUE_LENGTH    		20
#define 	UART_QUEUE_LENGTH    		20
#define 	EthSend_QUEUE_LENGTH    	20

typedef struct {
	uint16_t length;
	uint16_t port;
	ip_addr_t addres;
	uint8_t *buffer;
}EHT_MessageToQueue_t;

typedef struct {
	struct udp_pcb *upcb;
	uint16_t port;
	u32_t addres;
	struct pbuf *udp_buffer;
}EHT_SendMsgToQueue_t;

typedef struct {
	uint8_t uart_n;
	uint8_t *buffer;
	uint8_t size;
}Uart_MessageToQueue_t;


extern StaticQueue_t xEthStaticQueue;
extern uint8_t ucEthQueueStorageArea[ ETH_QUEUE_LENGTH * sizeof(EHT_MessageToQueue_t) ];
extern QueueHandle_t xEthQueue;
extern uint8_t EthMessageBuffer[ETH_QUEUE_LENGTH][DEFAULT_BUFF_SIZE];

extern StaticQueue_t xUartStaticQueue;
extern uint8_t ucUartQueueStorageArea[ UART_QUEUE_LENGTH * sizeof(Uart_MessageToQueue_t) ];
extern QueueHandle_t xUartQueue;

extern StaticQueue_t xEthSendStaticQueue;
extern uint8_t ucEthSendQueueStorageArea[ EthSend_QUEUE_LENGTH * sizeof(EHT_SendMsgToQueue_t) ];
extern QueueHandle_t xEthSendQueue;



#endif /* INC_RTG_TASKS_H_ */

 /*
 #define UP_PARIORIRY_TASK(DEV) \
 		vTaskPrioritySet( TASK_HANDLE(DEV), RTG_TASK_PRIORITY(DEV) +3 );

 #define DOWN_PARIORIRY_TASK(DEV) \
 		vTaskPrioritySet( TASK_HANDLE(DEV), RTG_TASK_PRIORITY(DEV));
*/
