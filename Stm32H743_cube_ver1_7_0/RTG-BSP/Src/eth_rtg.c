/*
 * eth_rtg.c
 *
 *  Created on: 25 באפר׳ 2021
 *      Author: RTG
 */


#include <bit.h>
#include <adc.h>
#include <ADIS16547.h>
#include <discrete.h>
#include <dlu.h>
#include <eth_rtg.h>
#include <RTG_main.h>
#include <string.h>
#include <sys/_stdint.h>
#include <rtg_tasks.h>
#include <uarts.h>

extern uint16_t udpCallbackRxPort; /* udp callbak message receive port to task */
extern uint8_t udpServoGetMessage; /* flag get message udp from SERVO port */
extern ETH_HandleTypeDef heth; /* from ethernetif.c for check ethrnet status */

u32_t MU_addr; /* for save address send udp message  */

UDP_t UART1_udp =    { .port_in = UDP_UART1_PORT_IN,    .port_out = UDP_UART1_PORT_OUT };
UDP_t UART3_udp =    { .port_in = UDP_UART3_PORT_IN,    .port_out = UDP_UART3_PORT_OUT };
UDP_t UART4_udp =    { .port_in = UDP_UART4_PORT_IN,    .port_out = UDP_UART4_PORT_OUT };
UDP_t UART6_udp =    { .port_in = UDP_UART6_PORT_IN,    .port_out = UDP_UART6_PORT_OUT };
UDP_t UART8_udp =    { .port_in = UDP_UART8_PORT_IN,    .port_out = UDP_UART8_PORT_OUT };
UDP_t SPI3_udp =     { .port_in = UDP_SPI3_PORT_IN,     .port_out = UDP_SPI3_PORT_OUT };
UDP_t DISCRETE_udp = { .port_in = UDP_DISCRETE_PORT_IN, .port_out = UDP_DISCRETE_PORT_OUT };
UDP_t CLONE_udp =    { .port_in = CLONE_UDP_PORT,       .port_out = CLONE_UDP_PORT };

#if NOT_NEED
UDP_t DLU_udp =      { .port_in = UDP_DLU_PORT_IN,      .port_out = UDP_DLU_PORT_OUT };
UDP_t SPI1_udp   = { .port_in = UDP_SPI1_PORT_IN,  .port_out = UDP_SPI1_PORT_OUT };
UDP_t SPI2_udp   = { .port_in = UDP_SPI2_PORT_IN,  .port_out = UDP_SPI2_PORT_OUT };
UDP_t SPI5_udp   = { .port_in = UDP_SPI5_PORT_IN,  .port_out = UDP_SPI5_PORT_OUT };
UDP_t SDRAM_udp  = { .port_in = UDP_SDRAM_PORT_IN, .port_out = UDP_SDRAM_PORT_OUT };
UDP_t FLASH_udp  = { .port_in = UDP_FLASH_PORT_IN, .port_out = UDP_FLASH_PORT_OUT };
UDP_t ADC_udp    = { .port_in = UDP_ADC_PORT_IN,   .port_out = UDP_ADC_PORT_OUT  };
#endif

void RTG_UDP_INIT(void) {
	MU_addr = ipaddr_addr(MAIN_PC_IP);
	RTG_UDP_COMMON_INIT( &CLONE_udp    );
	RTG_UDP_COMMON_INIT( &UART1_udp    );
	RTG_UDP_COMMON_INIT( &UART3_udp    );
	RTG_UDP_COMMON_INIT( &UART4_udp    );
	RTG_UDP_COMMON_INIT( &UART6_udp    );
	RTG_UDP_COMMON_INIT( &UART8_udp    );
	RTG_UDP_COMMON_INIT( &SPI3_udp     );
	RTG_UDP_COMMON_INIT( &DISCRETE_udp );
#if NOT_NEED
	RTG_UDP_COMMON_INIT(&SPI5_udp);
	RTG_UDP_COMMON_INIT(&SPI1_udp);
	RTG_UDP_COMMON_INIT(&SPI2_udp);
	RTG_UDP_COMMON_INIT(&SDRAM_udp);
	RTG_UDP_COMMON_INIT(&FLASH_udp);
	RTG_UDP_COMMON_INIT(&ADC_udp);
#endif
}

void RTG_UDP_COMMON_INIT(UDP_t *dev_udp) {
	dev_udp->upcb = udp_new();
	udp_bind(dev_udp->upcb, NULL, dev_udp->port_in);
	udp_recv(dev_udp->upcb, (udp_recv_fn) RTG_Udp_Receive_Callback, NULL);
}

/*
 * Get message from ETH & send to Queue ( receive in rtg_tasks.c)
 * */
void RTG_Udp_Receive_Callback(void *arg, struct udp_pcb *upcb, struct pbuf *p, ip_addr_t *addr, u16_t port) {
	static uint8_t buffer_number = 0;
	static EHT_MessageToQueue_t MessageHeader;
	if (p->len > RTG_MAX_ETH_BUFF) {
		pbuf_free(p);
		return;
	}
	if (*(uint16_t*) (p->payload) > (p->len - sizeof(CIB_Header))) {
		pbuf_free(p);
		return;
	}
	memcpy(EthMessageBuffer [buffer_number], p->payload, p->len);
	pbuf_free(p);
	MessageHeader.buffer = EthMessageBuffer [buffer_number];
	MessageHeader.length = p->len;
	MessageHeader.port = upcb->local_port;
	MessageHeader.addres = *addr;
	buffer_number = (buffer_number + 1) % ETH_QUEUE_LENGTH;
	if ( xQueueSend( xEthQueue,( void * ) &MessageHeader,( TickType_t ) 10 ) != pdPASS) {
		/* Failed to post the message, even after 10 ticks. */
	}
}
/***end        CallBacks on EthUdp receive message ****/

//**********************************************************
void RTG_Udp_Send(char *mess, uint16_t length) {
	RTG_Udp_SendTo(DISCRETE_udp, mess, length);
}

void RTG_Udp_SendTo(UDP_t dev_udp, char *mess, uint16_t length) {
	struct pbuf *udp_buffer = NULL;
	udp_buffer = pbuf_alloc(PBUF_TRANSPORT, length, PBUF_RAM);
	EHT_SendMsgToQueue_t MessageHeader={0};
	if (udp_buffer != NULL) {
		memcpy(udp_buffer->payload, mess, length);
		dev_udp.upcb->remote_ip.addr = MU_addr;
		MessageHeader.upcb = dev_udp.upcb;
		MessageHeader.port = dev_udp.port_in;
		MessageHeader.udp_buffer = udp_buffer;
		MessageHeader.addres = MU_addr;
		if( dev_udp.port_in == /*SERVO_UDP_PORT*/UDP_SPI3_PORT_IN )
		{

			if ( xQueueSendToFront( xEthSendQueue,( void * ) &MessageHeader,( TickType_t ) 10 ) != pdPASS) {
				/* Failed to post the message, even after 10 ticks. */
			}
		}
		else
		{
			if ( xQueueSend( xEthSendQueue,( void * ) &MessageHeader,( TickType_t ) 10 ) != pdPASS) {
				/* Failed to post the message, even after 10 ticks. */
			}
		}
	}
}

void RTG_Udp_SendToClone( char *mess, uint16_t length) {
	struct pbuf *udp_buffer = NULL;
	udp_buffer = pbuf_alloc(PBUF_TRANSPORT, length, PBUF_RAM);
	EHT_SendMsgToQueue_t MessageHeader={0};
	if (udp_buffer != NULL) {
		memcpy(udp_buffer->payload, mess, length);
		DISCRETE_udp.upcb->remote_ip.addr = 0xca01a8c0;
		MessageHeader.upcb = DISCRETE_udp.upcb;
		MessageHeader.port = DISCRETE_udp.port_in;
		MessageHeader.udp_buffer = udp_buffer;
		MessageHeader.addres = 0xca01a8c0;
		if ( xQueueSend( xEthSendQueue,( void * ) &MessageHeader,( TickType_t ) 10 ) != pdPASS) {
				/* Failed to post the message, even after 10 ticks. */
		}
	}
}
