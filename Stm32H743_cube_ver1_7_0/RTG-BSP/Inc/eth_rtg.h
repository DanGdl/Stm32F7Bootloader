/*
 * eth_rtg.h
 *
 *  Created on: 25 באפר׳ 2021
 *      Author: RTG
 */

#ifndef INC_ETH_RTG_H_
#define INC_ETH_RTG_H_

#include <lwip/udp.h>
#include <ethernetif.h>
#include <semphr.h>

#define RTG_MAX_ETH_BUFF DEFAULT_BUFF_SIZE
#define MAIN_PC_IP 		"192.168.1.101"
#define CIB_CLONE_IP 	"192.168.1.202"

#pragma pack(push,1)
typedef struct
{
	u16_t port_in;
	u16_t port_out;
	struct udp_pcb *upcb;
	uint8_t eth_Rbuf[RTG_MAX_ETH_BUFF];
	uint32_t RXlen;
	uint32_t TXlen;
	uint16_t count;
	uint16_t errors_send;
} UDP_t;
#pragma pack(pop)

extern struct netif gnetif;
/* udp callbak message receive port */
// SAA
#define UDP_UART1_PORT_IN     	50017
#define UDP_UART1_PORT_OUT     	50017
//
#define UDP_UART3_PORT_IN     	50020
#define UDP_UART3_PORT_OUT     	50020
//
#define UDP_UART6_PORT_IN     	50018
#define UDP_UART6_PORT_OUT     	50018
//
#define UDP_UART4_PORT_IN     	50012
#define UDP_UART4_PORT_OUT     	50012
// SERVO
#define UDP_UART8_PORT_IN     	50019
#define UDP_UART8_PORT_OUT     	50019
// IMU
#define UDP_SPI3_PORT_IN     	50000
#define UDP_SPI3_PORT_OUT     	50000
#define UDP_SPI5_PORT_IN     	50015
#define UDP_SPI5_PORT_OUT     	50015

//
#define UDP_DLU_PORT_IN         52001
#define UDP_DLU_PORT_OUT        52001
#define UDP_DISCRETE_PORT_IN    50025
#define UDP_DISCRETE_PORT_OUT  	50025

#define SERVO_UDP_PORT      UDP_UART8_PORT_IN
#define SNA_UDP_PORT        UDP_UART1_PORT_IN
#define GPS_UDP_PORT        UDP_UART4_PORT_IN
#define MGLT_UDP_PORT       UDP_UART6_PORT_IN
#define MAANASH_UDP_PORT 	UDP_UART3_PORT_IN

#define CLONE_UDP_PORT 			55000

extern UDP_t CLONE_udp;
extern UDP_t UART1_udp;
extern UDP_t UART3_udp;
extern UDP_t UART4_udp;
extern UDP_t UART6_udp;
extern UDP_t UART8_udp;
extern UDP_t SPI3_udp;
extern UDP_t DLU_udp;
extern UDP_t DISCRETE_udp;

#if NOT_NEED
//
#define UDP_SPI1_PORT_IN     	50023
#define UDP_SPI1_PORT_OUT     	50023
#define UDP_SPI2_PORT_IN     	50022
#define UDP_SPI2_PORT_OUT     	50022
#define UDP_LPUART1_PORT_IN     50021
#define UDP_LPUART1_PORT_OUT   	50021
#define UDP_UART2_PORT_IN     	50016
#define UDP_UART2_PORT_OUT     	50016
#define UDP_UART5_PORT_IN     	50013
#define UDP_UART5_PORT_OUT     	50013
#define UDP_UART7_PORT_IN     	50014
#define UDP_UART7_PORT_OUT     	50014
#define UDP_I2C_PORT_IN     	50024
#define UDP_I2C_PORT_OUT     	50024
#define UDP_SDRAM_PORT_IN       50026
#define UDP_SDRAM_PORT_OUT   	50026
#define UDP_FLASH_PORT_IN       50027
#define UDP_FLASH_PORT_OUT      50027
#define UDP_ADC_PORT_IN         50028
#define UDP_ADC_PORT_OUT        50028

extern UDP_t LPUART1_udp;
extern UDP_t UART2_udp;
extern UDP_t UART5_udp;
extern UDP_t UART7_udp;
extern UDP_t SPI1_udp;
extern UDP_t SPI2_udp;
extern UDP_t SPI5_udp;
extern UDP_t I2C_udp;
extern UDP_t SDRAM_udp;
extern UDP_t FLASH_udp;
extern UDP_t ADC_udp;
extern UDP_t CBIT_udp;
extern UDP_t PBIT_udp;
#endif

void RTG_UDP_INIT(void);
void RTG_UDP_COMMON_INIT(UDP_t *dev_udp);
void RTG_Udp_Receive_Callback(void *arg, struct udp_pcb *upcb, struct pbuf *p,
		ip_addr_t *addr, u16_t port);

void RTG_Task_UDP_UART(UDP_t *UART_udp, uint8_t to);

void RTG_Udp_Send(char *mess, uint16_t length);
void RTG_Udp_SendTo(UDP_t dev_udp, char *mess, uint16_t length);
void RTG_Udp_SendToClone( char *mess, uint16_t length) ;

#endif /* INC_ETH_RTG_H_ */
