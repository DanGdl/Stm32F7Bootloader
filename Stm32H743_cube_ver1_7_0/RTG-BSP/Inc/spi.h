/*
 * spis.h
 *
 *  Created on: 25 Mar 2021
 *      Author: itzhaki
 */

#ifndef INC_SPIS_H_
#define INC_SPIS_H_

#include <main.h>
#include <CIB_Protocol.h>

#include "FPGA_Version.h"


#define SPI_BUF_SIZE DEFAULT_BUFF_SIZE

//FPGA<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#define FPGA_SET_WRITE_BIT(u32_val)  ( ( ~( (uint32_t)1<<31 ) ) & (u32_val) )
#define FPGA_SET_READ_BIT(u32_val)  ( ( (uint32_t)1<<31 ) | (u32_val) )
//FPGA<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;
extern SPI_HandleTypeDef hspi3;
extern SPI_HandleTypeDef hspi5;

typedef struct handlSPI
{
	SPI_HandleTypeDef *handle;
	GPIO_TypeDef *cs_port;
	uint16_t cs_gpio;
	uint8_t *TxBuff;
	uint8_t *RxBuff;
	char Tx_ready;
	char Rx_ready;
	uint16_t size;
	uint32_t Timeout;
//	void* private;
} HSPI;

extern HSPI sSpi1;
extern HSPI sSpi2;
extern HSPI sSpi3;
extern HSPI sSpi5;

#define SPI_1 (&sSpi1)
#define SPI_2 (&sSpi2)
#define SPI_3 (&sSpi3)
#define SPI_5 (&sSpi5)

HAL_StatusTypeDef RTG_SPI_TransmitReceive(HSPI *spi);
HAL_StatusTypeDef RTG_SPI_TransmitReceive_DMA(HSPI *spi, uint16_t* TxBuff, uint8_t* RxBuff, uint32_t size);
HAL_StatusTypeDef RTG_SPI_Transmit(HSPI *spi, uint8_t *TBuf, int size);
HAL_StatusTypeDef RTG_SPI_Receive(HSPI *spi, int size);
HAL_StatusTypeDef RTG_SPI_TransmitReceiveSpi2();
//FPGA<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
uint32_t FpgaRead(uint32_t addr);
uint32_t FpgaWrite(uint32_t addr,uint32_t data);
CIB_FPGA_Ver FpgaGetVersion();
uint32_t FpgaGetDate();
uint32_t FpgaGetSysSync();
uint32_t FpgaSetSysSync(uint32_t data);
//FPGA<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

void RTG_SPI_Error(HSPI *sSpi);

#endif /* INC_SPIS_H_ */
