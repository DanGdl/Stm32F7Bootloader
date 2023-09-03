/*
 * spis.c
 *
 *  Created on: 25 Mar 2021
 *      Author: itzhaki
 */
/*
 * SPI Baud Rate
 * 	128	- 	7.8125 KBits/s
 * 	64	-  	1.5625 MBits/s
 * 	32	-	3.125  MBits/s
 * 	16	-	6.25   MBits/s
 *
 */
#include <eth_rtg.h>
#include <rtg_tasks.h>
#include <portmacro.h>
#include <stm32h743xx.h>
#include <stm32h7xx_hal.h>
#include <stm32h7xx_hal_def.h>
#include <stm32h7xx_hal_gpio.h>
#include <stm32h7xx_hal_spi.h>
#include <string.h>
#include <sys/_stdint.h>
#include <ADIS16547.h>
#include <spi.h>

extern char flashFlag;
extern char flasherrFlag;

extern uint8_t flashIDbuf [24];
extern TaskHandle_t xTaskHandle_IMU;

uint8_t spi1TXbuf [SPI_BUF_SIZE], spi1RXbuf [SPI_BUF_SIZE];
uint8_t spi2TXbuf [SPI_BUF_SIZE], spi2RXbuf [SPI_BUF_SIZE];
uint8_t spi3TXbuf [SPI_BUF_SIZE], spi3RXbuf [SPI_BUF_SIZE];
uint8_t spi5TXbuf [SPI_BUF_SIZE], spi5RXbuf [SPI_BUF_SIZE];

//
HSPI sSpi2 = { .handle = &hspi2, .cs_gpio = GPIO_PIN_4, .cs_port = GPIOD, .RxBuff = spi2RXbuf, .TxBuff = spi2TXbuf,
		.size = 0 };

HSPI sSpi1 = { .handle = &hspi1, .RxBuff = spi1RXbuf, .TxBuff = spi1TXbuf, .Timeout = 50, .size = 0 };
HSPI sSpi3 = { .handle = &hspi3, .RxBuff = spi3RXbuf, .TxBuff = spi3TXbuf, .Timeout = 50, .size = 0 };
HSPI sSpi5 = { .handle = &hspi5, .RxBuff = spi5RXbuf, .TxBuff = spi5TXbuf, .Timeout = 50, .size = 0 };

CIB_FPGA_Ver FpgaGetVersion(){
	CIB_FPGA_Ver ver;
	ver.reg32 = FpgaRead(FPGA_VERSION_REG);
	return ver;
}

uint32_t FpgaGetDate(){
	return FpgaRead(FPGA_DATE_REG);
}

uint32_t FpgaGetSysSync(){
	return FpgaRead(FPGA_SYS_SYN_REG);
}

uint32_t FpgaSetSysSync(uint32_t data){
	return FpgaWrite(FPGA_SYS_SYN_REG,data);
}
uint32_t FpgaRead(uint32_t addr){
	*((uint32_t*)sSpi2.TxBuff) = FPGA_SET_READ_BIT(addr);
	 sSpi2.size=2;
	if(RTG_SPI_TransmitReceiveSpi2()==HAL_OK)
		return *(((uint32_t*)sSpi2.RxBuff)+1);
	else return 0;
}
uint32_t FpgaWrite(uint32_t addr,uint32_t data){
	*((uint32_t*)sSpi2.TxBuff)  = FPGA_SET_WRITE_BIT(addr);
	*(((uint32_t*)sSpi2.TxBuff) + 1 )  = data;
	 sSpi2.size=2;
	if(RTG_SPI_TransmitReceiveSpi2()==HAL_OK)
		return *(((uint32_t*)sSpi2.RxBuff)+1);
	else return 0;
}


void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
	static portBASE_TYPE xHigherPriorityTaskWoken = pdTRUE;
	if (hspi == &hspi1) {
		sSpi1.Rx_ready = 1;
		memcpy(flashIDbuf, sSpi1.RxBuff, 20);
		flashFlag = 1;
	}
	else if (hspi == &hspi3) {
		sSpi3.Rx_ready = 1;
//		RTG_IMU_ADIS16547_Gyro_Accalrtion();
		YIELD_SEMAPHORE_GIVE_ISR(onSpiTxRx, xHigherPriorityTaskWoken);
	}
	else if (hspi == &hspi5) {
//		RTG_IMU_ADIS16467_Gyro_Accalrtion();
		sSpi5.Rx_ready = 1;
	}
}

/* SPI_Error_Code SPI Error Codes
 HAL_SPI_ERROR_NONE                            (0x00000000UL)   !< No error
 HAL_SPI_ERROR_MODF                            (0x00000001UL)   !< MODF error
 HAL_SPI_ERROR_CRC                             (0x00000002UL)   !< CRC error
 HAL_SPI_ERROR_OVR                             (0x00000004UL)   !< OVR error
 HAL_SPI_ERROR_FRE                             (0x00000008UL)   !< FRE error
 HAL_SPI_ERROR_DMA                             (0x00000010UL)   !< DMA transfer error
 HAL_SPI_ERROR_FLAG                            (0x00000020UL)   !< Error on RXNE/TXE/BSY/FTLVL/FRLVL Flag
 HAL_SPI_ERROR_ABORT                           (0x00000040UL)   !< Error during SPI Abort procedure
 HAL_SPI_ERROR_UDR                             (0x00000080UL)   !< Underrun error
 HAL_SPI_ERROR_TIMEOUT                         (0x00000100UL)   !< Timeout error
 HAL_SPI_ERROR_UNKNOW                          (0x00000200UL)   !< Unknown error
 HAL_SPI_ERROR_NOT_SUPPORTED                   (0x00000400UL)   !< Requested operation not supported
 */
void RTG_SPI_Error(HSPI *sSpi) {
	__HAL_SPI_CLEAR_MODFFLAG(sSpi->handle);
	__HAL_SPI_CLEAR_CRCERRFLAG(sSpi->handle);
	__HAL_SPI_CLEAR_OVRFLAG(sSpi->handle);
	__HAL_SPI_CLEAR_FREFLAG(sSpi->handle);
	__HAL_SPI_CLEAR_UDRFLAG(sSpi->handle);

	sSpi->handle->ErrorCode = HAL_SPI_ERROR_NONE;
	sSpi->handle->State = HAL_SPI_STATE_READY;
}
// don't wait for end
HAL_StatusTypeDef RTG_SPI_TransmitReceive_DMA(HSPI *spi, uint16_t *TxBuff, uint8_t *RxBuff, uint32_t size) {
	HAL_StatusTypeDef error;
	spi->Rx_ready = 0;
	error = HAL_SPI_TransmitReceive_DMA(spi->handle, (uint8_t*) TxBuff, RxBuff, size);
	return error;
}

// wait for end
HAL_StatusTypeDef RTG_SPI_TransmitReceive(HSPI *spi) {
	HAL_StatusTypeDef error;
	spi->Rx_ready = 0;
	error = HAL_SPI_TransmitReceive(spi->handle, spi->TxBuff, spi->RxBuff, spi->size, 0xffff);
	return error;
}
// wait for end
HAL_StatusTypeDef RTG_SPI_TransmitReceiveSpi2() {
	HAL_StatusTypeDef error;
	HAL_GPIO_WritePin(sSpi2.cs_port, sSpi2.cs_gpio, GPIO_PIN_SET);
	HAL_GPIO_WritePin(sSpi2.cs_port, sSpi2.cs_gpio, GPIO_PIN_RESET);
	error = HAL_SPI_TransmitReceive(sSpi2.handle, sSpi2.TxBuff, sSpi2.RxBuff, sSpi2.size, 0xffff);
	HAL_GPIO_WritePin(sSpi2.cs_port, sSpi2.cs_gpio, GPIO_PIN_SET);
	return error;
}

HAL_StatusTypeDef RTG_SPI_Receive(HSPI *spi, int size) {
	HAL_StatusTypeDef error;

	if (spi->handle->Init.Mode == SPI_MODE_MASTER) HAL_GPIO_WritePin(spi->cs_port, spi->cs_gpio, GPIO_PIN_SET);
	if (size > SPI_BUF_SIZE) size = SPI_BUF_SIZE;

	if (spi->handle->Init.Mode == SPI_MODE_MASTER) HAL_GPIO_WritePin(spi->cs_port, spi->cs_gpio, GPIO_PIN_RESET);
	spi->Rx_ready = 0;
	if (spi->handle->hdmatx && spi->handle->hdmarx) error = HAL_SPI_Receive_DMA(spi->handle, spi->RxBuff, size);
	else error = HAL_SPI_Receive_IT(spi->handle, spi->RxBuff, size);
	return error;
}

HAL_StatusTypeDef RTG_SPI_Transmit(HSPI *spi, uint8_t *TBuf, int size) {
	HAL_StatusTypeDef error;
	uint32_t tickstart;

	if (spi->handle->Init.Mode == SPI_MODE_MASTER) HAL_GPIO_WritePin(spi->cs_port, spi->cs_gpio, GPIO_PIN_SET);
	if (size > SPI_BUF_SIZE) size = SPI_BUF_SIZE;
	memcpy(spi->TxBuff, TBuf, size);
	memset(spi->RxBuff, 0, size);

	spi->Tx_ready = 0;
	if (spi->handle->Init.Mode == SPI_MODE_MASTER) HAL_GPIO_WritePin(spi->cs_port, spi->cs_gpio, GPIO_PIN_RESET);

	if (spi->handle->hdmatx && spi->handle->hdmarx) error = HAL_SPI_Transmit_DMA(spi->handle, spi->TxBuff, size);
	else error = HAL_SPI_Transmit_IT(spi->handle, spi->TxBuff, size);

	if ((spi->Timeout == 0U) || spi->handle->Init.Mode == SPI_MODE_SLAVE) return error;

	tickstart = HAL_GetTick();
	while (!spi->Tx_ready) {
		if (((HAL_GetTick() - tickstart) > (spi->Timeout * 2))) {
			if (HAL_SPI_Abort(spi->handle) == HAL_OK) {
				HAL_GPIO_WritePin(spi->cs_port, spi->cs_gpio, GPIO_PIN_SET);
				spi->Tx_ready = 1;
				return HAL_TIMEOUT;
			}
		}
	}
	spi->Tx_ready = 0;
	return error;
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi) {
	if (hspi == &hspi1) flasherrFlag = 1;
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
	if (hspi == &hspi3) sSpi3.Tx_ready = 1;
	else if (hspi == &hspi5) sSpi5.Tx_ready = 1;
	else if (hspi == &hspi1) {
		sSpi1.Tx_ready = 1;
		flashFlag = 1;
	}
//	else if (hspi == &hspi2)
//		sSpi2.Tx_ready = 1;
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) {
	if (hspi == &hspi3) sSpi3.Rx_ready = 1;
	else if (hspi == &hspi5) sSpi5.Rx_ready = 1;
	else if (hspi == &hspi1) {
		sSpi1.Rx_ready = 1;
		flashFlag = 1;
	}
//	else if (hspi == &hspi2)
//		sSpi2.Tx_ready = 1;
}

