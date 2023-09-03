/*
 * ADIS16467.c
 *
 *  Created on: 17 Feb 2022
 *      Author: itzhaki
 */

#include <ADIS16547.h>
#include <bit.h>
#include <RTG_main.h>
#include <spi.h>
#include <string.h>
#include <sys/_stdint.h>
#include <timer.h>

extern HSPI sSpi5;

CIB_400_Hz_Msg CIB_400_Hz;

uint16_t IMU_ADIS16467_CMD_Buffer [20] = { 0 };
uint16_t IMU_ADIS16467_CMD_Buffer_size;

void RTG_IMU_ADIS16467_INIT(void) {
	uint16_t *txBuff = (uint16_t*) sSpi5.TxBuff;
	CIB_400_Hz.hdr.length = sizeof(CIB_400_Hz) - sizeof(CIB_400_Hz.hdr);

	uint8_t ofset = 0;
	txBuff [ofset++] = RTG_IMU_CMD(IMU_WRITE, IMU_PAGE_ID_ADD, IMU_PAGE_ID_0);     // Set PAGE_ID to page 0
	txBuff [ofset++] = RTG_IMU_CMD(IMU_WRITE, IMU_ADIS16467_UP_SCALE_1, IMU_ADIS16467_UP_SCALE_D_1);
	txBuff [ofset++] = RTG_IMU_CMD(IMU_WRITE, IMU_ADIS16467_UP_SCALE_2, IMU_ADIS16467_UP_SCALE_D_2);
	txBuff [ofset++] = RTG_IMU_CMD(IMU_WRITE, IMU_ADIS16467_DECIMATION_CMD_1, IMU_ADIS16467_DECIMATION_400Hz);     // configure to 400Hz
	txBuff [ofset++] = RTG_IMU_CMD(IMU_WRITE, IMU_ADIS16467_DECIMATION_CMD_2, IMU_NULL);     // configure to 400Hz
	txBuff [ofset++] = RTG_IMU_CMD(IMU_WRITE, IMU_ADIS16467_MSC_CTL_1, IMU_ADIS16467_MSC_CTL_D_1);     // (21/11/2022)
	txBuff [ofset++] = RTG_IMU_CMD(IMU_WRITE, IMU_ADIS16467_MSC_CTL_2, IMU_ADIS16467_MSC_CTL_D_2);
	//ADD transmit to SPI
	sSpi5.size = ofset;
	RTG_SPI_TransmitReceive(&sSpi5);
	RTG_IMU_ADIS16467_CMD_Buffer_init();
}

/*
 * IMU start self test
 *
 */
void RTG_IMU_ADIS16467_DIAG_STS(void) {
	uint16_t *txBuff = (uint16_t*) sSpi5.TxBuff;
	uint8_t ofset = 0;
	txBuff [ofset++] = RTG_IMU_CMD(IMU_WRITE, IMU_PAGE_ID_ADD, IMU_PAGE_ID_0);     // Set PAGE_ID to page 0
	txBuff [ofset++] = RTG_IMU_CMD(IMU_WRITE, IMU_ADIS16467__GLOB_CMD_1,     // Self test phase one
			IMU_ADIS16467__SELF_TEST);
	txBuff [ofset++] = RTG_IMU_CMD(IMU_WRITE, IMU_ADIS16467__GLOB_CMD_2, IMU_NULL);     // Self test phase two
	//ADD transmit to SPI
	sSpi5.size = ofset;
	RTG_SPI_TransmitReceive(&sSpi5);
	// 25mSec Delay for IMU to finish the self test
}

void RTG_IMU_ADIS16467_CMD_Buffer_init() {
	uint8_t ofset = 0;
	IMU_ADIS16467_CMD_Buffer [ofset++] = RTG_IMU_CMD(IMU_WRITE, IMU_PAGE_ID_ADD, IMU_PAGE_ID_0);	// Set PAGE_ID to page 0
	IMU_ADIS16467_CMD_Buffer [ofset++] = RTG_IMU_CMD(IMU_READ, IMU_ADIS16467_X_DELTANG_LOW, IMU_NULL);	// READ DELTANG_LOW
	IMU_ADIS16467_CMD_Buffer [ofset++] = RTG_IMU_CMD(IMU_READ, IMU_ADIS16467_X_DELTANG_OUT, IMU_NULL);	//READ DELTANG_OUT
	IMU_ADIS16467_CMD_Buffer [ofset++] = RTG_IMU_CMD(IMU_READ, IMU_ADIS16467_Y_DELTANG_LOW, IMU_NULL);	// READ Y_DELTANG_LOW
	IMU_ADIS16467_CMD_Buffer [ofset++] = RTG_IMU_CMD(IMU_READ, IMU_ADIS16467_Y_DELTANG_OUT, IMU_NULL);	// READ Y_DELTANG_OUT
	IMU_ADIS16467_CMD_Buffer [ofset++] = RTG_IMU_CMD(IMU_READ, IMU_ADIS16467_Z_DELTANG_LOW, IMU_NULL);	// READ Z_DELTANG_LOW
	IMU_ADIS16467_CMD_Buffer [ofset++] = RTG_IMU_CMD(IMU_READ, IMU_ADIS16467_Z_DELTANG_OUT, IMU_NULL);	// READ Z_DELTANG_OUT

	IMU_ADIS16467_CMD_Buffer [ofset++] = RTG_IMU_CMD(IMU_READ, IMU_NULL, IMU_NULL);     // READ dummy
	IMU_ADIS16467_CMD_Buffer_size = ofset;
}

void RTG_IMU_ADIS16467_Gyro_Accalrtion(void) {
	uint8_t ofset = 0;
	// Copy relevant data from sSpi5.RxBuff
	memcpy(&CIB_400_Hz.data.adis16467.X_D_Ang_i, &sSpi5.RxBuff [ofset = 4], 4);
	memcpy(&CIB_400_Hz.data.adis16467.Y_D_Ang_i, &sSpi5.RxBuff [ofset += 4], 4);
	memcpy(&CIB_400_Hz.data.adis16467.Z_D_Ang_i, &sSpi5.RxBuff [ofset += 4], 4);

	CIB_400_Hz.data.adis16467.TIME_STAMP = GetTimeTag;
}

uint16_t RTG_IMU_ADIS16467_PBIT(CIB_PBIT_IMU_ADIS16467 *ADIS16467_VAL) {
	uint16_t *txBuff = (uint16_t*) sSpi5.TxBuff;
	uint8_t ofset = 0;

	txBuff [ofset++] = RTG_IMU_CMD(IMU_WRITE, 	IMU_PAGE_ID_ADD, IMU_PAGE_ID_0);     // Set PAGE_ID to page 0
	txBuff [ofset++] = RTG_IMU_CMD(IMU_READ, 	IMU_ADIS16467__PROD_ID, IMU_NULL);     // READ PROD_ID
	txBuff [ofset++] = RTG_IMU_CMD(IMU_READ, 	IMU_ADIS16467__FIRM_REV, IMU_NULL);     // READ FIRM_REV
	txBuff [ofset++] = RTG_IMU_CMD(IMU_READ, 	IMU_ADIS16467__FIRM_DM, IMU_NULL);     // READ FIRM_DM
	txBuff [ofset++] = RTG_IMU_CMD(IMU_READ, 	IMU_ADIS16467__FIRM_Y, IMU_NULL);     // READ FIRM_Y
	txBuff [ofset++] = RTG_IMU_CMD(IMU_READ, 	IMU_ADIS16467__SERIAL_NUM, IMU_NULL);     // READ IMU_SERIAL_NUM
	txBuff [ofset++] = RTG_IMU_CMD(IMU_READ, 	IMU_NULL, IMU_NULL);     // READ dummy
	// set the number of command send to IMU_ADIS16467
	sSpi5.size = ofset;
	pPbitMes->error_bits.I467 = !!RTG_SPI_TransmitReceive(&sSpi5);
	// Copy relevant data from sSpi5.RxBuff
	memcpy(&ADIS16467_VAL->prod_id, 	&sSpi5.RxBuff [ofset = 4], 2);
	memcpy(&ADIS16467_VAL->firm_rev, 	&sSpi5.RxBuff [ofset += 2], 2);
	memcpy(&ADIS16467_VAL->firm_dm, 	&sSpi5.RxBuff [ofset += 2], 2);
	memcpy(&ADIS16467_VAL->firm_y, 		&sSpi5.RxBuff [ofset += 2], 2);
	memcpy(&ADIS16467_VAL->imu_serial_num, 	&sSpi5.RxBuff [ofset += 2], 2);

	return (pPbitMes->error_bits.I467);
}

uint16_t RTG_IMU_ADIS16467_CBIT(CIB_CBIT_IMU_ADIS16467 *ADIS16467_VAL) {
	uint16_t *txBuff = (uint16_t*) sSpi5.TxBuff;
	uint8_t ofset = 0;
	CBIT_CURRENT_BUFF.current_bit.I467 = 0;

	txBuff [ofset++] = RTG_IMU_CMD(IMU_WRITE, 	IMU_PAGE_ID_ADD, IMU_PAGE_ID_0);     // Set PAGE_ID to page 0
	txBuff [ofset++] = RTG_IMU_CMD(IMU_READ, 	IMU_ADIS16467__DIAG_STS, IMU_NULL);     // READ IMU_DIAG_STS
	txBuff [ofset++] = RTG_IMU_CMD(IMU_READ, 	IMU_ADIS16467__TEMP_OUT, IMU_NULL);     //READ TEMP_OUT

	// set the number of command send to IMU_ADIS16467
	sSpi5.size = ofset;
	CBIT_CURRENT_BUFF.current_bit.I467 = !! RTG_SPI_TransmitReceive(&sSpi5);
	// Copy relevant data from sSpi5.RxBuff
	memcpy(&ADIS16467_VAL->diag_sts, 	&sSpi5.RxBuff [ofset = 4], 2);
	memcpy(&ADIS16467_VAL->temp_out, 	&sSpi5.RxBuff [ofset += 2], 2);
	ADIS16467_VAL->time_tag = GetTimeTag;

	CBIT_CURRENT_BUFF.current_bit.I467 |= ( !!ADIS16467_VAL->diag_sts.flags ) ;
	CBIT_CURRENT_BUFF.sticky_bit.I467  |= CBIT_CURRENT_BUFF.current_bit.I467;

	return (CBIT_CURRENT_BUFF.current_bit.I467);
}

