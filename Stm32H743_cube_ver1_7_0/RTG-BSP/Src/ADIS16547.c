/*
 * ADIS16547.c
 *
 *  Created on: 16 Feb 2022
 *      Author: itzhaki
 */

#include <ADIS16547.h>
#include <bit.h>
#include <discrete.h>
#include <spi.h>
#include <string.h>
#include <timer.h>
#include <CIB_Protocol.h>

extern HSPI sSpi3;

extern CIB_400_Hz_Msg CIB_400_Hz;

uint16_t IMU_ADIS16547_CMD_Buffer [20] = { 0 };
uint16_t IMU_ADIS16547_CMD_Buffer_size;

void RTG_IMU_ADIS16547_INIT(void) {
	uint16_t *txBuff = (uint16_t*) sSpi3.TxBuff;
	uint8_t ofset = 0;

	txBuff [ofset++] = RTG_IMU_CMD(IMU_WRITE, IMU_PAGE_ID_ADD, IMU_PAGE_ID_3);     // Set PAGE_ID to page 3
	txBuff [ofset++] = RTG_IMU_CMD(IMU_WRITE, IMU_SYNC_SCALE_1, IMU_SYNC_SCALE_D_1);	// configure control (add 21/11/2022)
	txBuff [ofset++] = RTG_IMU_CMD(IMU_WRITE, IMU_SYNC_SCALE_2, IMU_SYNC_SCALE_D_2);
	txBuff [ofset++] = RTG_IMU_CMD(IMU_WRITE, IMU_DECIMATION_CMD_1, IMU_DECIMATION_400Hz);     // configure to 400Hz
	txBuff [ofset++] = RTG_IMU_CMD(IMU_WRITE, IMU_DECIMATION_CMD_2, IMU_NULL);     // configure to 400Hz
	txBuff [ofset++] = RTG_IMU_CMD(IMU_WRITE, IMU_FNCTION_CTRL_1, IMU_FNCTION_C_D_1);	// configure control (add 21/11/2022)
	txBuff [ofset++] = RTG_IMU_CMD(IMU_WRITE, IMU_FNCTION_CTRL_2, IMU_FNCTION_C_D_2);

	sSpi3.size = ofset;
	RTG_SPI_TransmitReceive(&sSpi3);
	RTG_IMU_ADIS16547_CMD_Buffer_init();
}
/*
 * IMU start self test
 *
 */
void RTG_IMU_ADIS16547_DIAG_STS(void) {
	uint16_t *txBuff = (uint16_t*) sSpi3.TxBuff;
	uint8_t ofset = 0;

	txBuff [ofset++] = RTG_IMU_CMD(IMU_WRITE, IMU_PAGE_ID_ADD, IMU_PAGE_ID_3);		// Set PAGE_ID to page 3
	txBuff [ofset++] = RTG_IMU_CMD(IMU_WRITE, IMU_GLOB_CMD_1, IMU_SELF_TEST);		// Self test phase one
	txBuff [ofset++] = RTG_IMU_CMD(IMU_WRITE, IMU_GLOB_CMD_2, IMU_NULL);     // Self test phase two
	sSpi3.size = ofset;
	RTG_SPI_TransmitReceive(&sSpi3);
}

void RTG_IMU_ADIS16547_CMD_Buffer_init() {
	uint8_t ofset = 0;
	IMU_ADIS16547_CMD_Buffer [ofset++] = RTG_IMU_CMD(IMU_WRITE, IMU_PAGE_ID_ADD, IMU_PAGE_ID_0);	// Set PAGE_ID to page 0
	IMU_ADIS16547_CMD_Buffer [ofset++] = RTG_IMU_CMD(IMU_READ, X_DELTANG_LOW, IMU_NULL);		// READ DELTANG_LOW
	IMU_ADIS16547_CMD_Buffer [ofset++] = RTG_IMU_CMD(IMU_READ, X_DELTANG_OUT, IMU_NULL);		//READ DELTANG_OUT
	IMU_ADIS16547_CMD_Buffer [ofset++] = RTG_IMU_CMD(IMU_READ, Y_DELTANG_LOW, IMU_NULL);		// READ Y_DELTANG_LOW
	IMU_ADIS16547_CMD_Buffer [ofset++] = RTG_IMU_CMD(IMU_READ, Y_DELTANG_OUT, IMU_NULL);		// READ Y_DELTANG_OUT
	IMU_ADIS16547_CMD_Buffer [ofset++] = RTG_IMU_CMD(IMU_READ, Z_DELTANG_LOW, IMU_NULL);		// READ Z_DELTANG_LOW
	IMU_ADIS16547_CMD_Buffer [ofset++] = RTG_IMU_CMD(IMU_READ, Z_DELTANG_OUT, IMU_NULL);		// READ Z_DELTANG_OUT
	IMU_ADIS16547_CMD_Buffer [ofset++] = RTG_IMU_CMD(IMU_READ, X_DELTVEL_LOW, IMU_NULL);		// READ DELTANG_LOW
	IMU_ADIS16547_CMD_Buffer [ofset++] = RTG_IMU_CMD(IMU_READ, X_DELTVEL_OUT, IMU_NULL);		// READ DELTANG_OUT
	IMU_ADIS16547_CMD_Buffer [ofset++] = RTG_IMU_CMD(IMU_READ, Y_DELTVEL_LOW, IMU_NULL);		// READ Y_DELTANG_LOW
	IMU_ADIS16547_CMD_Buffer [ofset++] = RTG_IMU_CMD(IMU_READ, Y_DELTVEL_OUT, IMU_NULL);		// READ Y_DELTANG_OUT
	IMU_ADIS16547_CMD_Buffer [ofset++] = RTG_IMU_CMD(IMU_READ, Z_DELTVEL_LOW, IMU_NULL);		// READ Z_DELTANG_LOW
	IMU_ADIS16547_CMD_Buffer [ofset++] = RTG_IMU_CMD(IMU_READ, Z_DELTVEL_OUT, IMU_NULL);		// READ Z_DELTANG_OUT

	IMU_ADIS16547_CMD_Buffer [ofset++] = RTG_IMU_CMD(IMU_READ, IMU_DATA_CNT_L, IMU_NULL);		// READ DATA Counter

	IMU_ADIS16547_CMD_Buffer [ofset++] = RTG_IMU_CMD(IMU_READ, IMU_NULL, IMU_NULL);				// READ dummy
	IMU_ADIS16547_CMD_Buffer_size = ofset;
}

void RTG_IMU_ADIS16547_Gyro_Accalrtion(void) {
	uint8_t ofset = 0;
	// Copy relevant data from sSpi3.RxBuff
	memcpy(&CIB_400_Hz.data.adis16547.X_D_Ang_i, &sSpi3.RxBuff [ofset = 4],  4);
	memcpy(&CIB_400_Hz.data.adis16547.Y_D_Ang_i, &sSpi3.RxBuff [ofset += 4], 4);
	memcpy(&CIB_400_Hz.data.adis16547.Z_D_Ang_i, &sSpi3.RxBuff [ofset += 4], 4);
	memcpy(&CIB_400_Hz.data.adis16547.X_D_Vel_i, &sSpi3.RxBuff [ofset += 4], 4);
	memcpy(&CIB_400_Hz.data.adis16547.Y_D_Vel_i, &sSpi3.RxBuff [ofset += 4], 4);
	memcpy(&CIB_400_Hz.data.adis16547.Z_D_Vel_i, &sSpi3.RxBuff [ofset += 4], 4);

	memcpy(&CIB_400_Hz.data.adis16547.Data_count, &sSpi3.RxBuff[ofset += 2], 2);

	CIB_400_Hz.data.adis16547.TIME_STAMP = GetTimeTag;
}

uint16_t RTG_IMU_ADIS16547_PBIT(CIB_PBIT_IMU_ADIS16547 *ADIS16547_VAL) {
	uint16_t *txBuff = (uint16_t*) sSpi3.TxBuff;
	uint8_t ofset = 0;

	txBuff [ofset++] = RTG_IMU_CMD( IMU_WRITE, 	IMU_PAGE_ID_ADD, 	IMU_PAGE_ID_0);  	// Set PAGE_ID to page 0
	txBuff [ofset++] = RTG_IMU_CMD( IMU_READ, 	IMU_PROD_ID, 		IMU_NULL);     		// READ PROD_ID
	txBuff [ofset++] = RTG_IMU_CMD( IMU_READ, 	IMU_NULL, 			IMU_NULL);     		// READ dummy

	sSpi3.size = ofset;
	pPbitMes->error_bits.I547 =  !!RTG_SPI_TransmitReceive(&sSpi3);
	// Copy relevant data from sSpi3.RxBuff
	memcpy(&ADIS16547_VAL->prod_id, 	&sSpi3.RxBuff [ofset = 4], 2);

	if ((sSpi3.RxBuff [2] != 0) || (sSpi3.RxBuff [3] != 0)) {
		pPbitMes->error_bits.I547 = 1;     // keep it in the the current error register
	}

	ofset = 0;
	txBuff [ofset++] = RTG_IMU_CMD( IMU_WRITE, 	IMU_PAGE_ID_ADD, 	IMU_PAGE_ID_3);     // Set PAGE_ID to page 3
	txBuff [ofset++] = RTG_IMU_CMD( IMU_READ, 	IMU_FIRM_REV, 		IMU_NULL);     		// READ FIRM_REV
	txBuff [ofset++] = RTG_IMU_CMD( IMU_READ, 	IMU_FIRM_DM, 		IMU_NULL);     		// READ FIRM_DM
	txBuff [ofset++] = RTG_IMU_CMD( IMU_READ, 	IMU_FIRM_Y, 		IMU_NULL);     		// READ FIRM_Y
	txBuff [ofset++] = RTG_IMU_CMD( IMU_READ, 	IMU_BOOT_REV, 		IMU_NULL);     		// READ BOOT_REV
	txBuff [ofset++] = RTG_IMU_CMD( IMU_READ, 	IMU_NULL, 			IMU_NULL);     		// READ dummy

	sSpi3.size = ofset;
	pPbitMes->error_bits.I547 = !!RTG_SPI_TransmitReceive(&sSpi3);
	// Copy relevant data from sSpi3.RxBuff
	memcpy(&ADIS16547_VAL->firm_rev, 	&sSpi3.RxBuff [ofset = 4],  2);
	memcpy(&ADIS16547_VAL->firm_dm, 	&sSpi3.RxBuff [ofset += 2], 2);
	memcpy(&ADIS16547_VAL->firm_y, 		&sSpi3.RxBuff [ofset += 2], 2);
	memcpy(&ADIS16547_VAL->boot_rev, 	&sSpi3.RxBuff [ofset += 2], 2);

	ofset = 0;
	txBuff [ofset++] = RTG_IMU_CMD( IMU_WRITE, 	IMU_PAGE_ID_ADD, 	IMU_PAGE_ID_4);     // Set PAGE_ID to page 4
	txBuff [ofset++] = RTG_IMU_CMD( IMU_READ, 	IMU_SERIAL_NUM, 	IMU_NULL);     		// READ IMU_SERIAL_NUM
	txBuff [ofset++] = RTG_IMU_CMD( IMU_READ, 	IMU_NULL, 			IMU_NULL);     		// READ dummy

	sSpi3.size = ofset;
	pPbitMes->error_bits.I547 = !!RTG_SPI_TransmitReceive(&sSpi3);
	// Copy relevant data from sSpi3.RxBuff
	memcpy(&ADIS16547_VAL->imu_serial_num, 	&sSpi3.RxBuff [4], 2);
//	ADIS16547_VAL->TimeTag = GetTimeTag;

	return (pPbitMes->error_bits.I547);
}

uint16_t RTG_IMU_ADIS16547_CBIT(CIB_CBIT_IMU_ADIS16547 *ADIS16547_VAL) {
	uint16_t *txBuff = (uint16_t*) sSpi3.TxBuff;
	uint8_t ofset = 0;
	CBIT_CURRENT_BUFF.current_bit.I547 = 0;

	txBuff [ofset++] = RTG_IMU_CMD( IMU_WRITE, 	IMU_PAGE_ID_ADD, 	IMU_PAGE_ID_0);  	// Set PAGE_ID to page 0
	txBuff [ofset++] = RTG_IMU_CMD( IMU_READ, 	IMU_SYS_E_FLAG, 	IMU_NULL);     		// READ SYS_E_FLAG
	txBuff [ofset++] = RTG_IMU_CMD( IMU_READ, 	IMU_DIAG_STS, 		IMU_NULL);     		// READ IMU_DIAG_STS
	txBuff [ofset++] = RTG_IMU_CMD( IMU_READ, 	IMU_TEMP_OUT, 		IMU_NULL);     		// READ TEMP_OUT
	txBuff [ofset++] = RTG_IMU_CMD( IMU_READ, 	IMU_NULL, 			IMU_NULL);     		// READ dummy

	sSpi3.size = ofset;
	CBIT_CURRENT_BUFF.current_bit.I547 = !! RTG_SPI_TransmitReceive(&sSpi3);
	// Copy relevant data from sSpi3.RxBuff
	memcpy(&ADIS16547_VAL->sys_e_flag, 	&sSpi3.RxBuff [ofset = 4],  2);
	memcpy(&ADIS16547_VAL->diag_sts, 	&sSpi3.RxBuff [ofset += 2], 2);
	memcpy(&ADIS16547_VAL->temp_out, 	&sSpi3.RxBuff [ofset += 2], 2);

	ADIS16547_VAL->time_tag = GetTimeTag;

	CBIT_CURRENT_BUFF.current_bit.I547 |= ( !!ADIS16547_VAL->diag_sts.flags ) || ( !!ADIS16547_VAL->sys_e_flag.flags );
	CBIT_CURRENT_BUFF.sticky_bit.I547  |= CBIT_CURRENT_BUFF.current_bit.I547;

	return (CBIT_CURRENT_BUFF.current_bit.I547);
}



