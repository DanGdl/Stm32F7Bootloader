/*
 * bit.c
 *
 *  Created on: 26 Dec 2021
 *      Author: itzhaki
 */

#include <main.h>
#include <ethernetif.h>

//#include <portmacro.h>
#include <bit.h>
#include <spi.h>
#include <adc.h>
#include <timer.h>
#include <eth_rtg.h>
#include <SDRAM.h>
#include <SERVO.h>
#include <INT_FLASH.h>
#include <Serial_FLASH.h>
#include <ADIS16547.h>
#include <Safe_And_Arm.h>
#include <Battery.h>

extern TIM_HandleTypeDef htim5;
extern SERVO_BIT_VALUE_t servoBitValue;

DEVICE_COUNT_t DevCount = { 0 };

CIB_BIT_Msg PBIT_mes_st = { 0 };
CIB_BIT_Msg CBIT_mes_st = {0};


/********************** For internal CPU RAMs TEST *************************/
/* Allocating space for "ramtest" in the CPU internal RAM (RAM 128K) */
uint32_t ramtest __attribute__((section(".RAMTEST")));
/* Allocating space for "RAM1TEST" in the CPU internal RAM1 (RAM_D1 512K) */
uint32_t ram1test __attribute__((section(".RAM1TEST")));
/* Allocating space for "RAM2TEST" in the CPU internal RAM1 (RAM_D2 288K) */
uint32_t ram2test __attribute__((section(".RAM2TEST")));
/* Allocating space for "RAM3TEST" in the CPU internal RAM1 (RAM_D3 64K) */
/* Allocating space for "ITCMRAMTEST" in the CPU internal RAM1 (ITCMRAM 64K) */
uint32_t itcmRamTest __attribute__((section(".ITCMRAMTEST")));

uint32_t ram3test __attribute__((section(".RAM3TEST")));
uint32_t *pRamTest [] = { (uint32_t*) &ramtest, (uint32_t*) &ram1test, (uint32_t*) &ram2test, (uint32_t*) &ram3test,
		(uint32_t*) &itcmRamTest };

// Patterns to check for each ram
uint32_t ramTestValue [] = { 0xFFFFFFFF, 0x55555555, 0xAAAAAAAA, 0x00000000 };

#define MAX_TME_WAIT 50

void RTG_PBIT_FUNC() {
	uint16_t wait_count = 0;
	pPbitMes->header.count++;		// Add COUNTER to PBIT_mes buffer (always)

	pPbitMes->error_bits.SDRAM = RTG_SDRAM_bit( &pPbitMes->sdram );
	RTG_EXT_FLASH_bit		( &pPbitMes->external_flash );
	RTG_INT_Flash_BIT		( &pPbitMes->internal_flash );
	RTG_CpuInternalRamTest	( &pPbitMes->internal_rams);

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// PBIT Main Computer
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
	pPbitMes->mainComputer.FPGA_date.reg32 	= FpgaGetDate();
	pPbitMes->mainComputer.FPGA_ver		= FpgaGetVersion();
	pPbitMes->mainComputer.FPGA_SysSin	= FpgaGetSysSync();
	if(		pPbitMes->error_bits.INT_FLASH 	||
			pPbitMes->error_bits.EXT_FLASH	||
			pPbitMes->error_bits.INT_RAMS	||
			pPbitMes->error_bits.SDRAM		) pPbitMes->error_bits.MC = 1;
	pPbitMes->mainComputer.time_tag 	= GetTimeTag;

// wait for adis readied
	wait_count = 0;
	while( !ADS_IS_READED && wait_count < MAX_TME_WAIT ){
		RT_DELAY_MS(1);
		wait_count++;
	}
	if (!ADS_IS_READED) { /* Code for ERROR	*/
		pPbitMes->error_bits.I467 = 1;
		pPbitMes->error_bits.I547 = 1;
	}

// wait for Margalit  is readed
	wait_count = 0;
	while(  wait_count < MAX_TME_WAIT && !MGL_R_IS_FULL ){
			RT_DELAY_MS(1);
			wait_count++;
	}
	if ( !MGL_R_IS_FULL ) { /* Code for ERROR	*/

		pPbitMes->error_bits.TRF = 1;
	}
	MGL_SET_READED;

// wait for GPS version is readed
	wait_count = 0;
	while( !GPS_IS_READ_V && wait_count < MAX_TME_WAIT ){
			RT_DELAY_MS(1);
			wait_count++;
	}
	if ( !GPS_IS_READ_V || !GPS_IS_READED ) { /* Code for ERROR	*/
		pPbitMes->error_bits.GPS = 1;
	}

	/*-----------------------------------------------------------------------------------------------*/
	pPbitMes->header.time_tag = GetTimeTag;
	pPbitMes->checksum = RTG_CalculateChecksum((uint8_t*)pPbitMes, sizeof(CIB_PBIT_data));
	PBIT_ADD_COUNT;
	SET_PBIT_ENDED;
	CBIT_CURRENT_BUFF.status.PBIT_DONE = 1;
}
void RTG_CBIT_FUNC() {
	static uint8_t BIT_mes_to_send [RTG_MAX_ETH_BUFF];
	CBIT_CURRENT_BUFF.current_bit.results = 0;
#if BIT_SEND_TO_SNA == 1
	// S&A PBIT TX COMANDS// SEND detailed_status command
	RTG_detailed_status_tx();
#endif
	RTG_IMU_ADIS16547_CBIT(&CBIT_CURRENT_BUFF.imu_adis16547);
	RTG_IMU_ADIS16467_CBIT(&CBIT_CURRENT_BUFF.imu_adis16467);

	BattReadAll( &CBIT_CURRENT_BUFF.bms );

	CBIT_CURRENT_BUFF.adc = RTG_adc();
/*
 * Margalit updeted in RTG_Update_Margalit_CBIT (rtg_task.c)
 */
/*
 * GPS updeted in function onUARTxRxCallback (rtg_task.c)
 */
	CBIT_CURRENT_BUFF.header.time_tag = GetTimeTag;
	// Add COUNTER to CBIT_mes buffer (always)
	CBIT_CURRENT_BUFF.header.count++;
	CBIT_FULL_BUFF.hdr.seq++;

	taskENTER_CRITICAL();
	// Add CHECKSUM only after all test are finished (always)
	CBIT_CURRENT_BUFF.checksum =
		RTG_CalculateChecksum((uint8_t*)&CBIT_CURRENT_BUFF,CBIT_CURRENT_BUFF_SIZE);
	memcpy(BIT_mes_to_send, &CBIT_FULL_BUFF, CBIT_FULL_BUFF_SIZE );
	taskEXIT_CRITICAL();

	// Send BIT to UDP
	CBIT_ADD_COUNT;
	RTG_Udp_Send((char*) BIT_mes_to_send, CBIT_FULL_BUFF_SIZE);
}

/****************************************************************************************************/
// This test check the internal rams of the CPU by writing the values of 0xFFFFFFFF, 0x55555555,
// 0xAAAAAAAA, 0x00000000 to each memory location, read back and compare it.
//
// INTERNAL RAMS of the CPU:
// RAM NAME IN STM32H743XIHX_FLASH.ld file  (RAM NAME IN DATA SHEET)
//  --------------------------------------    --------------------
//                               |                  |
//                               |                  |
// RAM (DTCM), RAM1(AXI SRAM), RAM2(SRAM1, SRAM2, SRAM3), RAM3(SRAM4), ITCM
//
// EXT RAM MESSAGE STRUCTURE
// -------------------------------
// | TIME TAG(32) | Pass Fail(8) |
// -------------------------------
//
// Pass Fail register: 1 - FAIL, 0 - PASS
//
//   7   6   5      4       3      2      1     0
// -------------------------------------------------
// | 0 | 0 | 0 | ITCMRAM | RAM3 | RAM2 | RAM1 |RAM |
// -------------------------------------------------
uint16_t RTG_CpuInternalRamTest(CIB_PBIT_INTERNAL_RAMS *mes) {
	uint32_t *pVal;
	uint32_t val;
	uint8_t numbersOfMemory = 5;
	uint8_t dataType = 4;

	for (int i = 0; i < numbersOfMemory; i++) {
		// Get pointer to add in specific RAM in test
		pVal = (uint32_t*) pRamTest [i];

		for (int j = 0; j < dataType; j++) {
			// Write value to RAM
			*pVal = ramTestValue [j];

			// Read value from RAM
			val = *pVal;

			// Compare if error and write which RAM memory fail
			if (val != ramTestValue [j]) {
				switch (i) {
					case 0:
						mes->pass_or_fail.RAM = 1;
					break;
					case 1:
						mes->pass_or_fail.RAM1 = 1;
					break;
					case 2:
						mes->pass_or_fail.RAM2 = 1;
					break;
					case 3:
						mes->pass_or_fail.RAM3 = 1;
					break;
					case 4:
						mes->pass_or_fail.ITCMRAM = 1;
					break;
					default:
						;
				}
				break;
			}
		}
	}

	// Add TIME TAG
	pPbitMes->error_bits.INT_RAMS = !(!(mes->pass_or_fail.results));
	mes->time_tag = GetTimeTag;
	return 1;
}

