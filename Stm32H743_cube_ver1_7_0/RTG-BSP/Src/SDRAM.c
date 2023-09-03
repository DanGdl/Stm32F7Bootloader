/*
 * SDRAM.c
 *
 *  Created on: Aug 9, 2021
 *      Author: itzhaki
 */

#include <bit.h>
#include <core_cm7.h>
#include <eth_rtg.h>
#include <RTG_main.h>
#include <stdio.h>
#include <stdlib.h>
#include <stm32h7xx_ll_fmc.h>
#include <string.h>
#include <SDRAM.h>
#include <timer.h>

extern UDP_t SDRAM_udp;

FMC_SDRAM_CommandTypeDef command;



/* Status variables */
__IO uint32_t uwWriteReadStatus = 0;

static void RTG_SDRAM_Initialization_Sequence(SDRAM_HandleTypeDef *hsdram,
		FMC_SDRAM_CommandTypeDef *Command)
{
	__IO uint32_t tmpmrd = 0;
	/* Configure a clock configuration enable command */
	Command->CommandMode = FMC_SDRAM_CMD_CLK_ENABLE;
	Command->CommandTarget = FMC_SDRAM_CMD_TARGET_BANK2;
	Command->AutoRefreshNumber = 1;
	Command->ModeRegisterDefinition = 0;

	/* Send the command */
	HAL_SDRAM_SendCommand(hsdram, Command, SDRAM_TIMEOUT);

	/* Inserted delay is equal to 1 ms due to systick time base unit (ms) */
	RT_DELAY_MS(1);

	/* Configure a PALL (precharge all) command */
	Command->CommandMode = FMC_SDRAM_CMD_PALL;
	Command->CommandTarget = FMC_SDRAM_CMD_TARGET_BANK2;
	Command->AutoRefreshNumber = 1;
	Command->ModeRegisterDefinition = 0;

	/* Send the command */
	HAL_SDRAM_SendCommand(hsdram, Command, SDRAM_TIMEOUT);

	/* Configure a Auto-Refresh command */
	Command->CommandMode = FMC_SDRAM_CMD_AUTOREFRESH_MODE;
	Command->CommandTarget = FMC_SDRAM_CMD_TARGET_BANK2;
	Command->AutoRefreshNumber = 8;
	Command->ModeRegisterDefinition = 0;

	/* Send the command */
	HAL_SDRAM_SendCommand(hsdram, Command, SDRAM_TIMEOUT);

	/* Program the external memory mode register */
	tmpmrd = (uint32_t) SDRAM_MODEREG_BURST_LENGTH_1 |
	SDRAM_MODEREG_BURST_TYPE_SEQUENTIAL |
	SDRAM_MODEREG_CAS_LATENCY_3 |
	SDRAM_MODEREG_OPERATING_MODE_STANDARD |
	SDRAM_MODEREG_WRITEBURST_MODE_SINGLE;

	Command->CommandMode = FMC_SDRAM_CMD_LOAD_MODE;
	Command->CommandTarget = FMC_SDRAM_CMD_TARGET_BANK2;
	Command->AutoRefreshNumber = 1;
	Command->ModeRegisterDefinition = tmpmrd;

	/* Send the command */
	HAL_SDRAM_SendCommand(hsdram, Command, SDRAM_TIMEOUT);

	/* Set the refresh rate counter */
	/* Set the device refresh rate */
	HAL_SDRAM_ProgramRefreshRate(hsdram, REFRESH_COUNT);
}

void RTG_SDRAM_int(void)
{
	RTG_SDRAM_Initialization_Sequence(&hsdram1, &command);
}

/**
 * @brief  Write data to SDRAM.
 * @param  add: Start address of write
 * @param  buf: Pointer to buffer data
 * @retval error
 */
uint8_t RTG_Write_SDRAM(uint32_t add, uint32_t *data)
{
	//check if address is out of SDRAM limit
	if (add > SDRAM_SIZE_16M)
		return SRAM_ADD_ERROR;

	*(__IO uint32_t*) (SDRAM_BANK_ADDR + add * SDRAM_MEM_BUS_WIDTH_32) = *data;

	return SRAM_OK;
}

/**
 * @brief  Read data from SDRAM.
 * @param  add: Start address of read
 * @param  buf: Pointer to buffer data
 * @retval error
 */
uint8_t RTG_Read_SDRAM(uint32_t add, uint32_t *data)
{
	//check if address is out of SDRAM limit
	if (add > SDRAM_SIZE_16M)
		return SRAM_ADD_ERROR;

	*data = *(__IO uint32_t*) (SDRAM_BANK_ADDR + add * SDRAM_MEM_BUS_WIDTH_32);

	return SRAM_OK;
}

/**
 * @brief  Write buffer to SDRAM.
 * @param  add: Start address of write
 * @param  buf: Pointer to buffer data
 * @param  size: Amount of data to write
 * @retval error
 */
uint8_t RTG_Write_Buff_To_SDRAM(uint32_t add, uint32_t *buf, uint32_t size)
{
	uint32_t i;

	//Check if address is out of SDRAM limit
	if ((add + size) > SDRAM_SIZE)
		return SRAM_ADD_ERROR;

	for (i = 0; i < size; i++)
		RTG_Write_SDRAM(add + i, &buf[i]);

	return SRAM_OK;
}

/**
 * @brief  Read from SDRAM to buffer.
 * @param  add: Start address of read
 * @param  buf: Pointer to buffer data
 * @param  size: Amount of data to read
 * @retval error
 */
uint8_t RTG_Read_Buff_From_SDRAM(uint32_t add, uint32_t *buf, uint32_t size)
{
	uint32_t i;

	//check if address is out of SDRAM limit
	if ((add + size) > SDRAM_SIZE)
		return SRAM_ADD_ERROR;

	for (i = 0; i < size; i++)
		RTG_Read_SDRAM(add + i, &buf[i]);

	return SRAM_OK;
}



/**
 * @brief  Write data to All SDRAM (32Mb = 1M * 32) , Read and compare
 * @param  data: Data write to SDRAM
 * @param  testType: memory test type (Random)
 * @retval error
 */

/****************************************************************************/
// This test check the random value write to the SDRAM then read and compare
//
// SDRAM MESSAGE STRUCTURE
// -------------------------------
// | Time Tag(32) | Pass Fail(8) |
// -------------------------------
//
// Pass Fail register: 1 - FAIL, 0 - PASS
//   7   6   5   4   3   2            1                   0
//>-----------------------------------------------------------------
//   0 | 0 | 0 | 0 | 0 | 0 | SRAM_INTEGRITY_ERROR | SRAM_ADD_ERROR |
//>-----------------------------------------------------------------
uint8_t RTG_Memory_Test_BIT_RANDOM(CIB_PBIT_SDRAM *mes) {
	uint32_t writeData;
	uint32_t readData;
	uint32_t uwIndex = 0;
	uint32_t base_address_test = SDRAM_BANK_ADDR + SDRAM_SIZE - SDRAM_TEST_SIZE;     // Last 1M of the SDRAM
	uint32_t *pTemp = (uint32_t*) base_address_test;

	mes->pass_or_fail = 0;
	for (uwIndex = 0; uwIndex < ( SDRAM_TEST_SIZE); uwIndex += SDRAM_MEM_BUS_WIDTH_32) {
		if ((base_address_test + uwIndex) >= ( SDRAM_BANK_ADDR + SDRAM_SIZE)) break;
		pTemp = (uint32_t*) (base_address_test + uwIndex);
		writeData = (uint32_t) uwIndex;
//		writeData = (uint32_t)rand();
		if (HAL_SDRAM_Write_32b(&hsdram1, pTemp, &writeData, 1) != SRAM_OK) {
			mes->pass_or_fail |= SRAM_ADD_ERROR;
			break;
		}
		if ((HAL_SDRAM_Read_32b(&hsdram1, pTemp, &readData, 1)) != SRAM_OK) {
			mes->pass_or_fail |= SRAM_ADD_ERROR;
			break;
		}
	}

	// Add TIME TAG
	mes->time_tag = GetTimeTag;
	return !(!mes->pass_or_fail);
}


uint16_t RTG_SDRAM_bit(CIB_PBIT_SDRAM *mes)
{
	mes->pass_or_fail = RTG_Memory_Test_BIT_RANDOM(mes);
	mes->time_tag = GetTimeTag;
	return mes->pass_or_fail;
}
