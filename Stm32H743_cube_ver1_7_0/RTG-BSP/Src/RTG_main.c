/*
 * RTG_main.c
 *
 *  Created on: 6 ביוני 2021
 *      Author: RTG
 *      +++++++++++
 */

/**
 * @brief  System Clock Configuration to 400MHz
 *         The system Clock is configured as follow :
 *            System Clock source            = PLL (HSE)
 *            SYSCLK(Hz)                     = 400000000 (CPU Clock)
 *            HCLK(Hz)                       = 200000000 (AXI and AHBs Clock)
 *            AHB Prescaler                  = 2
 *            D1 APB3 Prescaler              = 2 (APB3 Clock  100MHz)
 *            D2 APB1 Prescaler              = 2 (APB1 Clock  100MHz)
 *            D2 APB2 Prescaler              = 2 (APB2 Clock  100MHz)
 *            D3 APB4 Prescaler              = 2 (APB4 Clock  100MHz)
 *            HSE Frequency(Hz)              = 25000000
 *            PLL_M                          = 5
 *            PLL_N                          = 160
 *            PLL_P                          = 2
 *            PLL_Q                          = 4
 *            PLL_R                          = 2
 * @param  None
 * @retval None
 */

#include <stdio.h>
#include <string.h>

#include <RTG_main.h>
#include <bit.h>
#include <adc.h>
#include <rtg_tasks.h>
#include <uarts.h>
#include <version.h>
#include <Safe_And_Arm.h>
#include <SERVO.h>
#include <SDRAM.h>
#include <IMU-TRF-90M.h>
#include <ADIS16547.h>
#include <discrete.h>

extern TIM_HandleTypeDef htim5;

uint8_t EthMessageBuffer[ETH_QUEUE_LENGTH][RTG_MAX_ETH_BUFF];
StaticQueue_t xEthStaticQueue;
uint8_t ucEthQueueStorageArea[ ETH_QUEUE_LENGTH * sizeof(EHT_MessageToQueue_t) ];
QueueHandle_t xEthQueue;

StaticQueue_t xUartStaticQueue;
uint8_t ucUartQueueStorageArea[ UART_QUEUE_LENGTH * sizeof(Uart_MessageToQueue_t) ];
QueueHandle_t xUartQueue;

StaticQueue_t xEthSendStaticQueue;
uint8_t ucEthSendQueueStorageArea[ EthSend_QUEUE_LENGTH * sizeof(EHT_SendMsgToQueue_t) ];
QueueHandle_t xEthSendQueue;

/*
 * Run Init code before sceduler
 * (main.c)
 */
void RTG_MAIN_INIT()
{
	xEthQueue = xQueueCreateStatic(ETH_QUEUE_LENGTH,
			sizeof(EHT_MessageToQueue_t),
			ucEthQueueStorageArea,
			&xEthStaticQueue);
	configASSERT(xEthQueue);

	xEthSendQueue = xQueueCreateStatic(EthSend_QUEUE_LENGTH,
			sizeof(EHT_SendMsgToQueue_t),
			ucEthSendQueueStorageArea,
			&xEthSendStaticQueue);
	configASSERT(xEthSendQueue);

	xUartQueue = xQueueCreateStatic(UART_QUEUE_LENGTH,
			sizeof(Uart_MessageToQueue_t),
			ucUartQueueStorageArea,
			&xUartStaticQueue);
	configASSERT(xUartQueue);

	INIT_ALL_SEMAPHORES;
	INIT_ALL_TASKS;
	RTG_UartInit();

}
/*
 * Run RTG code from main.c
 * in defaultTask
 *
 */
void RTG_Entry_Point(void)
{
	RTG_UDP_INIT();
	RTG_DISCRETE_INT();
	RTG_SDRAM_int();
	RTG_ADC_int();
	RTG_IMU_ADIS16467_INIT();
	RTG_IMU_ADIS16547_INIT();
	RTG_Servo_init_msgs();			// SERVO INIT send UART

	CBIT_CURRENT_BUFF.header.type = *(uint16_t*)CBIT_MESSAGE_TYPE;
	CBIT_FULL_BUFF.hdr.length = CBIT_CURRENT_BUFF_SIZE;

	pPbitMes->header.type = *((uint16_t*) PBIT_MESSAGE_TYPE);
	PBIT_mes.hdr.length = PBIT_CURRENT_BUFF_SIZE;

#if BIT_SEND_TO_SNA == 1
	RTG_versions_tx(); 			// S&A PBIT TX COMANDS // SEND versions command
#endif
	RTG_IMU_ADIS16467_DIAG_STS();     //Start IMU ADIS16467 self test
	RTG_IMU_ADIS16547_DIAG_STS();     //Start IMU ADIS16547 self test
//*******  25mSec Delay for IMU to finish the self test ****************************************
	RT_DELAY_MS(50);				// delay for devices end work

	RTG_IMU_ADIS16547_PBIT(&pPbitMes->imu_adis16547);
	RTG_IMU_ADIS16467_PBIT(&pPbitMes->imu_adis16467);
	ADS_SET_READED;

	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

	CBIT_CURRENT_BUFF.status.INIT_COMPLETED = 1;
//>>>> INIT DEVICES END <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
	RTG_PBIT_FUNC();
}

uint16_t RTG_Copy_Data_to_Message(uint8_t *source, uint8_t *dist,
		uint16_t len, uint8_t count)
{
	CIB_Header *pDest = (CIB_Header*) dist;
	// copy header
	pDest->length = len;
	pDest->seq = count;
	// copy data
	memcpy( (dist + sizeof(CIB_Header)),source, len);
	return (len + sizeof(CIB_Header));
}

uint16_t RTG_CalculateChecksum(uint8_t *dataToCalculate, uint32_t size)
{
	uint16_t checksum = 0;
	for (uint32_t i = 0; i < (size - 2); i++) // (size - 2) remove 2 bytes of checksum
	{
		checksum += *(dataToCalculate + i);
	}
	return checksum;
}

// ********* old not use *****************************************************
uint16_t RTG_Prepare_pack_msg(uint8_t *source, uint16_t len, uint8_t count, uint8_t spare)
{
	CIB_Header *pDest = (CIB_Header*) source;
	int offset = sizeof(CIB_Header);
	int i;
// move message to package length
	for (i = len; i >= 0; i--)
	{
		*(source + i + offset) = *(source + i);
	}
// set header
	pDest->length = len;
	pDest->seq = count;
	return (len + offset);
}

//**************** end RTG main *************************************************
