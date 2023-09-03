/*
 * discrete.c
 *
 *  Created on: Nov 23, 2021
 *      Author: itzhaki
 */

#include <string.h>
#include <RTG_main.h>
#include <bit.h>
#include <rtg_tasks.h>
#include <timer.h>
#include <uarts.h>
#include <spi.h>
#include <discrete.h>

extern UDP_t DISCRETE_udp;

extern TIMSt_t OnePPC_Time;
extern CIB_400_Hz_Msg CIB_400_Hz;
extern TIM_HandleTypeDef htim4;


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	static portBASE_TYPE xHigherPriorityTaskWoken;

	if (GPIO_Pin == GPIO_PIN_0)     // !PPS Interupt PA0
	{
		OnePPC_Time.RtcPrev = OnePPC_Time.RtcLast;
		OnePPC_Time.RtcLast = OnePPC_Time.ticks25Mks;
		OnePPC_Time.RTCdelta = OnePPC_Time.RtcLast - OnePPC_Time.RtcPrev;
		if(
				abs((int)(OnePPC_Time.RTCdelta - 40000))>0
				&&
				(OnePPC_Time.RtcPrev)>30000
				)
			htim4.Instance->ARR += (int)(OnePPC_Time.RTCdelta - 40000)/7;
	}

	if (GPIO_Pin == GPIO_PIN_14)     //PB14 -- IMU DataReady
	{
		CIB_400_Hz.data.Data_Ready_Time = GetTimeTag;
		YIELD_SEMAPHORE_GIVE_ISR(onDataReady, xHigherPriorityTaskWoken);
	}
}

/**
  * @brief This function handles EXTI line[15:10] interrupts.
  */
void RTG_DISCRETE_INT(void) {
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_SET);
}

void RTG_UDP_Discrete(uint8_t *receive) {
	static DicreteToUdp_t DiscToUdp;
	static uint16_t tmp=0;
	CIB_Cmd_Data *pCIB_Cmd_Data = (CIB_Cmd_Data*)(receive + sizeof(CIB_Header));
	CIB_Header *pCIB_Header = (CIB_Header*)receive;
	uint32_t second,min,hour;
	uint8_t chSum[4]={0};
	char bufZDA[100]={0};
	char bufGGA[100]={0};

	switch (pCIB_Cmd_Data->cmd) {
		case CIB_TOD:
			if( pCIB_Header->length < sizeof( CIB_Cmd_Data) ) break;
			updateTod( pCIB_Cmd_Data->Sec );
			RTG_Udp_SendToClone((char*)receive,pCIB_Header->length+sizeof(CIB_Header));
			second = TICKS25MKS_TO_D_SEC(pCIB_Cmd_Data->Sec);
			min = second/60;
			second = second - min*60 ;
			hour = min/60;
			min = min -hour*60;
			sprintf((char*)bufZDA,
					"$GPZDA,%02lu%02lu%02lu.00,%02lu,%02lu,%04lu,," //GP_LOCAL_TZ GP_LOCAL_TZ_M,
					,hour+tmp,min,second,
					pCIB_Cmd_Data->Day,
					pCIB_Cmd_Data->Mon,
					pCIB_Cmd_Data->Year
					);
			sprintf((char*)bufGGA,
					GP_GGA,
					hour+tmp,min,second
					);
			chSum[B_ZDA] = CIB2SEEKER_CheckSum(bufZDA,strlen((char*)bufZDA));
			chSum[B_GGA] = CIB2SEEKER_CheckSum(bufGGA,strlen(bufGGA));
			chSum[B_GSV] = CIB2SEEKER_CheckSum(GP_GSV,strlen(GP_GSV));
			chSum[B_GSA] = CIB2SEEKER_CheckSum(GP_GSA,strlen(GP_GSA));
			sUart[UART_CIB2SEEKER].dataTx_length =
					sprintf((char*)sUart[UART_CIB2SEEKER].TXbuf,
							"%s*%02x\r\n%s*%02x\r\n%s*%02x\r\n%s*%02x\r\n" ,
					(char*)bufGGA, chSum[B_GGA],
					(char*)GP_GSA, chSum[B_GSA],
					(char*)GP_GSV, chSum[B_GSV],
					(char*)bufZDA, chSum[B_ZDA]
										  );
			RTG_Uart_SendMessege(&sUart[UART_CIB2SEEKER]);
		break;
		case CIB_PRIME_MARGALIT:	// CIB_IMU_PRIME_MARGALIT:
			FpgaSetSysSync(0);
		break;
		case CIB_PRIME_ADIS: 	//CIB_IMU_PRIME_ADIS:
			FpgaSetSysSync(1);
		break;
		case CIB_OUTPUT_DISCRETE_CLEAR:
			RTG_writeOutputDiscrete(pCIB_Cmd_Data->discrete, 0);
		break;

		case CIB_OUTPUT_DISCRETE_ACTIVE:
			RTG_writeOutputDiscrete(pCIB_Cmd_Data->discrete, 1);
		break;

		case CIB_OUTPUT_DISCRETES_READ:
			DiscToUdp.opcode = pCIB_Cmd_Data->cmd;
			DiscToUdp.data = RTG_readOutputDiscrete();
			DiscToUdp.header.length = 3;
			DiscToUdp.header.seq ++;
			RTG_Udp_SendTo(DISCRETE_udp, (char*) &DiscToUdp, sizeof(DicreteToUdp_t));
		break;

		case CIB_INPUT_DISCRETES_READ:
			DiscToUdp.opcode = pCIB_Cmd_Data->cmd;
			DiscToUdp.data = RTG_readInputDiscrete();
			DiscToUdp.header.length = 3;
			DiscToUdp.header.seq ++;
			RTG_Udp_SendTo(DISCRETE_udp, (char*) &DiscToUdp, sizeof(DicreteToUdp_t));
		break;

		case CIB_PBIT_REQUEST:     // Send PBIT
			if(!PBIT_IS_ENDED) break;
			PBIT_mes_st.hdr.seq++;
			RTG_Udp_Send((char*) &PBIT_mes, PBIT_FULL_BUFF_SIZE);
		break;
	}
}

void RTG_writeOutputDiscrete(uint8_t bit, uint16_t value) {
	switch (bit) {
		case DISC_OUT_TVC_DSCNNCTD:
			HAL_GPIO_WritePin( GPIO_OUT_TVC_DSCNNCTD, value);
		break;
		case DISC_OUT_FUSE_1_IGNITION:
			HAL_GPIO_WritePin( GPIO_OUT_FUSE_1_IGNITION, value);
		break;
		case DISC_OUT_FUSE_2_IGNITION:
			HAL_GPIO_WritePin( GPIO_OUT_FUSE_2_IGNITION, value);
		break;
		case DISC_OUT_FUSE_3_IGNITION:
			HAL_GPIO_WritePin( GPIO_OUT_FUSE_3_IGNITION, value);
		break;
		case DISC_OUT_RST_IMU_1_OUTPUT:
			HAL_GPIO_WritePin( GPIO_OUT_RST_IMU_1_OUTPUT, value);
		break;
		case DISC_OUT_RST_IMU_2_OUTPUT:
			HAL_GPIO_WritePin( GPIO_OUT_RST_IMU_2_OUTPUT, value);
		break;
		case DISC_OUT_Ext_Com_Dis:
			HAL_GPIO_WritePin( GPIO_OUT_Ext_Com_Dis, value);
		break;
		case DISC_OUT_FTS2_Batt_Ign:
			HAL_GPIO_WritePin( GPIO_OUT_FTS2_Batt_Ign, value);
		break;
		case DISC_OUT_WCS_END_COM:
			HAL_GPIO_WritePin( GPIO_OUT_WCS_END_COM, value);
		break;
	}
}

uint16_t RTG_readOutputDiscrete(void) {
	uint16_t outDis = 0;

	SET_VALUE(outDis, DISC_OUT_TVC_DSCNNCTD, 		HAL_GPIO_ReadPin( GPIO_OUT_TVC_DSCNNCTD ));
	SET_VALUE(outDis, DISC_OUT_FUSE_1_IGNITION, 	HAL_GPIO_ReadPin( GPIO_OUT_FUSE_1_IGNITION ));
	SET_VALUE(outDis, DISC_OUT_FUSE_2_IGNITION, 	HAL_GPIO_ReadPin( GPIO_OUT_FUSE_2_IGNITION ));
	SET_VALUE(outDis, DISC_OUT_FUSE_3_IGNITION, 	HAL_GPIO_ReadPin( GPIO_OUT_FUSE_3_IGNITION ));
	SET_VALUE(outDis, DISC_OUT_RST_IMU_1_OUTPUT, 	HAL_GPIO_ReadPin( GPIO_OUT_RST_IMU_1_OUTPUT ));
	SET_VALUE(outDis, DISC_OUT_RST_IMU_2_OUTPUT, 	HAL_GPIO_ReadPin( GPIO_OUT_RST_IMU_2_OUTPUT ));
	SET_VALUE(outDis, DISC_OUT_FTS2_Batt_Ign, 		HAL_GPIO_ReadPin( GPIO_OUT_FTS2_Batt_Ign ));
	SET_VALUE(outDis, DISC_OUT_Ext_Com_Dis, 		HAL_GPIO_ReadPin( GPIO_OUT_Ext_Com_Dis ));
	SET_VALUE(outDis, DISC_OUT_WCS_END_COM, 		HAL_GPIO_ReadPin( GPIO_OUT_WCS_END_COM ));

	return outDis;
}

uint16_t RTG_readInputDiscrete(void) {
	uint16_t inDis = 0;

	SET_VALUE(inDis, DISC_IN_SQUIB_BAT_FTS, 	! 	HAL_GPIO_ReadPin( GPIO_IN_SQUIB_BAT_FTS ));
	SET_VALUE(inDis, DISC_IN_PTL_SMPL, 			! 	HAL_GPIO_ReadPin( GPIO_IN_PTL_SMPL ));
	SET_VALUE(inDis, DISC_IN_MPE_2_IND, 		! 	HAL_GPIO_ReadPin( GPIO_IN_MPE_2_IND ));
	SET_VALUE(inDis, DISC_IN_SnA_ENA_SMPL, 		! 	HAL_GPIO_ReadPin( GPIO_IN_SnA_ENA_SMPL ));
	SET_VALUE(inDis, DISC_IN_MLL_IND_INPUT, 		HAL_GPIO_ReadPin( GPIO_IN_MLL_IND_INPUT ));
	SET_VALUE(inDis, DISC_IN_PPS_FPGA_2_CPU, 	! 	HAL_GPIO_ReadPin( GPIO_IN_PPS_FPGA_2_CPU ));
	SET_VALUE(inDis, DISC_IN_CPU2FPGA_GPIO2, 	! 	HAL_GPIO_ReadPin( GPIO_IN_CPU2FPGA_GPIO2 ));
	SET_VALUE(inDis, DISC_IN_CPU2FPGA_GPIO3, 	! 	HAL_GPIO_ReadPin( GPIO_IN_CPU2FPGA_GPIO3 ));
	SET_VALUE(inDis, DISC_IN_CPU2FPGA_GPIO4, 	! 	HAL_GPIO_ReadPin( GPIO_IN_CPU2FPGA_GPIO4 ));
	SET_VALUE(inDis, DISC_IN_CPU2FPGA_GPIO1, 	! 	HAL_GPIO_ReadPin( GPIO_IN_CPU2FPGA_GPIO1 ));
	SET_VALUE(inDis, DISC_IN_DIO1_IMU2_SYNC, 	! 	HAL_GPIO_ReadPin( GPIO_IN_DIO1_IMU2_SYNC ));
	SET_VALUE(inDis, DISC_IN_DIO2_IMU2_DTRDY, 	! 	HAL_GPIO_ReadPin( GPIO_IN_DIO2_IMU2_DTRDY ));
	SET_VALUE(inDis, DISC_IN_DIO2_IMU1_DTRDY, 	! 	HAL_GPIO_ReadPin( GPIO_IN_DIO2_IMU1_DTRDY ));
	SET_VALUE(inDis, DISC_IN_DIO1_IMU1_SYNC, 	! 	HAL_GPIO_ReadPin( GPIO_IN_DIO1_IMU1_SYNC ));

	return inDis;
}

uint8_t RTG_Fpga_Rev(void) {
	union FpgaVer FpgaV = { 0 };

	FpgaV.disceretReg = 0;
	FpgaV.pin_E_5 = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_5);
	FpgaV.pin_E_6 = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_6);
	FpgaV.pin_I_12 = HAL_GPIO_ReadPin(GPIOI, GPIO_PIN_12);
	return FpgaV.disceretReg;
}

/****************************************************************/
// This test check the DISCRETE input and output
//
// DISCRETE MESSAGE STRUCTURE
// ---------------------------------------------------------------------------------------
// | Time Tag(32) | Pass Fail(16) | DISCRETE INPUT VALUE(16) | DISCRETE OUTPUT VALUE(16) |
// ---------------------------------------------------------------------------------------
//
// Pass Fail register: 0 - PASS, 0 - PASS
//   7   6   5   4   3   2   1       0
// ---------------------------------------------------------------
// |  0 | 0 | 0 | 0 | 0 | 0 | INPUT Pass/Fail | OUTPUT Pass/Fail |
// ---------------------------------------------------------------
void RTG_Discrete_bit(BIT_Discrete_t *mes) {
	uint16_t val;
	uint8_t passFail = 0;

	//Read output discrete
	val = RTG_readOutputDiscrete() & 0x7FF;
	//write to BIT message buffer
	mes->OUT_Disc_Size = val;

	if (val != DISCRETR_COMPEAR_OUTPUT) {
		passFail = 1;
		// keep it in the the current error register
	}
	//Read input discrete
	val = RTG_readInputDiscrete() & 0x7FFF;
	//write to BIT message buffer
	mes->IN_Disc_Size = val;

	if (val != DISCRETR_COMPEAR_INPUT) {
		passFail += 2;
		// keep it in the the current error register

	}
	mes->Error = passFail;
	// Add TIME TAG
	mes->TimeTag = GetTimeTag;
	return;
}

void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */

  /* USER CODE END EXTI15_10_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_14);
  /* USER CODE BEGIN EXTI15_10_IRQn 1 */

  /* USER CODE END EXTI15_10_IRQn 1 */
}


