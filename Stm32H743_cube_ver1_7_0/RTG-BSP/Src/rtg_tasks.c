/*
 * rtg_tasks.c
 *
 *  Created on: 10 בדצמ׳ 2022
 *      Author: e_tas
 */
#include <string.h>
#include <stdlib.h>

#include <rtg_tasks.h>
#include <RTG_main.h>
#include <uarts.h>
#include <dlu.h>
#include <spi.h>
#include <bit.h>
#include <timer.h>
#include <IMU-TRF-90M.h>
#include <discrete.h>
#include <ADIS16547.h>
#include <Safe_And_Arm.h>
#include <SERVO.h>

INIT_ALL_TASKS_VAR
INIT_ALL_SEMAPHORE_VARS

uint8_t udpServoGetMessage;
uint8_t uartServoGetMessage;
/* udp callbak message receive port to task */
uint16_t udpCallbackRxPort;
/* udp callbak message receive port to task MAC*/
uint16_t UartCallbackNumber;

T_FUN_HEADER(onDataReady) {
	while (1) {
		if (!SEMAPHORE_TAKE(onDataReady)) continue;
		RTG_SPI_TransmitReceive_DMA(&sSpi3, IMU_ADIS16547_CMD_Buffer, sSpi3.RxBuff, IMU_ADIS16547_CMD_Buffer_size);
		//TODO:2 send message to UART SERVO
		if (udpServoGetMessage) {
			RTG_Uart_SendMessege(&sUart [UART_SERVO]);
			udpServoGetMessage=0;
		}     // if (udpServoGetMessage)
	}     // while
}

T_FUN_HEADER( onSpiTxRx ) {
	static uint8_t count = 0;
	static CIB_400_Hz_Msg CIB_400_Hz_Msg_send;
	while (1) {
		if (!SEMAPHORE_TAKE(onSpiTxRx)) continue;

		RTG_IMU_ADIS16547_Gyro_Accalrtion();
		CIB_400_Hz.data.discretes.out = RTG_readOutputDiscrete();
		CIB_400_Hz.data.discretes.in =  RTG_readInputDiscrete();
		CIB_400_Hz.hdr.seq++;
		CIB_400_Hz_Msg_send = CIB_400_Hz;
		RTG_Udp_SendTo(SPI3_udp, (char*) &CIB_400_Hz_Msg_send, sizeof(CIB_400_Hz_Msg_send));
		sSpi5.Rx_ready = 0;
		RTG_SPI_TransmitReceive_DMA(&sSpi5, IMU_ADIS16467_CMD_Buffer, sSpi5.RxBuff, IMU_ADIS16467_CMD_Buffer_size);
		for(int i=0; i<4 && !sSpi5.Rx_ready;i++)
			RT_DELAY_TIC(1);
		if(sSpi5.Rx_ready)
			RTG_IMU_ADIS16467_Gyro_Accalrtion();
		// run CBIT function
		count = (count + 1) % 40;
		if (count) continue;
		SEMAPHORE_GIVE(runCbit);
	}     // while
}

T_FUN_HEADER( runCbit ) {
	while (1) {
		if (!SEMAPHORE_TAKE(runCbit)) continue;
//		if (! PBIT_IS_ENDED) continue;
		RTG_CBIT_FUNC();
	}     // while
}

T_FUN_HEADER( onUdpCallback ) {
	static EHT_MessageToQueue_t MessageHeader;
	while (1) {
		memset(&MessageHeader, 0, sizeof(MessageHeader));
		if (xQueueReceive(xEthQueue, &(MessageHeader), portMAX_DELAY) != pdPASS) continue;
		switch (MessageHeader.port) {
			case CLONE_UDP_PORT:
				CBIT_CURRENT_BUFF.clone = *(CIB_Clone_Data*) (MessageHeader.buffer/* + sizeof(CIB_Header)*/);
			break;
			case UDP_DISCRETE_PORT_IN:
				RTG_UDP_Discrete((uint8_t*) MessageHeader.buffer);
			break;

			case SERVO_UDP_PORT:     // SERVO send to UART
				sUart [UART_SERVO].dataTx_length = *(uint16_t*) (MessageHeader.buffer);
//				if (sUart [UART_SERVO].dataTx_length < SERVO_UART_DATA_LEN)
//				{
//					sUart [UART_SERVO].ErrorRxLength++;
//				}
//				else
				{
					memcpy(sUart [UART_SERVO].TXbuf,
							((uint8_t*) MessageHeader.buffer) + sizeof(CIB_Header),
							sUart [UART_SERVO].dataTx_length);
					udpServoGetMessage = 1;
				}
				break;

			case SNA_UDP_PORT:     // SAA
				sUart [UART_SAA].dataTx_length = *(uint16_t*) (MessageHeader.buffer);
				memcpy(sUart [UART_SAA].TXbuf,
						((uint8_t*) MessageHeader.buffer) + sizeof(CIB_Header),
						sUart [UART_SAA].dataTx_length);
				RTG_Uart_SendMessege(&sUart [UART_SAA]);
			break;

			case MGLT_UDP_PORT:     // MRGALIT
				sUart [UART_MARGALIT].dataTx_length = *(uint16_t*) (MessageHeader.buffer);
				sUart [UART_MARGALIT].recieveCount++;

				memcpy(sUart [UART_MARGALIT].TXbuf, ((uint8_t*) MessageHeader.buffer) + sizeof(CIB_Header),
						sUart [UART_MARGALIT].dataTx_length);
				RTG_Uart_SendMessege(&sUart [UART_MARGALIT]);
			break;

			case MAANASH_UDP_PORT:     // UART_MAANASH

				sUart [UART_MAANASH].dataTx_length = *(uint16_t*) (MessageHeader.buffer);
				memcpy(sUart [UART_MAANASH].TXbuf, ((uint8_t*) MessageHeader.buffer) + sizeof(CIB_Header),
						sUart [UART_MAANASH].dataTx_length);
				taskENTER_CRITICAL();
				RTG_Uart_SendMessege(&sUart [UART_MAANASH]);
				taskEXIT_CRITICAL();
			break;

			case GPS_UDP_PORT:     // GPS
				sUart [UART_GPS].dataTx_length = *(uint16_t*) (MessageHeader.buffer);
				memcpy(sUart [UART_GPS].TXbuf, ((uint8_t*) MessageHeader.buffer) + sizeof(CIB_Header),
						sUart [UART_GPS].dataTx_length);
				RTG_Uart_SendMessege(&sUart [UART_GPS]);
			break;

//			case UDP_DLU_PORT_IN:
//				if (DLU_GET_opcode( MessageHeader.buffer ) == ADT_PBIT_REPORT) {
//				}
//				else if (DLU_GET_opcode( MessageHeader.buffer ) == ADT_CBIT_REPORT) {
//				}
//				DLU_ADD_COUNT;
//			break;

			default:
			break;
		}     // switch
	}
}

T_FUN_HEADER( onUARTxRxCallback ) {
	static uint32_t 			send_length;
	static uint32_t 			nUART;
	static uint32_t 			gps_id,gps_messagelen,timetag;
	static OEM7_Version_t 		*pGpsVer;
	static OEM7_RXSTATUS_t 		*pGpsRxStat;
	static CIB_CBIT_GPS        	tempGps;
	static uint8_t 		udpBuf[1000];
	static Uart_MessageToQueue_t Message;
	while (1) {
		if (xQueueReceive(xUartQueue, &(Message), portMAX_DELAY) != pdPASS) continue;
		if (Message.size >= UART_BUF_SIZE) continue;

		nUART = Message.uart_n;
		switch (nUART) {
			case UART_SERVO:
				memcpy(SERVO_UART_buff.buffer, sUart [UART_SERVO].RXbuf,SERVO_UART_DATA_LEN);
				SERVO_UART_buff.header.length = SERVO_UART_DATA_LEN;
				SRV_ADD_COUNT;
				sUart [UART_SERVO].recieveCount++;
				uartServoGetMessage=1;
				SERVO_UART_buff.header.seq  ++;
				RTG_Udp_SendTo(*sUart [UART_SERVO].UART_udp, (char*) &SERVO_UART_buff,
						SERVO_UART_buff .header.length + 4);
				SRV_SET_READED;

			break;
			case UART_SAA:     //U1
			{
				// Check if it is DETAILED_STATUS_MSG
				if (sUart [UART_SAA].RXbuf [7] == SnA_DETAILED_STATUS_MSG) {
//					SAA_ADD_COUNT;
				}
				// Check if it is VERSIONS_MSG
				else if (sUart [UART_SAA].RXbuf [7] == SnA_VERSIONS_MSG) {
					SAA_ADD_COUNT;
				}
				// Check if it is PROTOCOL_ERROR_MSG
				else if (sUart [UART_SAA].RXbuf [7] == SnA_PROTOCOL_ERROR_MSG) {
					memcpy((uint8_t*) &SSA_errorMessage, sUart [UART_SAA].RXbuf, sizeof(SSA_errorMessage));
				}
				send_length = RTG_Copy_Data_to_Message(sUart [UART_SAA].RXbuf,udpBuf, sUart [UART_SAA].dataRx_length,
						sUart [UART_SAA].UART_udp->count++);
				RTG_Udp_SendTo(*sUart [UART_SAA].UART_udp, (char*) udpBuf, send_length);
				SAA_SET_READED;
			}
			break;

			case UART_MARGALIT:     //U6
			{
				RTG_MargalitTo_400Hz(*((IMU_MGLT_Msg*) sUart [UART_MARGALIT].RXbuf),&CIB_400_Hz.data.margalit);
				if ( !MGL_IS_READED && !MGL_R_IS_FULL )     // if PBIT
				{
					RTG_Update_Margalit_PBIT((IMU_MGLT_Msg*) sUart [UART_MARGALIT].RXbuf,&pPbitMes->margalit);
				}

				RTG_Update_Margalit_CBIT((IMU_MGLT_Msg*) sUart [UART_MARGALIT].RXbuf, &CBIT_CURRENT_BUFF.margalit );
			}
			break;

			case UART_GPS:     //U4
			{
				timetag = GetTimeTag;
				if(!GPS_IS_READED)					GPS_SendVersion();
				for(uint32_t i=0;i< sUart [UART_GPS].dataRx_length;i++){
					if(!IS_OEM7_HEADER( &sUart [UART_GPS].RXbuf[i]) ) continue;

					gps_id = OEM7_MESSAGE_ID( &sUart [UART_GPS].RXbuf[i] );
					gps_messagelen =OEM7_MESSAGE_LENGTH(&sUart [UART_GPS].RXbuf[i]);
					switch(gps_id){
					case OEM7_OPCODE_BESTXYZ:
						memcpy(udpBuf,&sUart [UART_GPS].RXbuf[i],gps_messagelen+sizeof(OEM7_Hader_t)+4);
						send_length =
							RTG_Copy_Data_to_Message(&sUart [UART_GPS].RXbuf[i],udpBuf,
							gps_messagelen+sizeof(OEM7_Hader_t)+4,sUart [UART_GPS].UART_udp->count++);
						RTG_Udp_SendTo(*sUart [UART_GPS].UART_udp,(char*) udpBuf, send_length);
						GPS_SET_READED;
					break;
					case OEM7_OPCODE_VERSION:
						if( GPS_IS_READ_V ) break;
						pGpsVer =(OEM7_Version_t*) &sUart [UART_GPS].RXbuf[i];
						for(uint32_t j=0;j<pGpsVer->Numberofcomponents;j++){
							if( pGpsVer->comp[j].type == 1 ){
								memcpy(&pPbitMes->gps, &pGpsVer->comp[j],sizeof(CIB_PBIT_GPS) );
								GPS_SET_READ_V;
								GPS_SET_READED;
								break;
							}
						}
						break;
					case OEM7_OPCODE_RXSTATUS:
						pGpsRxStat =(OEM7_RXSTATUS_t*) &sUart [UART_GPS].RXbuf[i];
						tempGps.TimeStatus = pGpsRxStat->header.TimeStatus;
						tempGps.Week = pGpsRxStat->header.Week;
						tempGps.ms = pGpsRxStat->header.ms;

						tempGps.error.word = pGpsRxStat->error;
						tempGps.rxstat.word = pGpsRxStat->rxstat;
						tempGps.aux1stat.word = pGpsRxStat->aux1stat;
						tempGps.aux2stat.word = pGpsRxStat->aux2stat;
						tempGps.aux3stat.word = pGpsRxStat->aux3stat;
						tempGps.aux4stat.word = pGpsRxStat->aux4stat;

						tempGps.TimeTag = timetag;//GetTimeTag;
						taskENTER_CRITICAL();
						CBIT_CURRENT_BUFF.gps = tempGps;
						taskEXIT_CRITICAL();
						GPS_SET_READED;
						break;
					default:
						break;
					}
					i+=gps_messagelen-1;
				}// for
			}
			break;
			case UART_MAANASH:     //U3
			send_length = RTG_Copy_Data_to_Message(sUart [UART_MAANASH].RXbuf,udpBuf,
					sUart [UART_MAANASH].dataRx_length,sUart [UART_MAANASH].UART_udp->count++ );
			RTG_Udp_SendTo(*sUart [UART_MAANASH].UART_udp, (char*) udpBuf, send_length);
			break;
		}     //switch
		sUart [nUART].recieveCount++;
	}     // while

}

extern ETH_HandleTypeDef heth;
extern u32_t MU_addr; /* for save address send udp message  */

T_FUN_HEADER( onEthSend ) {
	EHT_SendMsgToQueue_t Message={0};
	while (1) {
		if (xQueueReceive(xEthSendQueue, &(Message), portMAX_DELAY) != pdPASS) continue;
		if (Message.udp_buffer != NULL){
			if (heth.gState != HAL_ETH_STATE_READY
					&& heth.gState != HAL_ETH_STATE_BUSY ) {
				heth.gState = HAL_ETH_STATE_READY;
			}
			if( udp_sendto(Message.upcb, Message.udp_buffer,
					(ip_addr_t*) &Message.addres, Message.port)  != ERR_OK ){
			}
			pbuf_free(Message.udp_buffer);
		}
	}
}

/*============ END =====================================================*/

