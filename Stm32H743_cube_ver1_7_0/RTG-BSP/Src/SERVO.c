/*
 * SERVO.c
 *
 *  Created on: 7 Mar 2022
 *      Author: itzhaki
 */

#include <bit.h>
#include <cmsis_os2.h>
#include <string.h>
#include <SERVO.h>
#include <timer.h>
#include <uarts.h>


SERVO_UART_rx_t SERVO_UART_buff =
{ 0 };


static uint16_t RTG_Servo_CalculateChecksum(uint16_t *buf, uint16_t len)
{
	uint16_t checksum = 0;

	for (uint16_t i = 0; i < ((len / 2) - 1); i++)
	{
		checksum += buf [i];
	}

	return checksum;
}


/*****************************************************************************************************************************/
// GPS MESSAGE STRUCTURE
// ---------------------------------------------------------------------------------------------------------------------------
// | TIME TAG(32) |  BIT RESULTS(8) | STAUS2(16) | TEMPERATURE(16) | AXIS1 Status(16) | AXIS2 Status(16) | AXIS3 Status(16) | AXIS4 Status(16) |
// ---------------------------------------------------------------------------------------------------------------------------
// Pass Fail register: 1 - FAIL, 0 - PASS
//
//   7   6   5        4             3            2             1           0
// ------------------------------------------------------------------------------
// | 0 | 0 | 0 | axis4Status | axis3Status | axis2Status | axis1Status |status2 |
// ------------------------------------------------------------------------------
void RTG_Servo_BIT(SERVO_BIT_VALUE_t *mes)
{
	uint16_t syncroWord;
	uint16_t MessageID;
	uint16_t calc_checksum;

	// Clear structure
	memset((uint8_t*) mes, 0, sizeof(SERVO_BIT_VALUE_t));
	memcpy(&syncroWord, SERVO_UART_buff.buffer, 2);
	memcpy(&MessageID,  SERVO_UART_buff.buffer + SERVO_MESSAGE_ID_POS, 2);

	if (syncroWord == SERVO_SYNCRO_WORD)
	{
		if (MessageID == MESSAGE_ID)
		{
			memcpy(&calc_checksum,  SERVO_UART_buff.buffer + SERVO_CHECKSUM, 2);

			if (RTG_Servo_CalculateChecksum((uint16_t*)  SERVO_UART_buff.buffer,
			SERVO_UART_DATA_LEN) == calc_checksum)
			{
				memcpy(&mes->Status2,  SERVO_UART_buff.buffer + SERVO_STATUS2_POS, 2);
				memcpy(&mes->elecTemperature,
						 SERVO_UART_buff.buffer + SERVO_ETEMPERATURE_POS, 2);
				memcpy(&mes->axis1Status,  SERVO_UART_buff.buffer + SERVO_AXIS1_STATUS, 2);
				memcpy(&mes->axis2Status,  SERVO_UART_buff.buffer + SERVO_AXIS2_STATUS, 2);
				memcpy(&mes->axis3Status,  SERVO_UART_buff.buffer + SERVO_AXIS3_STATUS, 2);
				memcpy(&mes->axis4Status,  SERVO_UART_buff.buffer + SERVO_AXIS4_STATUS, 2);

				// Check status
				if (mes->Status2 & 0x0002) mes->bitResults.status2 = 1;
				if (mes->axis1Status & 0x803F) mes->bitResults.axis1Status = 1;
				if (mes->axis2Status & 0x803F) mes->bitResults.axis2Status = 1;
				if (mes->axis3Status & 0x803F) mes->bitResults.axis3Status = 1;
				if (mes->axis4Status & 0x803F) mes->bitResults.axis4Status = 1;

				if (mes->bitResults.results)
				{
					// keep it in the the current error register
//					ERROR_STATUS_Current.SERVO = 1;
//
//					//keep it in the error sticky register
//					if (!PBIT_IS_ENDED) ERROR_STATUS_STICKY.SERVO = 1;
				}
//				else ERROR_STATUS_Current.SERVO = 0;

				// Add TIME TAG
				mes->TimeTag = GetTimeTag;
			}
		}
	}
	else mes->TimeTag = GetTimeTag;
}

// ################################################################################
// init_servo_msgs - initialize all messages
// ################################################################################

void RTG_Servo_init_msgs(void)
{
	SERVO_command_message_struct command_msg =
	{ 0 };
	int16_t sync_word = SERVO_SYNCRO_WORD;
	int8_t message_counter = 0x00;
	int8_t message_counter_echo = 0x00;
	int16_t message_ID = 0x0001;
	int16_t command_word = MOTORS_ON_AND_PTP;
	float command_position_axis = 0;
	float command_speed_axis = 0;
	int16_t spare = 0x0000;
	int16_t time_tag = 0X00000000;

	// HEADER
	command_msg.header.sync_word = sync_word;
	command_msg.header.message_counter = message_counter;
	command_msg.header.message_counter_echo = message_counter_echo;
	command_msg.header.message_ID = message_ID;

	// DATA
	command_msg.command_word = command_word;
	command_msg.spare = spare;
	command_msg.time_tag = time_tag;
	command_msg.command_position_axis_1 = command_position_axis;
	command_msg.command_speed_axis_1 = command_speed_axis;
	command_msg.command_position_axis_2 = command_position_axis;
	command_msg.command_speed_axis_2 = command_speed_axis;
	command_msg.command_position_axis_3 = command_position_axis;
	command_msg.command_speed_axis_3 = command_speed_axis;
	command_msg.command_position_axis_4 = command_position_axis;
	command_msg.command_speed_axis_4 = command_speed_axis;

	command_msg.checksum = RTG_Servo_CalculateChecksum((uint16_t*) &command_msg,
			sizeof(command_msg));

	memcpy(sUart [UART_SERVO].TXbuf, (uint8_t*) &command_msg, sizeof(command_msg));
	sUart [UART_SERVO].dataTx_length = sizeof(command_msg);
	RTG_Uart_SendMessege(&sUart [UART_SERVO]);
}

