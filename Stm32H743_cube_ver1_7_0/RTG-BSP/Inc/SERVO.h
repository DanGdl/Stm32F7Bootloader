/*
 * SERVO.h
 *
 *  Created on: 7 Mar 2022
 *      Author: itzhaki
 */

#ifndef SERVO_H_
#define SERVO_H_

#include <RTG_main.h>
#include <sys/_stdint.h>

#define SERVO_SYNCRO_WORD 	0xAC13
#define MOTORS_ON_AND_PTP	0b0000000000000111
#define MESSAGE_ID 17

#define SERVO_SYNCRO_WORD_POS 			0
#define SERVO_MESSAGE_COUNTER_POS 		(SERVO_SYNCRO_WORD_POS + 2)
#define SERVO_MESSAGE_COUNTER_ECHO_POS 	(SERVO_MESSAGE_COUNTER_POS + 1)
#define SERVO_MESSAGE_ID_POS 			(SERVO_MESSAGE_COUNTER_ECHO_POS + 1)
#define SERVO_STATUS2_POS 			    (SERVO_MESSAGE_ID_POS + 2)
#define SERVO_ETEMPERATURE_POS 			(SERVO_STATUS2_POS + 2)
#define SERVO_AXIS1						(SERVO_ETEMPERATURE_POS + 2)
#define SERVO_AXIS1_STATUS				(SERVO_AXIS1 + 16)
#define SERVO_AXIS2						(SERVO_AXIS1 + 20)
#define SERVO_AXIS2_STATUS				(SERVO_AXIS2 + 16)
#define SERVO_AXIS3						(SERVO_AXIS2 + 20)
#define SERVO_AXIS3_STATUS				(SERVO_AXIS3 + 16)
#define SERVO_AXIS4						(SERVO_AXIS3 + 20)
#define SERVO_AXIS4_STATUS				(SERVO_AXIS4 + 16)
#define SERVO_CHECKSUM					(SERVO_AXIS4 + 20)

#pragma pack (push,1)

typedef union
{
	uint8_t results;
	struct
	{
		uint8_t status2 :1;
		uint8_t axis1Status :1;
		uint8_t axis2Status :1;
		uint8_t axis3Status :1;
		uint8_t axis4Status :1;
	};
} SERVO_BIT_RESULTS;

typedef struct
{
	uint32_t TimeTag;
	SERVO_BIT_RESULTS bitResults;
	uint16_t Status2;
	uint16_t elecTemperature;
	uint16_t axis1Status;
	uint16_t axis2Status;
	uint16_t axis3Status;
	uint16_t axis4Status;
} SERVO_BIT_VALUE_t;

/***************************************************************/
typedef struct
{
	uint16_t sync_word;
	uint8_t message_counter;
	uint8_t message_counter_echo;
	uint16_t message_ID;
} SERVO_header_struct;

typedef struct
{
	SERVO_header_struct header;
	uint16_t command_word;
	uint16_t spare;
	int32_t time_tag;
	float command_position_axis_1;
	float command_speed_axis_1;
	float command_position_axis_2;
	float command_speed_axis_2;
	float command_position_axis_3;
	float command_speed_axis_3;
	float command_position_axis_4;
	float command_speed_axis_4;
	uint16_t checksum;
} SERVO_command_message_struct;

/***************************************************************/
#define SERVO_UART_DATA_LEN 92

typedef struct
{
	CIB_Header header;
	uint8_t buffer[SERVO_UART_DATA_LEN + 20];
} SERVO_UART_rx_t;

/***************************************************************/

#pragma pack (pop)
extern SERVO_UART_rx_t SERVO_UART_buff;

void RTG_Servo_BIT(SERVO_BIT_VALUE_t *mes);
void RTG_Servo_init_msgs(void);

#endif /* SERVO_H_ */
