#ifndef DLU_H
#define DLU_H

#include <sys/_stdint.h>

//######################################
//############## CONSTATNS #############
//######################################

// OPCODES
#define  ADT_CBIT_REPORT  				0x85
#define  ADT_PBIT_REPORT 				0x86

//######################################
//############## headers ###############
//######################################
#pragma pack (push,1)

// ADT message header
typedef struct
{
	uint32_t time_stamp;
	uint16_t seq_num;
	uint8_t opcode;
} ADT_message_header_struct;

//######################################
// CBIT report OPCODE 0X85
//######################################

typedef struct
{
	ADT_message_header_struct header;

	uint8_t reserved :1;
	uint8_t ADT_SP4T_failure :1;
	uint8_t internal_DC_voltage_failure :1;
	uint8_t digital_section_failure :1;
	uint8_t RF_failure :1;
	uint8_t synthesizers_lock_detect :1;
	uint8_t pedestal_failure :1;
	uint8_t reserved_1 :1;

	uint8_t PA_over_current_failure :1;
	uint8_t Rx_input_over_power :1;
	uint8_t digital_section_over_heat :1;
	uint8_t RF_section_over_heat :1;
	uint8_t system_startUp :1;
	uint8_t reserved_2 :1;
	uint8_t PA_power_output_low :1;
	uint8_t PA_return_power_failure :1;

	uint16_t PA_power_output;
	uint16_t PA_return_power;

//	/************** itzhaki ***************/
//	uint8_t not_use;
//
//	uint8_t reserved_3				 	: 1;
//	uint8_t Sat_smaler_then_2 			: 1;
//	uint8_t TWO_D 						: 1;
//	uint8_t Three_D 					: 1;
//	uint8_t TWO_D_diff	 				: 1;
//	uint8_t reserved_4 					: 1;
//	uint8_t reserved_5			 		: 1;
//	uint8_t reserved_6				 	: 1;
//
//	uint16_t reserved_7;
//	/************** itzhaki ***************/
} ADT_CBIT_receive_message;

//######################################
// PBIT report OPCODE 0X86
//######################################

typedef struct
{
	ADT_message_header_struct header;

	uint8_t ADT_Flash_memories_Failure :1; //0 OK Parameters 1 Failure
	uint8_t ADT_DRAMs_Failure :1; //0 OK Parameters 1 Failure
	uint8_t ADT_DSP_Bit_Failure :1; //0 OK Parameters 1 Failure
	uint8_t reserved :5;

	uint8_t PBIT_Done :1; //0 PBIT Didn't done yet1, PBIT Done
	uint8_t FW_Main_version_Fail :1; //0 OK Parameters 1 Failure
	uint8_t DSP_SW_Main_version_Fail :1; //0 OK Parameters 1 Failure
	uint8_t DSP_SW_images_Fail :1; //0 OK Parameters 1 Failure
	uint8_t reserved_1 :4;

} ADT_PBIT_receive_message;

//######################################
// structs for send PBIT CBIT messages
//######################################

//########## CBIT ######################

typedef union
{
	uint16_t error16;
	struct
	{
		uint8_t ADT_SP4T_failure :1;
		uint8_t internal_DC_voltage_failure :1;
		uint8_t digital_section_failure :1;
		uint8_t RF_failure :1;
		uint8_t synthesizers_lock_detect :1;
		uint8_t pedestal_failure :1;
		uint8_t PA_over_current_failure :1;
		uint8_t Rx_input_over_power :1;
		uint8_t digital_section_over_heat :1;
		uint8_t RF_section_over_heat :1;
		uint8_t system_startUp :1;
		uint8_t PA_power_output_low :1;
		uint8_t PA_return_power_failure :1;
//		//itzhaki
//		uint8_t Sat_smaler_then_2 			: 1;
//		uint8_t TWO_D 						: 1;
//		uint8_t Three_D 					: 1;
//		uint8_t TWO_D_diff	 				: 1;
	} error;
} DLU_CBIT_errors_t;

typedef struct
{
	uint32_t TimeTag;
	DLU_CBIT_errors_t PassFaild;
	uint16_t PA_power_output;
	uint16_t PA_return_power;
} ADT_CBIT_send_message;

//########## PBIT ######################

typedef union
{
	uint8_t error8;
	struct
	{
		uint8_t ADT_Flash_memories_Failure :1; //0 OK Parameters 1 Failure
		uint8_t ADT_DRAMs_Failure :1; //0 OK Parameters 1 Failure
		uint8_t ADT_DSP_Bit_Failure :1; //0 OK Parameters 1 Failure
//==== is change >> 1 PBIT Didn't done , yet 0 PBIT Done
		uint8_t PBIT_Done :1; //0 OK Parameters 1 Failure
		uint8_t FW_Main_version_Fail :1; //0 OK Parameters 1 Failure
		uint8_t DSP_SW_Main_version_Fail :1; //0 OK Parameters 1 Failure
		uint8_t DSP_SW_images_Fail :1; //0 OK Parameters 1 Failure
	} error;
} DLU_PBIT_errors_t;

typedef struct
{
	uint32_t TimeTag;
	DLU_PBIT_errors_t PassFaild;
} ADT_PBIT_send_message;

#pragma pack (pop)

extern ADT_CBIT_receive_message ADT_CBIT_rec_message;
extern ADT_PBIT_receive_message ADT_PBIT_rec_message;
// for DLU
#define BIT_DLU_CP_RX_TO_PBIT(src)  \
	DLU_pbit_buld_struct((ADT_PBIT_receive_message*) src , &PBIT_mes_st.DLU )


#define BIT_DLU_CP_RX_TO_CBIT(src)  \
	DLU_cbit_buld_struct((ADT_CBIT_receive_message*) src , &CBIT_CURRENT_BUFF_R.DLU )


#define DLU_CP_RX_TO_PBIT(dist) \
	DLU_pbit_buld_struct( &ADT_PBIT_rec_message, (ADT_PBIT_send_message*)dist )

#define DLU_CP_RX_TO_CBIT(dist) \
	DLU_cbit_buld_struct( &ADT_CBIT_rec_message,(ADT_CBIT_send_message*)dist)

#define DLU_GET_opcode(buff) ((ADT_message_header_struct*)buff)->opcode

#define DLU_CBIT_buld_struct(buff, dist) \
	DLU_cbit_buld_struct( (ADT_CBIT_receive_message*)buff,(ADT_CBIT_send_message*)dist)

void DLU_pbit_buld_struct(ADT_PBIT_receive_message *buff,
		ADT_PBIT_send_message *dist);
void DLU_cbit_buld_struct(ADT_CBIT_receive_message *buff,
		ADT_CBIT_send_message *dist);

#endif
