/*
 * Safe_And_Arm.h
 *
 *  Created on: May 15, 2022
 *      Author: Sharon
 */

#ifndef INC_SAFE_AND_ARM_H_
#define INC_SAFE_AND_ARM_H_

#include <sys/_stdint.h>

#pragma pack (push,1)

//######################################
//############## CONSTATNS #############
//######################################
#define SAA_ADDRESS 0x01

// OPCODES
#define SnA_DETAILED_STATUS_MSG			0X50
#define SnA_STREAM_STATUS_MSG			0X51
#define SnA_ARM_MSG						0XF5
#define SnA_REQUEST_RANDOM_MSG			0X0F
#define SnA_FIRE_FUSE_MSG				0XF0
#define SnA_FIRE_PYRO_MSG				0XAC
#define SnA_VERSIONS_MSG				0XB9
#define SnA_DISARM_MSG					0X52
#define SnA_SAFE_MSG					0XCA
#define SnA_KEEP_ALIVE_MSG				0X53
#define SnA_CHARGE_COMMAND_MSG			0XEE
#define SnA_TAKE_OFF_MSG				0X9D
#define SnA_SENSOR_SIMULATE_COMMAND_MSG	0XB3
#define SnA_LM_GATE_CONTROLLER_MSG		0XAF
#define SnA_PROTOCOL_ERROR_MSG	 		0XFF

// VOLTAGE
#define MINIMAL_IGNITION_VOLTAGE		900 // [V]

// ERROR CODES
#define CRC_INVALID						0X03
#define COMMAND_ERROR					0x06
#define FUSE_ADDRESS					0X09
#define GLOBAL_ERROR					0x0C
#define NACK							0x80FF
#define ACK								0x8010

//######################################
//############## ENUMS #################
//######################################

typedef enum {     // (1) in ICD
	SM_PWR = 0x00,
	SM_BIT = 0x13,
	SM_PRE_LAUNCH = 0x26,
	SM_FLYOUT = 0x49,
	SM_READY_FOR_ARM = 0x8C,
	SM_ARM = 0xF0,
	SM_FIRE = 0x0F,
	SM_DUD = 0xFF,
	SM_DUD_SAFE = 0x1D,
	SM_TRANS = 0xAA
} state_machine_values;

typedef enum {     // (2) in ICD
	HV_FAIL = 0x00,
	HW_ID_FAIL = 0x01,
	FPGA_VER_FAIL = 0x02,
	DETONATOR_FAIL = 0x03,
	IMPACT_G_SW_FAIL = 0x04,
	FPGA_SD_FAIL = 0x05,
	ACC_LAUNCH_FAIL = 0x06,
	CRC_PARAM_FAIL = 0x07,
	CRC_CODE_FAIL = 0x08,
	DIS1_FAIL = 0x09,
	LOGIC_DC_FAIL = 0x0A,
	ARM_PWR_IND_FAIL = 0x0B,
	DIS2_FAIL = 0x0C,
	VIN_ARM_IND_FAIL = 0x0D,
	IMPACT_MEMS_FAIL = 0x0E,
	LOGIC_SEQ_FAIL = 0x10,
	ARM_PULSES_FAIL = 0x11,
	ACC_100G_FAIL = 0x12,
	ACC_16G_FAIL = 0x13,
	CLK_TEST_FAIL = 0x14,
	EXT_TRIG_FAIL = 0x15,
	FPGA1_FAIL = 0x16,
	FPGA2_FAIL = 0x17,
	FPGA_SAFE = 0x18,
	BARO_FAIL = 0x19,
	CURRENT_SENS_FAIL = 0x1A,
	CHARGE_EN0_FAIL = 0x1C,
	CHARGE_EN1_FAIL = 0x1D,
	DYNAMIC_KEY_FAIL = 0x1E,
	TAKE_OFF_END_FAIL = 0x1F
} BIT_result_values;

typedef enum {     // (3) in ICD
	NONE_ = 0x00, FIRE_BY_MEMS = 0x01, FIRE_BY_G_SW = 0x02, FIRE_BY_EXT = 0x03, FIRE_BY_CMD = 0x04
} FIRE_source_values;

// from this enum down (4) - (10) these are not really enums,
// it was just written like this for convenience purposes.
// it represents the ACTUAL BITS that are ON in the register
// when each of these cases happen
typedef enum {     // (4) in ICD
	HW_ID1 = 0x01,
	HW_ID2 = 0x02,
	FPGA1_DIS1_IND = 0x04,
	FPGA2_DIS1_IND = 0x08,
	FPGA1_DIS2_IND = 0x10,
	FPGA2_DIS2_IND = 0x20,
	MCU_EXTI_TRIGGER = 0x40,
	MCU_SENS_INIT_OK = 0x80
} status_register_1;

typedef enum {     // (5) in ICD
	FPGA1_MAIN_SM0 = 0x01,
	FPGA1_MAIN_SM1 = 0x02,
	FPGA1_MAIN_SM2 = 0x04,
	FPGA1_SD_END = 0x08,
	FPGA1_SEQ_OK = 0x10,
	FPGA1_SD_HALF = 0x20,
	FPGA1_ARM_CMD_EN = 0x40,
	FPGA1_HV_SAFE = 0x80
} status_register_2;

typedef enum {     // (6) in ICD
	FPGA2_MAIN_SM0 = 0x01,
	FPGA2_MAIN_SM1 = 0x02,
	FPGA2_MAIN_SM2 = 0x04,
	FPGA2_SD_END = 0x08,
	FPGA2_SEQ_OK = 0x10,
	FPGA2_SD_HALF = 0x20,
	FPGA2_ARM_CMD_EN = 0x40,
	FPGA2_HV_SAFE = 0x80
} status_register_3;

typedef enum {     // (7) in ICD
	MCU_VIN_ARM_IND = 0x01,
	MCU_ARM_WORD_OK = 0x02,
	DET_CONNCET = 0x04,
	MCU_SST_END = 0x08,
	MCU_ERROR = 0x10,
	MCU_ARM_HV_IND = 0x20,
	MCU_ARM_PWR_IND = 0x40,
	MCU_UNSAFE = 0x80
} status_register_4;

typedef enum {     // (8) in ICD
	FPGA1_CRC_OK = 0x01,
	FPGA2_CRC_OK = 0x02,
	FPGA1_ARM_OK = 0x08,
	FPGA1_ARM_WORD_FAIL = 0x10,
	FPGA2_AIRBORNE = 0x20,
	FPGA2_ARM_OK = 0x40,
	FPGA2_ARM_WORD_FAIL = 0x80
} status_register_5;

typedef enum {     // (9) in ICD
	FPGA1_AIRBORNE = 0x01,
	FPGA1_UFM_OK = 0x02,
	FPGA1_UFM_ERROR = 0x04,
	FPGA1_SD_EN = 0x08,
	FPGA2_UFM_OK = 0x10,
	FPGA2_UFM_ERROR = 0x20,
	FPGA2_PWM_FREQ_OK = 0x40,
	FPGA2_PWM_FREQ_ERROR = 0x80
} status_register_6;

typedef enum {     // (10) in ICD
	STREAM_FLAG = 0x02, WAKE_FLAG = 0x04, NO_COMM_FLAG = 0x08, UART_SIM = 0x20
} status_register_7;

//######################################
//############## headers ###############
//######################################

typedef struct {
	uint16_t length;
	uint8_t seq;
	uint8_t spare;
} Hdr_RTG_SnA;

typedef struct st_SnA_header {
	uint16_t preamble;
	uint8_t sync;
	uint8_t length;
	/*payload*/
	uint16_t message_ID;
	uint8_t address;
	uint8_t opcode;
} SnA_header;

//######################################
//##### messages without RTG header ####
//######################################

//######################################
// detailed status message - OPCODE 0X50
//######################################

typedef struct {
	SnA_header hdr;
	uint8_t stream_mode;
	uint32_t crc32;
} detailed_status_message_struct;

typedef struct {
	SnA_header hdr;
	uint8_t state_machine;
	uint8_t mcu_sd;
	uint32_t mcu_bit;
	uint16_t hv_capacitor;
	uint16_t logic_dc;
	uint16_t vdd;
	int16_t acc_x_1;
	int16_t acc_y_1;
	int16_t acc_z_1;
	int16_t acc_x_2;
	int16_t acc_y_2;
	int16_t acc_z_2;
	uint8_t fpga_1_sd;
	uint8_t fpga_2_sd;
	uint16_t fire_delay;
	uint16_t impact_counter;
	uint16_t g_switch_counter;
	uint8_t fire_source;
	uint16_t count_accelometer;
	uint8_t fpga_1_seq_fail;
	uint8_t fpga_2_seq_fail;
	uint8_t fpga_1_debug;
	uint8_t fpga_2_debug;
	uint8_t fpga_1_arm_window;
	uint8_t fpga_2_arm_window;
	uint16_t barometer_read;
	uint16_t current_read;
	uint16_t takeoff_flyout_timeout_MCU;
	uint8_t takeoff_flyout_timeout_fpga_1;
	uint8_t takeoff_flyout_timeout_fpga_2;
	uint8_t spare [6];
	uint8_t status_bits_1;
	uint8_t status_bits_2;
	uint8_t status_bits_3;
	uint8_t status_bits_4;
	uint8_t status_bits_5;
	uint8_t status_bits_6;
	uint8_t status_bits_7;
	uint8_t status_bits_8_spare;
	uint32_t crc32;
} detailed_status_message_reply_struct;

typedef struct {
	uint32_t TimeTag;
//	uint8_t error;
	uint32_t mcu_bit;			//BIT result
	uint8_t mcu_version_low;     //Verson
	uint8_t mcu_version_high;
	uint16_t hv_capacitor;		//Voltage
	uint16_t logic_dc;
	uint16_t vdd;
	uint8_t fpga_1_version_low;     //FPGA Versions
	uint8_t fpga_1_version_high;
	uint8_t fpga_2_version_low;
	uint8_t fpga_2_version_high;
	uint8_t fpga_1_seq_fail;     //FPGA fail
	uint8_t fpga_2_seq_fail;
	uint8_t fpga_1_debug;		//FPGA debug
	uint8_t fpga_2_debug;
	uint8_t status_bits_1;		//IO Status
	uint8_t status_bits_2;
	uint8_t status_bits_3;
	uint8_t status_bits_4;
	uint8_t status_bits_5;
	uint8_t status_bits_6;
	uint8_t status_bits_7;
	uint8_t status_bits_8_spare;
} SAA_PBIT_t;

typedef struct {
	uint32_t TimeTag;
//	uint8_t error;
	uint32_t mcu_bit;			//BIT result
	uint8_t MCU_state_machine;     //MCU SM
	uint16_t hv_capacitor;		//Voltage
	uint16_t logic_dc;
	uint16_t vdd;
	uint8_t fpga_1_seq_fail;     //FPGA fail
	uint8_t fpga_2_seq_fail;
	uint8_t fpga_1_debug;		//FPGA debug
	uint8_t fpga_2_debug;
	uint8_t status_bits_1;		//IO Status bits
	uint8_t status_bits_2;
	uint8_t status_bits_3;
	uint8_t status_bits_4;
	uint8_t status_bits_5;
	uint8_t status_bits_6;
	uint8_t status_bits_7;
	uint8_t status_bits_8_spare;
} SAA_CBIT_t;

//######################################
// stream status message - OPCODE 0X51
//######################################

typedef struct {
	SnA_header hdr;
	uint32_t crc32;
} stream_status_message_struct;

typedef struct {
	SnA_header hdr;
	uint8_t MCU_state_machine;
	uint8_t status_1;
	uint8_t status_6;
	uint16_t HV_capacitor;
	uint8_t FPGA_1_safe_delay;
	uint8_t FPGA_2_safe_delay;
	uint32_t crc32;
} stream_status_message_reply_struct;

//######################################
// ARM message - OPCODE 0XF5
//######################################

typedef struct {
	SnA_header hdr;
	uint16_t impact_delay;
	uint8_t frequency;
	uint8_t duty;
	uint8_t FPGA_CMD;
	uint8_t ARM_word [16];
	uint32_t crc32;
} ARM_message_struct;

typedef struct {
	SnA_header hdr;
	uint16_t data;
	uint32_t crc32;
} ARM_message_reply_struct;

//######################################
// random number request message - OPCODE 0X0F
//######################################

typedef struct {
	SnA_header hdr;
	uint32_t crc32;
} random_number_request_message_struct;

typedef struct {
	SnA_header hdr;
	uint32_t data;
	uint32_t crc32;
} random_number_request_message_reply_struct;

//######################################
// fire fuse message - OPCODE 0XF0
//######################################

typedef struct {
	SnA_header hdr;
	uint32_t crc32;
} fire_fuse_message_struct;

typedef struct {
	SnA_header hdr;
	uint16_t data;
	uint32_t crc32;
} fire_fuse_message_reply_struct;

//######################################
// fire pyro message - OPCODE 0XAC
//######################################

typedef struct {
	SnA_header hdr;
	uint32_t crc32;
} fire_pyro_message_struct;

typedef struct {
	SnA_header hdr;
	uint16_t data;
	uint32_t crc32;
} fire_pyro_message_reply_struct;

//######################################
// disarm message - OPCODE 0X52
//######################################

typedef struct {
	SnA_header hdr;
	uint32_t crc32;
} disarm_message_struct;

typedef struct {
	SnA_header hdr;
	uint16_t data;
	uint32_t crc32;
} disarm_message_reply_struct;

//######################################
// safe message - OPCODE 0XCA
//######################################

typedef struct {
	SnA_header hdr;
	uint32_t crc32;
} safe_message_struct;

typedef struct {
	SnA_header hdr;
	uint16_t data;
	uint32_t crc32;
} safe_message_reply_struct;

//######################################
// charge message - OPCODE 0XEE
//######################################

typedef struct {
	SnA_header hdr;
	uint32_t crc32;
} charge_message_struct;

typedef struct {
	SnA_header hdr;
	uint16_t data;
	uint32_t crc32;
} charge_message_reply_struct;

//######################################
// keep alive message - OPCODE 0X53
//######################################

typedef struct {
	SnA_header hdr;
	uint32_t crc32;
} keep_alive_message_struct;

typedef struct {
	SnA_header hdr;
	uint16_t data;
	uint32_t crc32;
} keep_alive_message_reply_struct;

//######################################
// simulate sensors message - OPCODE 0XB3
//######################################

typedef struct {
	SnA_header hdr;
	uint8_t acc_x_1;
	uint8_t acc_y_1;
	uint8_t acc_z_1;
	uint8_t acc_x_2;
	uint8_t acc_y_2;
	uint8_t acc_z_2;
	uint16_t barometer;
	uint32_t crc32;
} simulate_sensors_message_struct;

typedef struct {
	SnA_header hdr;
	uint16_t data;
	uint32_t crc32;
} simulate_sensors_message_reply_struct;

//######################################
// versions message - OPCODE 0XB9
//######################################

typedef struct {
	SnA_header hdr;
	uint32_t crc32;
} versions_message_struct;

typedef struct {
	SnA_header hdr;
	uint8_t mcu_version_low;
	uint8_t mcu_version_high;
	uint32_t parameters_crc;
	uint32_t code_crc;
	uint8_t fpga_1_version_low;
	uint8_t fpga_1_version_high;
	uint8_t fpga_2_version_low;
	uint8_t fpga_2_version_high;
	uint16_t fpga_1_crc;
	uint16_t fpga_2_crc;
	uint8_t unit_ICD_protocol_version_low;
	uint8_t unit_ICD_protocol_version_high;
	uint32_t crc32;
} versions_message_reply_struct;

//######################################
// global error message - OPCODE 0XFF
//######################################

typedef struct {
	SnA_header hdr;
	uint8_t data;
	uint32_t crc32;
} global_error_message_reply_struct;

//######################################
//###### messages with RTG header ######
//######################################

typedef struct {
	Hdr_RTG_SnA hdr;
	detailed_status_message_struct data;
} detailed_status_with_RTG_header;

typedef struct {
	Hdr_RTG_SnA hdr;
	stream_status_message_struct data;
} stream_status_with_RTG_header;

typedef struct {
	Hdr_RTG_SnA hdr;
	ARM_message_struct data;
} ARM_with_RTG_header;

typedef struct {
	Hdr_RTG_SnA hdr;
	random_number_request_message_struct data;
} random_number_request_with_RTG_header;

typedef struct {
	Hdr_RTG_SnA hdr;
	fire_fuse_message_struct data;
} fire_fuse_with_RTG_header;

typedef struct {
	Hdr_RTG_SnA hdr;
	fire_pyro_message_struct data;
} fire_pyro_with_RTG_header;

typedef struct {
	Hdr_RTG_SnA hdr;
	versions_message_struct data;
} versions_with_RTG_header;

typedef struct {
	Hdr_RTG_SnA hdr;
	disarm_message_struct data;
} disarm_with_RTG_header;

typedef struct {
	Hdr_RTG_SnA hdr;
	safe_message_struct data;
} safe_with_RTG_header;

typedef struct {
	Hdr_RTG_SnA hdr;
	charge_message_struct data;
} charge_with_RTG_header;

typedef struct {
	Hdr_RTG_SnA hdr;
	keep_alive_message_struct data;
} keep_alive_with_RTG_header;

typedef struct {
	Hdr_RTG_SnA hdr;
	simulate_sensors_message_struct data;
} simulate_sensors_with_RTG_header;
#pragma pack(pop)

/*
 // * Global Structs for save uart Message
 * defined in Safe_And_Arm.c
 */
extern detailed_status_message_reply_struct SAA_detStaMes;
extern versions_message_reply_struct SAA_verMes;
extern global_error_message_reply_struct SSA_errorMessage;

void RTG_DetailedStatusRxCbit( uint8_t *buffer );
void RTG_Detailed_StatusRxPbit(uint8_t *buffer);
void RTG_VersionsRxPbit(uint8_t *buffer);
// send command
void RTG_detailed_status_tx();
void RTG_versions_tx();

#endif /* INC_SAFE_AND_ARM_H_ */
