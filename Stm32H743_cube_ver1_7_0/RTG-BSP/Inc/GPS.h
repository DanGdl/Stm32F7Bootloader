/*
 * GPS.h
 *
 *  Created on: Feb 27, 2022
 *      Author: itzhaki
 */

#ifndef INC_GPS_H_
#define INC_GPS_H_

#include <sys/_stdint.h>

#define GPS_HEADER_SIZE 10

#define GPS_BIT_TYPE (uint16_t)0x0063

#define GPS_BIT_DATA_LENGTH 50

#define SOT "SOT"
#define GPS_SYNC_SIZE (sizeof(SOT)-1)

#define GPS_BIT_REQ_LEN 10

#define PERIODICBIT 0x1000

#pragma pack(push,1)

#define OEM7_OPCODE_BESTXYZ		241
#define OEM7_OPCODE_VERSION		37
#define OEM7_OPCODE_RXSTATUS	93
// OEM7<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
/*
 * size 28 bytes
 */
typedef struct {
	uint8_t 		Sync[3];
	uint8_t 		HeaderLength;
	uint16_t 		MessageID;
	uint8_t			MessageType;
	uint8_t			PortAddress;
	uint16_t		MessageLength;
	uint16_t		Sequence;
	uint8_t			IdleTime;
	uint8_t			TimeStatus;
	uint16_t		Week;
	uint32_t		ms;
	uint32_t		ReceiverStatus;
	uint16_t		Reserved;
	uint16_t		ReceiverSWVersion;

}OEM7_Hader_t;

typedef struct OEM7_RXSTATUS{
	OEM7_Hader_t header;
	uint32_t		error;
	uint32_t		stats;
	uint32_t		rxstat;
	uint32_t		rxstatpri;
	uint32_t		rxstatset;
	uint32_t		rxstatclear;
	uint32_t		aux1stat;
	uint32_t		aux1statpri;
	uint32_t		aux1statset;
	uint32_t		aux1statclear;
	uint32_t		aux2stat;
	uint32_t		aux2statpri;
	uint32_t		aux2statset;
	uint32_t		aux2statclear;
	uint32_t		aux3stat;
	uint32_t		aux3statpri;
	uint32_t		aux3statset;
	uint32_t		aux3statclear;
	uint32_t		aux4stat;
	uint32_t		aux4statpri;
	uint32_t		aux4statset;
	uint32_t		aux4statclear;
	uint32_t		crc;
}OEM7_RXSTATUS_t;

typedef struct OEM7_VersionComp{
	uint32_t		type;
	uint8_t			model[16];
	uint8_t			ProductSerialnumber[16];
	uint8_t			HardwareVersion[16];
	uint8_t			Firmwareversion[16];
	uint8_t			BootVersion[16];
	uint8_t			compdate[12];
	uint8_t			compTime[12];

}OEM7_VersionComp_t;

typedef struct OEM7_Version{
	OEM7_Hader_t header;
	uint32_t		Numberofcomponents;
	OEM7_VersionComp_t comp[3];
}OEM7_Version_t;

typedef struct
{
	uint32_t 		TimeTag;
	uint8_t			TimeStatus;
	uint16_t		Week;
	uint32_t		ms;
	uint32_t		ReceiverStatus;
	OEM7_VersionComp_t comp;
} GPS_PBIT_RESULTS_t;

typedef struct {
	uint32_t 		TimeTag;
	uint8_t			TimeStatus;
	uint16_t		Week;
	uint32_t		ms;
	uint32_t		error;
	uint32_t		rxstat;
	uint32_t		aux1stat;
	uint32_t		aux2stat;
	uint32_t		aux3stat;
	uint32_t		aux4stat;
} GPS_CBIT_RESULTS_t;

typedef union {
    uint32_t word;
    struct {
        uint32_t DRAM : 1;
        uint32_t Invalid_FW : 1;
        uint32_t ROM : 1;
        uint32_t NA_3 : 1;
        uint32_t ESN_Access : 1;
        uint32_t AuthCode : 1;
        uint32_t NA_6 : 1;
        uint32_t Supply_Voltage : 1;
        uint32_t NA_8 : 1;
        uint32_t Temperature : 1;
        uint32_t MINOS : 1;
        uint32_t PLL_RF : 1;
        uint32_t NA_12_14 : 3;
        uint32_t NVM : 1;
        uint32_t SW_res_limit_exceeded : 1;
        uint32_t Model_invalid_4_rec : 1;
        uint32_t NA_18_19 : 2;
        uint32_t Remote_loading : 1;
        uint32_t Export_restriction : 1;
        uint32_t Safe_Mode : 1;
        uint32_t NA_23_30 : 8;
        uint32_t Comp_HW_failure : 1;
    };
} GPS_RECEIVER_ERROR;

typedef union {
	uint32_t word;
	struct {
		uint32_t                                   Error_flag :1; //
		uint32_t                                   Temperature_warning :1; //
		uint32_t                                   Voltage_warning :1; //
		uint32_t                                   Primary_antenna_power_is_off :1; //
		uint32_t                                   LNA_failure :1; //
		uint32_t                                   Primary_antenna_circuit_open :1; //
		uint32_t                                   Primary_antenna_short_circuit :1; //
		uint32_t                                   CPU_overload :1; //
		uint32_t                                   COM_buffe_overrun :1; //
		uint32_t                                   spoofing_detected :1; //
		uint32_t                                   bit_ul_10 :1; //
		uint32_t                                   Limk_overrun :1; //
		uint32_t                                   Input_overrun :1; //
		uint32_t                                   AUX_transmit_overrun :1; //
		uint32_t								   Antenna_gain_out_range :1; //
		uint32_t                                   Jammer_detected :1; //
		uint32_t                                   INS_rest :1; //
		uint32_t                                   IMU_com_failure :1; //
		uint32_t                                   Almanac_invalid :1; //
		uint32_t                                   Position_solution_invalid :1; //
		uint32_t                                   Position_Fixed :1; //
		uint32_t                                   Clock_steering_disabled :1; //
		uint32_t                                   Clock_model_invalid :1; //
		uint32_t                                   External_oscilator_locked :1; //
		uint32_t                                   Software_resource_warning :1; //
		uint32_t                                   Version_bit_0 :1; //
		uint32_t                                   Version_Bit_1 :1; //
		uint32_t                                   HDR_tracking :1; //
		uint32_t                                   Digital_filtering_enabled :1; //
		uint32_t                                   Auxilary_3_event :1; //
		uint32_t                                   Auxilary_2_event :1; //
		uint32_t                                   Auxilary_1_event :1; //
	};
} Receiver_Status;

/** Auxiliary 1 Status strings - refer to Oem7 manual. */
typedef union {
    uint32_t word;
    struct {
        uint32_t Jammer_detected_on_RF1 : 1;
        uint32_t Jammer_detected_on_RF2 : 1;
        uint32_t Jammer_detected_on_RF3 : 1;
        uint32_t Position_averaging_On : 1;
        uint32_t Jammer_detected_on_RF4 : 1;
        uint32_t Jammer_detected_on_RF5 : 1;
        uint32_t Jammer_detected_on_RF6 : 1;
        uint32_t USB_not_connected : 1;
        uint32_t USB1_buffer_overrun : 1;
        uint32_t USB2_buffer_overrun : 1;
        uint32_t USB3_buffer_overrun : 1;
        uint32_t NA_11 : 1;
        uint32_t Profile_Activation_Error : 1;
        uint32_t Throttled_Ethernet_Reception : 1;
        uint32_t NA_14_17 : 4;
        uint32_t Ethernet_not_connected : 1;
        uint32_t ICOM1_buffer_overrun : 1;
        uint32_t ICOM2_buffer_overrun : 1;
        uint32_t ICOM3_buffer_overrun : 1;
        uint32_t NCOM1_buffer_overrun : 1;
        uint32_t NCOM2_buffer_overrun : 1;
        uint32_t NCOM3_buffer_overrun : 1;
        uint32_t NA_25_30 : 6;
        uint32_t IMU_measurement_outlier_detected : 1;
    };
} GPS_AUX1_STATUS;

/** Auxiliary 2 Status strings - refer to Oem7 manual. */
typedef union {
    uint32_t word;
    struct {
        uint32_t SPI_Communication_Failure : 1;
        uint32_t I2C_Communication_Failure : 1;
        uint32_t COM4_buffer_overrun : 1;
        uint32_t COM5_buffer_overrun : 1;
        uint32_t NA_4_8 : 5;
        uint32_t COM1_buffer_overrun : 1;
        uint32_t COM2_buffer_overrun : 1;
        uint32_t COM3_buffer_overrun : 1;
        uint32_t PLL_RF1_unlock : 1;
        uint32_t PLL_RF2_unlock : 1;
        uint32_t PLL_RF3_unlock : 1;
        uint32_t PLL_RF4_unlock : 1;
        uint32_t PLL_RF5_unlock : 1;
        uint32_t PLL_RF6_unlock : 1;
        uint32_t CCOM1_buffer_overrun : 1;
        uint32_t CCOM2_buffer_overrun : 1;
        uint32_t CCOM3_buffer_overrun : 1;
        uint32_t CCOM4_buffer_overrun : 1;
        uint32_t CCOM5_buffer_overrun : 1;
        uint32_t CCOM6_buffer_overrun : 1;
        uint32_t ICOM4_buffer_overrun : 1;
        uint32_t ICOM5_buffer_overrun : 1;
        uint32_t ICOM6_buffer_overrun : 1;
        uint32_t ICOM7_buffer_overrun : 1;
        uint32_t Secondary_antenna_not_powered : 1;
        uint32_t Secondary_antenna_open_circuit : 1;
        uint32_t Secondary_antenna_short_circuit : 1;
        uint32_t Reset_loop_detected : 1;
    };
} GPS_AUX2_STATUS;

/** Auxiliary 3 Status strings - refer to Oem7 manual. */
typedef union {
    uint32_t word;
    struct {
        uint32_t SCOM_buffer_overrun : 1;
        uint32_t WCOM1_buffer_overrun : 1;
        uint32_t FILE_buffer_overrun : 1;
        uint32_t NA_3_28 : 26;
        uint32_t Web_content_is_corrupt_or_does_not_exist : 1;
        uint32_t RF_Calibration_Data_is_present_and_in_error : 1;
        uint32_t RF_Calibration_data_exists_and_has_no_errors : 1;
    };
} GPS_AUX3_STATUS;

typedef union {
    uint32_t word;
    struct {
        uint32_t LT_60_percent_of_available_satellites_are_tracked_well : 1;
        uint32_t LT_15_percent_of_available_satellites_are_tracked_well : 1;
        uint32_t NA_2_11 : 10;
        uint32_t Clock_freewheeling_due_to_bad_position_integrity : 1;
        uint32_t NA_13 : 1;
        uint32_t Usable_RTK_Corrections_LT_60_percent : 1;
        uint32_t Usable_RTK_Corrections_LT_15_percent : 1;
        uint32_t Bad_RTK_Geometry : 1;
        uint32_t NA_27_18 : 2;
        uint32_t Long_RTK_Baseline : 1;
        uint32_t Poor_RTK_COM_link : 1;
        uint32_t Poor_ALIGN_COM_link : 1;
        uint32_t GLIDE_Not_Active : 1;
        uint32_t Bad_PDP_Geometry : 1;
        uint32_t No_TerraStar_Subscription : 1;
        uint32_t NA_25_27 :3;
        uint32_t Bad_PPP_Geometry : 1;
        uint32_t NA_29 : 1;
        uint32_t No_INS_Alignment : 1;
        uint32_t INS_not_converged : 1;
    };
} GPS_AUX4_STATUS;





extern OEM7_Hader_t oem_7;
#define IS_OEM7_HEADER(buff) 			( ( (*(uint32_t*)&oem_7 ) & 0x0ffffff ) ==( ( *((uint32_t*)buff) ) & 0x0ffffff ) )
#define OEM7_MESSAGE_LENGTH( buff )  	( ((OEM7_Hader_t*)buff)->MessageLength )
#define OEM7_MESSAGE_ID( buff )  		( ((OEM7_Hader_t*)buff)->MessageID )
void GPS_SendVersion();
// OEM7<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

typedef struct
{
	uint8_t sync[GPS_SYNC_SIZE];
	uint8_t serialNum;
	uint16_t type;
	uint16_t length;
	uint16_t checksum;
	uint32_t bitResults;
	uint16_t power;
	uint16_t temperature;
	uint32_t Reserv;
	uint32_t Ttag;
	uint8_t cpldVer[7];
	uint8_t swVer[7];
	uint8_t swDate[12];
} GPS_UART_IN_t;

typedef struct
{
	uint8_t sync[GPS_SYNC_SIZE];
	uint8_t serialNum;
	uint16_t type;
	uint16_t length;
	uint16_t checksum;
} GPS_HEADER;


#pragma pack(pop)


#endif /* INC_GPS_H_ */
