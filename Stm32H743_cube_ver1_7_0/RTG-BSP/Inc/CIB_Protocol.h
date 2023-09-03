/*********************************************************
 * CIB_Protocol.h
 *  ====  ====
 *
 *********************************************************/
#ifndef CIB_PROTOCOL_H_
#define CIB_PROTOCOL_H_

#include <stdlib.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#pragma pack (push,1)


#define NEW_PROTOCOL
#ifdef NEW_PROTOCOL
// @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
// New Protocol
// @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@


// ########################################################
// CIB Header
// ########################################################
typedef struct {
    uint16_t length;
    uint16_t seq;      // Changed to uint16_t
}CIB_Header;


// ########################################################
// CIB to MMC Messages
// ########################################################

//________________________________________________
// 400 Hz
// Sent over CHAN_RX_L_400_Hz channel
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

// Input Discretes bit offset
typedef enum {
    DISC_OUT_TVC_DSCNNCTD =     0,    // pin_G_6:   Future Use
    DISC_OUT_FUSE_1_IGNITION =  1,    // pin_G_3:
    DISC_OUT_FUSE_2_IGNITION =  2,    // pin_I_14:  Future Use
    DISC_OUT_FUSE_3_IGNITION =  3,    // pin_K_2:   Future Use
    DISC_OUT_RST_IMU_1_OUTPUT = 4,    // pin_D_11
    DISC_OUT_RST_IMU_2_OUTPUT = 5,    // pin_J_6
    DISC_OUT_Ext_Com_Dis      = 6,    // pin_I_8
    DISC_OUT_FTS2_Batt_Ign    = 7,    // pin_H_2:   Not Used
	DISC_OUT_WCS_END_COM	  = 8,	  // pin_A_11
    DISC_OUT_COUNT            = 9,
    DISC_OUT_NA               = 0x0F
} CIB_OUTPUT_DISCRETES;

// Output Discretes Bit Offset
typedef enum {
    DISC_IN_SQUIB_BAT_FTS   = 0,       // pin_A_8
    DISC_IN_PTL_SMPL        = 1,       // pin_K_6
    DISC_IN_MPE_2_IND       = 2,       // pin_G_14
    DISC_IN_SnA_ENA_SMPL    = 3,       // pin_K_7
    DISC_IN_MLL_IND_INPUT   = 4,       // pin_K_4
    DISC_IN_PPS_FPGA_2_CPU  = 5,       // pin_A_0
    DISC_IN_CPU2FPGA_GPIO2  = 6,       // pin_E_5
    DISC_IN_CPU2FPGA_GPIO3  = 7,       // pin_E_6
    DISC_IN_CPU2FPGA_GPIO4  = 8,       // pin_E_2
    DISC_IN_CPU2FPGA_GPIO1  = 9,       // pin_I_12
    DISC_IN_DIO1_IMU2_SYNC  = 10,      // pin_J_5
    DISC_IN_DIO2_IMU2_DTRDY = 11,      // pin_J_7
    DISC_IN_DIO2_IMU1_DTRDY = 12,      // pin_B_14
    DISC_IN_DIO1_IMU1_SYNC  = 13,      // pin_B_15
    DISC_IN_COUNT           = 14
} CIB_INPUT_DISCRETES;

// Discretes Data
typedef struct {
    uint16_t in;
    uint16_t out;
} CIB_Discretes_Data;

// IMU Data
typedef struct {
    int32_t X_D_Vel_i;
    int32_t Y_D_Vel_i;
    int32_t Z_D_Vel_i;
    int32_t X_D_Ang_i;
    int32_t Y_D_Ang_i;
    int32_t Z_D_Ang_i;
    int16_t Data_count;
    uint32_t TIME_STAMP;   // was 16 uint16_t
} CIB_IMU_Data;

// 400 Hz Data
typedef struct  {
    CIB_IMU_Data adis16547;          // adis time stamp
    CIB_IMU_Data adis16467;          // Only Ang, Vel = 0.0   LSB Timetag 1= 49.02 us
    CIB_IMU_Data margalit;
    CIB_Discretes_Data discretes;
    uint32_t Data_Ready_Time;        // LSB = 25 uSec
} CIB_400_Hz_Data;

// $00 Hz Message
typedef struct {
    CIB_Header hdr;
    CIB_400_Hz_Data data;
} CIB_400_Hz_Msg;


//________________________________________________
// GPS
// Sent over CHAN_RX_L_GPS channel
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//#include <GPS_Novatel_Protocol.h>

// GPS BESTXYZ Message
typedef struct  {
    CIB_Header hdr;
//    BESTXYZ data;
} CIB_GPS_BESTXYZ_Msg;


//________________________________________________
// Servo
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//#include <Servo_Protocol.h>

// Servo Command
// Sent over CHAN_TX_R_SERVO
typedef struct  {
    CIB_Header hdr;
//    SERVO_CMD data;
} CIB_SERVO_COM_Msg;

// Servo Report
// Sent over CHAN_RX_L_SERVO channel
typedef struct {
    CIB_Header hdr;
//    SERVO_RPT status;
} CIB_SERVO_RPT_Msg;



//________________________________________________
// SnA
// The CIB initiates the Detailed Status request periodically every 10ms (CBIT time)
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//#include <SnA_Protocol.h>

typedef struct {
    CIB_Header hdr;
//    CIB_SnA_data data;
} CIB_SnA_Msg;

typedef struct {
    CIB_Header hdr;
//    CIB_SnA_reply_data data;
} CIB_SnA_Reply_Msg;



//________________________________________________
// WCS ODC
// Sent Over CHAN_TX_R_WCS_ODC channel
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//#include <LRAD.h>

typedef struct {
    CIB_Header hdr;
//    MSL2LCUNOMINALMSG data;
} CIB_WCS_NOMINAL_Msg;

#if 0
typedef struct {
    CIB_Header hdr;
    MSL2LCUVERSIONSMSG data;
}CIB_WCS_VERSIONS_Msg;

typedef struct {
    CIB_Header hdr;
    MSL2LCUGPSIBITRESULTSMSG data;
}CIB_WCS_GPS_IBIT_RESULTS_Msg;

typedef struct {
    CIB_Header hdr;
    MSL2LCUDLTIBITRESULTSMSG data;
}CIB_WCS_DLT_IBIT_RESULTS_Msg;
#endif


// ########################################################
// CIB Command
// Sent over CHAN_TX_L_CIB cjhannrel
// ########################################################
typedef enum {
    CIB_OUTPUT_DISCRETE_CLEAR  = 0,
    CIB_OUTPUT_DISCRETE_ACTIVE = 1,
    CIB_OUTPUT_DISCRETES_READ  = 2,
    CIB_INPUT_DISCRETES_READ   = 3,
    CIB_PBIT_REQUEST           = 4,
    CIB_PRIME_MARGALIT         = 5,
    CIB_PRIME_ADIS             = 6,
    CIB_TOD                    = 7
} CIB_COMMAND;

typedef struct {
    uint8_t   cmd :4;
    uint8_t   discrete :4;
    uint32_t  Year;
    uint32_t  Mon ;
    uint32_t  Day ;
    uint32_t  Sec ;   // seconds from midnight - in 25 uSec
} CIB_Cmd_Data;

typedef struct {
    CIB_Header hdr;
    CIB_Cmd_Data data;
} CIB_Cmd_Msg;



// ########################################################
// Units CBIT / PBIT
// Sent over CHAN_RX_L_CIB
// ########################################################

// BIT Header
typedef enum {
    CIB_CBIT_Type = 0x4243,     // "BC" for CBIT TEST
    CIB_PBIT_Type = 0x4250      // "BP" for PBIT TEST
} CIB_BIT_Type;

typedef struct {
    uint16_t type;
    uint16_t count;
    uint32_t time_tag;
} CIB_BIT_header;


// ########################################################
// CBIT
// ########################################################

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// CBIT ADC
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
typedef union {
    uint16_t Results;
    struct {
        uint16_t cpu_temp : 1;
        uint16_t ext_temp : 1;
        uint16_t ps_1_1v : 1;
        uint16_t ps_1_8v_eth_ps :1;
        uint16_t ps_1_5v : 1;
        uint16_t ps_1_8v : 1;
        uint16_t ps_3_3v : 1;
        uint16_t ps_5_0v : 1;
        uint16_t ps_1_0v : 1;
        uint16_t ps_1_2v : 1;
        uint16_t fpga1 : 1;
        uint16_t fpga2 : 1;
    };
} CIB_ADC_ERROR_FLAGS;

typedef struct {
    uint32_t time_tag;
    CIB_ADC_ERROR_FLAGS errors_flags;
    float cpu_temp;
    float ext_temp;
    float ps_1_1v;
    float ps_1_8v_eth_ps;
    float ps_1_5v;
    float ps_1_8v;
    float ps_3_3v;
    float ps_5_0v;
    float ps_1_0v;
    float ps_1_2v;
    float fpga1;
    float fpga2;
} CIB_CBIT_ADC;

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// CBIT IMU  ADIS16547
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
typedef union {
    uint16_t flags;
    struct {
        uint16_t NA_0 : 1;
        uint16_t boot_memory_failure : 1;
        uint16_t SRAM_error_condition : 1;
        uint16_t SPI_communication_error :1;
        uint16_t NA_4 : 1;
        uint16_t sensor_failure : 1;
        uint16_t flash_memory_update_failure : 1;
        uint16_t processing_overrun : 1;
        uint16_t sync_error : 1;
        uint16_t NA_9_14 : 6;
        uint16_t watchdog_timer_flag : 1;
    };
} CIB_IMU_ADIS16547_SYS_E_FLAGS;

typedef union {
    uint16_t flags;
    struct {
        uint16_t X_gyro_failure : 1;
        uint16_t Y_gyro_failure : 1;
        uint16_t Z_gyro_failure : 1;
        uint16_t X_accel_failure : 1;
        uint16_t Y_accel_failure : 1;
        uint16_t Z_accel_failure : 1;
        uint16_t NA_6_15 : 10;
    };
} CIB_IMU_ADIS16547_DIAG_STS;

typedef struct {
    uint32_t time_tag;
    CIB_IMU_ADIS16547_SYS_E_FLAGS sys_e_flag;
    CIB_IMU_ADIS16547_DIAG_STS diag_sts;
    uint16_t temp_out;                       // twos complement, 1�C per 80 LSB,  25�C = 0x0000
} CIB_CBIT_IMU_ADIS16547;

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// CBIT IMU  ADIS16467
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
typedef union {
    uint16_t flags;
    struct {
        uint16_t NA_0 : 1;
        uint16_t datapath_overrun : 1;
        uint16_t flash_memory_update_failure : 1;
        uint16_t SPI_communication_error :1;
        uint16_t standby_mode  : 1;
        uint16_t sensor_failure : 1;
        uint16_t memory_failure : 1;
        uint16_t clock_error : 1;
        uint16_t NA_8_15 : 8;
    };
} CIB_IMU_ADIS16467_DIAG_STAT;

typedef struct {
    uint32_t time_tag;
    CIB_IMU_ADIS16467_DIAG_STAT diag_sts;
    uint16_t temp_out;                      // twos complement, 1 LSB = 0.1�C, 0�C = 0x0000
} CIB_CBIT_IMU_ADIS16467;


// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// CBIT Margalit
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
#include <MGLT_Protocol.h>

typedef struct {
    uint32_t                                        time_tag;
    int16_t                                         GYRO_X_Temperature; //
    int16_t                                         GYRO_Y_Temperature; //
    int16_t                                         GYRO_Z_Temperature; //
    int16_t                                         GYRO_X_SLD; //
    int16_t                                         GYRO_Y_SLD; //
    int16_t                                         GYRO_Z_SLD; //
    int16_t                                         GYRO_NEG_Supply_Vdc; //
    int16_t                                         GYRO_POS_Supply_Vdc; //
    int16_t                                         ACC_X_Temperature; //
    int16_t                                         ACC_Y_Temperature; //
    int16_t                                         ACC_Z_Temperature; //
    int16_t                                         POS_15V_Internal_Vdc; //
    int16_t                                         Input_Supply_Vdc; //
    int16_t                                         Digital_3_3V_P_Supply; //
    int16_t                                         Analog_3_5V_P_Supply; //
    int16_t                                         A_D_External_Ref; //
    int16_t                                         A_D_Temperature; //
    int16_t                                         A_D_VCC_P2_5V_N2_5V; //
    int16_t                                         PCB_Temperature; //
    uint16_t                                        ETI; //
    BIT_Result_1_BDT                                BIT_Result_1; //
    BIT_Result_2_BDT                                BIT_Result_2; //
    BIT_Result_3_BDT                                BIT_Result_3; //
    BIT_Result_4_BDT                                BIT_Result_4; //
    Warning_BIT_Result_5_BDT                        Warning_BIT_Result_5; //
} CIB_CBIT_Margalit;


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// CBIT GPS
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//#include <GPS_Novatel_Protocol.h>
#include <GPS.h>
typedef struct {
    uint32_t            TimeTag;
    uint8_t             TimeStatus;
    uint16_t            Week;
    uint32_t            ms;
    GPS_RECEIVER_ERROR  error;
    Receiver_Status     rxstat;
    uint8_t             RXSTATUS_Rxed;
    GPS_AUX1_STATUS     aux1stat;
    GPS_AUX2_STATUS     aux2stat;
    GPS_AUX3_STATUS     aux3stat;
    GPS_AUX4_STATUS     aux4stat;
} CIB_CBIT_GPS;


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// CBIT BMS
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
typedef struct BatteryStatus {
     uint16_t EC3:4;       //EC3:0 (Bit 3�0): Error Code
     uint16_t FD:1;        //FD (Bit 4): Fully Discharged
     uint16_t FC:1;        //FC (Bit 5): Fully Charged
     uint16_t DSG:1;       //DSG (Bit 6): Charge FET Test
     uint16_t INIT:1;      //INIT (Bit 7): Initialization
     uint16_t RTA:1;       //RTA (Bit 8): Remaining Time Alarm
     uint16_t RCA:1;       //RCA (Bit 9): Remaining Capacity Alarm
     uint16_t RSVD0:1;     //RSVD (Bit 10): Reserved
     uint16_t TDA:1;       //TDA (Bit 11): Terminate Discharge Alarm
     uint16_t OTA:1;       //OTA (Bit 12): Overtemperature Alarm
     uint16_t RSVD1:1;     //RSVD (Bit 13): Reserved
     uint16_t TCA:1;       //TCA (Bit 14): Terminate Charge Alarm
     uint16_t OCA:1;       //OCA (Bit 15): Overcharged Alarm
}BatteryStatus_t;

typedef struct BattaryData{
     uint32_t   TIME_TAG;
     float      Temperature;           // 0.1K
     uint16_t   Voltage;               // mV
     short      Current;               // 10 mA
     short      AverageCurrent;        // 10 mA
     uint8_t    AbsoluteStateOfCharge; // %
     uint16_t   RemainingCapacity;     // in 10 mAh or 100 mWh
     uint16_t   FullChargeCapacity;    // in 10 mAh or 100 mWh
     uint16_t   CycleCount ;           // cycles
     uint16_t   DesignCapacity;        // in  mAh or 10 mWh
     uint16_t   DesignVoltage;         // mV
     uint16_t   BatteryStatus;
} CIB_CBIT_BMS;


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// CBIT PDU
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
typedef union LTC2991_PassFail_DevAdd1 {
    uint16_t PassFailReg;
    struct {
        uint16_t Main_Current:1;
        uint16_t Servo_Current:1;
        uint16_t Current:1;
        uint16_t S_A_Current:1;
        uint16_t Chassis_Current:1;
        uint16_t Temperature:1;
        uint16_t S_A_Logic_12V:1;
        uint16_t VCC:1;
    };
} LTC2991_P_F_DevAdd1;

typedef union LTC2991_PassFail_DevAdd2 {
    uint16_t PassFailReg;
    struct {
        uint16_t Battery_Voltage:1;
        uint16_t Avionic_Voltage:1;
        uint16_t DLU_Voltage:1;
        uint16_t N15V:1;
        uint16_t VCC:1;
    };
} LTC2991_P_F_DevAdd2;

typedef struct LTC2991_DEV1_CALCULATE_VALUE {
    uint32_t TIME_TAG;
    LTC2991_P_F_DevAdd1 LTC2991_P_F_Dev_Add1;
    float Main_Current;
    float Servo_Current;
    float Current;
    float S_A_Current;
    float Chassis_Current;
    float Temperature;
    float S_A_Logic_12V;
    float VCC;
} CIB_CBIT_PDU_1;

typedef struct LTC2991_DEV2_CALCULATE_VALUE {
    uint32_t TIME_TAG;
    LTC2991_P_F_DevAdd2 LTC2991_P_F_Dev_Add2;
    float Battery_Voltage;
    float Avionic_Voltage;
    float DLU_Voltage;
    float N15V;
    float VCC;
} CIB_CBIT_PDU_2;



//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// CBIT
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
typedef union {
    uint16_t results;
    struct {
        uint16_t INIT_COMPLETED:1;
        uint16_t PBIT_DONE:1;
    };
}CIB_Status;

typedef union {
    uint16_t results;
    struct {
        uint16_t ADC:1;
        uint16_t I547:1;
        uint16_t I467:1;
        uint16_t TRF:1;
        uint16_t GPS:1;
        uint16_t SERVO:1;  // NA ---> Always Set to 0  - Done by MMC
        uint16_t DLU:1;    // NA ---> Always Set to 0  - Done by MMC
        uint16_t S_A1:1;   // NA ---> Always Set to 0  - Done by MMC
        uint16_t S_A2:1;   // NA ---> Always Set to 0
        uint16_t S_A3:1;   // NA ---> Always Set to 0
        uint16_t SKR:1;    // NA ---> Always Set to 0
        uint16_t BMS:1;
        uint16_t PDU_1:1;
        uint16_t PDU_2:1;
        uint16_t ADC_CLONE:1;
        uint16_t NA_3:1;   // NA ---> Always Set to 0
    };
}CIB_CBIT_Error;

typedef struct {
    CIB_CBIT_PDU_1          pdu_1;
    CIB_CBIT_PDU_2          pdu_2;
    CIB_CBIT_ADC            adc;
}CIB_Clone_Data;

typedef struct{
    CIB_BIT_header          header;
    CIB_Status              status;
    CIB_CBIT_Error          sticky_bit;
    CIB_CBIT_Error          current_bit;
    CIB_CBIT_ADC            adc;
    CIB_CBIT_IMU_ADIS16547  imu_adis16547;
    CIB_CBIT_IMU_ADIS16467  imu_adis16467;
    CIB_CBIT_Margalit       margalit;
    CIB_CBIT_GPS            gps;
    CIB_CBIT_BMS            bms;
    // From CLONE
    CIB_Clone_Data			clone;
    uint16_t                checksum;
}CIB_CBIT_data;



// ########################################################
// PBIT
// ########################################################

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// PBIT Main Computer
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
/*
 * val	Holds the current FPGA version creation date and time:
Bits 31-27: Day (1-31)
Bits 26-23: Month (1-12)
Bits 22-17: Year (0-63 => 2000-2063)
Bits 16-12: Hours (0-23)
Bits 11-06: Minutes (0-59)
Bits 05-00: Seconds (0-59)
 *
 */
typedef union {
    uint32_t reg32;
    struct {
        uint32_t Seconds	:6;
        uint32_t Minutes 	:6;
        uint32_t Hours		:5;
        uint32_t Year		:6;
        uint32_t Month		:4;
        uint32_t Day		:5;
    };
} CIB_FPGA_DATE_t;

typedef union {
    uint32_t reg32;
    struct {
        uint32_t ver		:16;
        uint32_t ver_id		:8;
        uint32_t type		:8;
    };
} CIB_FPGA_Ver;

typedef struct {
    uint32_t 				time_tag;
    CIB_FPGA_Ver 			FPGA_ver;
    CIB_FPGA_DATE_t 		FPGA_date;
    uint32_t 				FPGA_SysSin;
} CIB_PBIT_Main_Computer;


// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// PBIT Internal RAMs
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
typedef union {
    uint8_t results;
    struct {
        uint8_t RAM :1;
        uint8_t RAM1 :1;
        uint8_t RAM2 :1;
        uint8_t RAM3 :1;
        uint8_t ITCMRAM :1;
    };
} CIB_PBIT_INTERNAL_RAMS_Err_Flags;

typedef struct {
    uint32_t time_tag;
    CIB_PBIT_INTERNAL_RAMS_Err_Flags pass_or_fail;
} CIB_PBIT_INTERNAL_RAMS;

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// PBIT Internal Flash
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
typedef struct{
    uint32_t time_tag;
    uint8_t pass_or_fail;
}CIB_PBIT_INTERNAL_FLASH;

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// PBIT External Flash
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
typedef struct{
    uint32_t time_tag;
    uint8_t dev_id[6];
    uint8_t pass_or_fail;
}CIB_PBIT_EXTERNAL_FLASH;

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// PBIT SDRAM -
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
typedef struct{
    uint32_t time_tag;
    uint8_t pass_or_fail;
}CIB_PBIT_SDRAM;

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// PBIT IMU  ADIS16547
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
typedef struct {
    uint16_t prod_id;
    uint16_t firm_rev;
    uint16_t firm_dm;
    uint16_t firm_y;
    uint16_t boot_rev;
    uint16_t imu_serial_num;
} CIB_PBIT_IMU_ADIS16547;

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// PBIT IMU  ADIS16467
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
typedef struct {
    uint16_t prod_id;
    uint16_t firm_rev;
    uint16_t firm_dm;
    uint16_t firm_y;
    uint16_t imu_serial_num;
} CIB_PBIT_IMU_ADIS16467;

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// PBIT Margalit
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
typedef struct {
    unsigned short                                  System_Serial_No_LS; //
    unsigned short                                  System_Serial_No_MS; //
    unsigned short                                  SW_Version_LS_No; //
    unsigned short                                  SW_Version_Mid_No; //
    unsigned short                                  SW_Version_MS_No; //
    SW_Version_Date_BDT                             SW_Version_Date; //
    Part_Number_BDT                                 Part_Number; //
} CIB_PBIT_Margalit;

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// PBIT GPS
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
typedef struct {
    uint32_t        type;
    uint8_t         model[16];
    uint8_t         ProductSerialnumber[16];
    uint8_t         HardwareVersion[16];
    uint8_t         Firmwareversion[16];
    uint8_t         BootVersion[16];
    uint8_t         compdate[12];
    uint8_t         compTime[12];
} CIB_PBIT_GPS;


// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// PBIT
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
typedef union {
    uint32_t results;
    struct {
        uint32_t MC :1;
        uint32_t INT_RAMS :1;
        uint32_t INT_FLASH :1;
        uint32_t EXT_FLASH :1;
        uint32_t SDRAM :1;
        uint32_t I547:1;
        uint32_t I467:1;
        uint32_t TRF:1;
        uint32_t GPS:1;
        uint32_t SERVO:1;  // NA ---> Always Set to 0  - Done by MMC
        uint16_t DLU:1;    // NA ---> Always Set to 0  - Done by MMC
        uint16_t S_A1:1;   // NA ---> Always Set to 0  - Done by MMC
        uint16_t S_A2:1;   // NA ---> Always Set to 0
        uint16_t S_A3:1;   // NA ---> Always Set to 0
        uint16_t SKR:1;    // NA ---> Always Set to 0
        uint32_t BMS:1;    // NA ---> Always Set to 0
        uint32_t PDU:1;    // NA ---> Always Set to 0
        uint32_t NA_14:14;
    };
}CIB_PBIT_Error;

typedef struct{
    CIB_BIT_header          header;
    CIB_PBIT_Error          error_bits;
    CIB_PBIT_Main_Computer  mainComputer;
    CIB_PBIT_INTERNAL_RAMS  internal_rams;
    CIB_PBIT_INTERNAL_FLASH internal_flash;
    CIB_PBIT_EXTERNAL_FLASH external_flash;
    CIB_PBIT_SDRAM          sdram;
    CIB_PBIT_IMU_ADIS16547  imu_adis16547;
    CIB_PBIT_IMU_ADIS16467  imu_adis16467;
    CIB_PBIT_Margalit       margalit;
    CIB_PBIT_GPS            gps;
    uint16_t                checksum;
}CIB_PBIT_data;


// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// BIT Message
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
typedef union {
    CIB_BIT_header          header;
    CIB_CBIT_data           cbit;
    CIB_PBIT_data           pbit;
}CIB_BIT_data;

typedef struct {
    CIB_Header               hdr;
    CIB_BIT_data             data;
}CIB_BIT_Msg;


#endif // new
#pragma pack(pop)

#ifdef __cplusplus
}
#endif


#endif /* CIB_PROTOCOL_H_ */


