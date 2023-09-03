/*********************************************************
 * MGLT_Protocol
 *  ====  ====
 * Modified from ADI Generated code
 *
 * Author: Benny Kalman
 *********************************************************/
#ifndef MGLT_PROTOCOL_H
#define MGLT_PROTOCOL_H

#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

#pragma pack (push,1)


/* MGLT */
#ifndef _MESSAGES_ENUM_TYPE_MGLT
#define _MESSAGES_ENUM_TYPE_MGLT
typedef enum {
	IMU_MGLT_msg                                   = 1,
	MUX_Data_msg                                   = 0
}MGLT_OPCODE;
#endif /* _MESSAGES_ENUM_TYPE_MGLT*/


/* BIT_Status_BDT */
#ifndef _TYPE_BIT_Status_BDT
#define _TYPE_BIT_Status_BDT
typedef struct {
	unsigned short                                  GX_Fail :1; //
	unsigned short                                  GY_Fail :1; //
	unsigned short                                  GZ_Fail :1; //
	unsigned short                                  AX_Fail :1; //
	unsigned short                                  AY_Fail :1; //
	unsigned short                                  AZ_Fail :1; //
	unsigned short                                  P_Supply :1; //
	unsigned short                                  bit_7 :1; //
	unsigned short                                  ADC_Not_Valid :1; //
	unsigned short                                  LDNR :1; //
	unsigned short                                  P_ON_BIT_FAIL_1 :1; //
	unsigned short                                  Injection_Fail :1; //
	unsigned short                                  bit_12 :1; //
	unsigned short                                  bit_13 :1; //
	unsigned short                                  Supplement_BIT :1; //
	unsigned short                                  Sensors_Data_Not_Valid :1; //
} BIT_Status_BDT;
#endif /* _TYPE_BIT_Status_BDT*/

/* MGLT_System_Modes */
#ifndef _ENUM_TYPE_MGLT_System_Modes
#define _ENUM_TYPE_MGLT_System_Modes
typedef enum {
    PWR_ON_BIT_MODE                                = 1,
    OPER_MODE                                      = 2,
    INJECTION_MODE                                 = 7
}MGLT_System_Modes;
#endif /* _ENUM_TYPE_MGLT_System_Modes*/


/* MGLT_MUX_Data_ID */
#ifndef _ENUM_TYPE_MGLT_MUX_Data_ID
#define _ENUM_TYPE_MGLT_MUX_Data_ID
typedef enum {
    ID_GYRO_X_Temperature                             = 0,
    ID_GYRO_Y_Temperature                             = 1,
    ID_GYRO_Z_Temperature                             = 2,
    ID_GYRO_X_SLD                                     = 3,
    ID_GYRO_Y_SLD                                     = 4,
    ID_GYRO_Z_SLD                                     = 5,
    ID_GYRO_POS_Supply_Vdc                            = 6,
    ID_GYRO_NEG_Supply_Vdc                            = 7,
    ID_ACC_X_Temperature                              = 8,
    ID_ACC_Y_Temperature                              = 9,
    ID_ACC_Z_Temperature                              = 10,
    ID_POS_15V_Internal_Vdc                           = 11,
    ID_Input_Supply_Vdc                               = 12,
    ID_Digital_3_3V_P_Supply                          = 13,
    ID_Analog_3_5V_P_Supply                           = 14,
    ID_A_D_External_Ref                               = 15,
    ID_A_D_Temperature                                = 16,
    ID_A_D_VCC_P2_5V_N2_5V                            = 17,
    ID_PCB_Temperature                                = 18,
    ID_ETI                                            = 19,
    ID_BIT_Result_1                                   = 20,
    ID_BIT_Result_2                                   = 21,
    ID_BIT_Result_3                                   = 22,
    ID_BIT_Result_4                                   = 23,
    ID_Warning_BIT_Results_5                          = 24,
    ID_System_Serial_No_LS                            = 25,
    ID_System_Serial_No_MS                            = 26,
    ID_SW_Version_LS_No                               = 27,
    ID_SW_Version_Mid_No                              = 28,
    ID_SW_Version_MS_No                               = 29,
    ID_SW_Version_Date                                = 30,
    ID_System_Part_No                                 = 31
}MGLT_MUX_Data_ID;
#endif /* _ENUM_TYPE_MGLT_MUX_Data_ID*/




/* MUX_Status_BDT */
#ifndef _TYPE_MUX_Status_BDT
#define _TYPE_MUX_Status_BDT
typedef struct {
	unsigned short                                  Mux_Count :8; //
	unsigned short                                  System_Ready :1; //
	unsigned short                                  bit_9 :1; //
	unsigned short                                  System_Mode :3; //
	unsigned short                                  bit_13 :1; //
	unsigned short                                  bit_14 :1; //
	unsigned short                                  bit_15 :1; //
} MUX_Status_BDT;
#endif /* _TYPE_MUX_Status_BDT*/

/* BIT_Result_1_BDT */
#ifndef _TYPE_BIT_Result_1_BDT
#define _TYPE_BIT_Result_1_BDT
typedef struct {
	unsigned short                                  G_X_A_D_Saturation :1; //
	unsigned short                                  G_Y_A_D_Saturation :1; //
	unsigned short                                  G_Z_A_D_Saturation :1; //
	unsigned short                                  G_X_BIT :1; //
	unsigned short                                  G_Y_BIT :1; //
	unsigned short                                  G_Z_BIT :1; //
	unsigned short                                  G_X_SLD :1; //
	unsigned short                                  G_Y_SLD :1; //
	unsigned short                                  G_Z_SLD :1; //
	unsigned short                                  G_X_Temp_Limit :1; //
	unsigned short                                  G_Y_Temp_Limit :1; //
	unsigned short                                  G_Z_Temp_Limit :1; //
	unsigned short                                  GYRO_POS_Suply :1; //
	unsigned short                                  GYRO_NEG_Suply :1; //
	unsigned short                                  bit_14 :1; //
	unsigned short                                  bit_15 :1; //
} BIT_Result_1_BDT;
#endif /* _TYPE_BIT_Result_1_BDT*/

/* BIT_Result_2_BDT */
#ifndef _TYPE_BIT_Result_2_BDT
#define _TYPE_BIT_Result_2_BDT
typedef struct {
	unsigned short                                  A_X_A_D_Saturation :1; //
	unsigned short                                  A_Y_A_D_Saturation :1; //
	unsigned short                                  A_Z_A_D_Saturation :1; //
	unsigned short                                  A_X_Temp_Limit :1; //
	unsigned short                                  A_Y_Temp_Limit :1; //
	unsigned short                                  A_Z_Temp_Limit :1; //
	unsigned short                                  POS_15V_Internal_Vdc_Fault :1; //
	unsigned short                                  Input_Supply_Vdc_Fault :1; //
	unsigned short                                  A_D_External_Ref_Fault :1; //
	unsigned short                                  A_D_VCC_P2_5V_N2_5V_Fault :1; //
	unsigned short                                  bit_10 :1; //
	unsigned short                                  LNDR_Read_Fail :1; //
	unsigned short                                  bit_12 :1; //
	unsigned short                                  bit_13 :1; //
	unsigned short                                  Digital_3_3V_Supply_Fault :1; //
	unsigned short                                  Analog_3_5V_Supply_Fault :1; //
} BIT_Result_2_BDT;
#endif /* _TYPE_BIT_Result_2_BDT*/

/* BIT_Result_3_BDT */
#ifndef _TYPE_BIT_Result_3_BDT
#define _TYPE_BIT_Result_3_BDT
typedef struct {
	unsigned short                                  A_D_G_X_Fail :1; //
	unsigned short                                  A_D_G_Y_Fail :1; //
	unsigned short                                  A_D_G_Z_Fail :1; //
	unsigned short                                  A_D_A_X_Fail :1; //
	unsigned short                                  A_D_A_Y_Fail :1; //
	unsigned short                                  A_D_A_Z_Fail :1; //
	unsigned short                                  bit_6 :1; //
	unsigned short                                  A_D_MUX_Fail :1; //
	unsigned short                                  bit_8 :1; //
	unsigned short                                  bit_9 :1; //
	unsigned short                                  bit_10 :1; //
	unsigned short                                  bit_11 :1; //
	unsigned short                                  bit_12 :1; //
	unsigned short                                  bit_13 :1; //
	unsigned short                                  bit_14 :1; //
	unsigned short                                  bit_15 :1; //
} BIT_Result_3_BDT;
#endif /* _TYPE_BIT_Result_3_BDT*/

/* BIT_Result_4_BDT */
#ifndef _TYPE_BIT_Result_4_BDT
#define _TYPE_BIT_Result_4_BDT
typedef struct {
	unsigned short                                  Processor_Fail :1; //
	unsigned short                                  RAM_Fail :1; //
	unsigned short                                  BOOT_CRC_Fail :1; //
	unsigned short                                  APP_CRC_Fail :1; //
	unsigned short                                  CAL_CRC_Fail :1; //
	unsigned short                                  A_D_FPGA_CAL_O_T :1; //
	unsigned short                                  A_D_FPGA_CAL_T_O :1; //
	unsigned short                                  FPGA_Init_RW_Fault :1; //
	unsigned short                                  FPGA_Version_Fault :1; //
	unsigned short                                  bit_9 :1; //
	unsigned short                                  bit_10 :1; //
	unsigned short                                  bit_11 :1; //
	unsigned short                                  bit_12 :1; //
	unsigned short                                  bit_13 :1; //
	unsigned short                                  SW_HW_Missmatch :1; //
	unsigned short                                  bit_15 :1; //
} BIT_Result_4_BDT;
#endif /* _TYPE_BIT_Result_4_BDT*/


/* Warning_BIT_Result_5_BDT */
#ifndef _TYPE_Warning_BIT_Result_5_BDT
#define _TYPE_Warning_BIT_Result_5_BDT
typedef struct {
    unsigned short                                  G_X_SLD_Deg :1; //
    unsigned short                                  G_Y_SLD_Deg :1; //
    unsigned short                                  G_Z_SLD_Deg :1; //
    unsigned short                                  GYRO_Temp_Diff :1; //
    unsigned short                                  ACC_Temp_Diff :1; //
    unsigned short                                  G_X_Saturation :1; //
    unsigned short                                  A_Y_Saturation :1; //
    unsigned short                                  A_Z_Saturation :1; //
    unsigned short                                  PCB_Temp :1; //
    unsigned short                                  bit_12 :1; //
    unsigned short                                  One_GYRO_Temp_Limit :1; //
    unsigned short                                  One_ACC_Temp_Limit :1; //
    unsigned short                                  bit_15 :1; //
    unsigned short                                  G_Y_Saturation :1; //
    unsigned short                                  G_Z_Saturation :1; //
    unsigned short                                  A_X_Saturation :1; //
} Warning_BIT_Result_5_BDT;
#endif /* _TYPE_Warning_BIT_Result_5_BDT*/


/* SW_Vewrion_Date_BDT */
#ifndef _TYPE_SW_Vewrion_Date_BDT
#define _TYPE_SW_Vewrion_Date_BDT
typedef struct {
	unsigned short                                  Year :8; //
	unsigned short                                  Month :8; //
} SW_Version_Date_BDT;
#endif /* _TYPE_SW_Vewrion_Date_BDT*/

/* Part_Number_BDT */
#ifndef _TYPE_Part_Number_BDT
#define _TYPE_Part_Number_BDT
typedef struct {
	unsigned short                                  Suffix :10; //
	unsigned short                                  prefix :6; //
} Part_Number_BDT;
#endif /* _TYPE_Part_Number_BDT*/


/* MUX_Data_UNION */
#ifndef _TYPE_MUX_Data_UNION
#define _TYPE_MUX_Data_UNION
typedef union {
    short                                           GYRO_X_Temperature; //
    short                                           GYRO_Y_Temperature; //
    short                                           GYRO_Z_Temperature; //
    short                                           GYRO_X_SLD; //
    short                                           GYRO_Y_SLD; //
    short                                           GYRO_Z_SLD; //
    short                                           GYRO_NEG_Supply_Vdc; //
    short                                           GYRO_POS_Supply_Vdc; //
    short                                           ACC_X_Temperature; //
    short                                           ACC_Y_Temperature; //
    short                                           ACC_Z_Temperature; //
    short                                           POS_15V_Internal_Vdc; //
    short                                           Input_Supply_Vdc; //
    short                                           Digital_3_3V_P_Supply; //
    short                                           Analog_3_5V_P_Supply; //
    short                                           A_D_External_Ref; //
    short                                           A_D_Temperature; //
    short                                           A_D_VCC_P2_5V_N2_5V; //
    short                                           PCB_Temperature; //
    unsigned short                                  ETI; //
    BIT_Result_1_BDT                                BIT_Result_1; //
    BIT_Result_2_BDT                                BIT_Result_2; //
    BIT_Result_3_BDT                                BIT_Result_3; //
    BIT_Result_4_BDT                                BIT_Result_4; //
    Warning_BIT_Result_5_BDT                        Warning_BIT_Result_5; //
    unsigned char                                   System_Serial_No_LS; //
    unsigned char                                   System_Serial_No_MS; //
    unsigned short                                  SW_Version_LS_No; //
    unsigned short                                  SW_Version_Mid_No; //
    unsigned short                                  SW_Version_MS_No; //
    SW_Version_Date_BDT                             SW_Vewrion_Date; //
    Part_Number_BDT                                 Part_Number; //
} MUX_Data_UNION;
#endif /* _TYPE_MUX_Data_UNION*/

/* IMU_MGLT_Msg */
#ifndef _MSG_TYPE_IMU_MGLT_MSG
#define _MSG_TYPE_IMU_MGLT_MSG
typedef struct {
    unsigned short                                  Header; //
    short                                           ACC_X; //
    short                                           ACC_Y; //
    short                                           ACC_Z; //
    short                                           W_X; //
    short                                           W_Y; //
    short                                           W_Z; //
    unsigned char                                   A_X_Delta_Vel[3]; //
    unsigned char                                   A_Y_Delta_Vel[3]; //
    unsigned char                                   A_Z_Delta_Vel[3]; //
    unsigned char                                   G_X_Delta_Ang[3]; //
    unsigned char                                   G_Y_Delta_Ang[3]; //
    unsigned char                                   G_Z_Delta_Ang[3]; //
    BIT_Status_BDT                                  BIT_Status; //
    MUX_Status_BDT                                  MUX_Status; //
    MUX_Data_UNION                                  MUX_Data; //
    unsigned short                                  Checksum; //
} IMU_MGLT_Msg;
#endif /* _MSG_TYPE_IMU_MGLT_MSG */


/* MUX_Data_DT */
#ifndef _TYPE_MUX_Data_DT
#define _TYPE_MUX_Data_DT
typedef struct {
	short                                           GYRO_X_Temperature; //
	short                                           GYRO_Y_Temperature; //
	short                                           GYRO_Z_Temperature; //
	short                                           GYRO_X_SLD; //
	short                                           GYRO_Y_SLD; //
	short                                           GYRO_Z_SLD; //
	short                                           GYRO_POS_Supply_Vdc; //
	short                                           GYRO_NEG_Supply_Vdc; //
	short                                           ACC_X_Temperature; //
	short                                           ACC_Y_Temperature; //
	short                                           ACC_Z_Temperature; //
	short                                           POS_15V_Internal_Vdc; //
	short                                           Input_Supply_Vdc; //
	short                                           Digital_3_3V_P_Supply; //
	short                                           Analog_3_5V_P_Supply; //
	short                                           A_D_External_Ref; //
	short                                           A_D_Temperature; //
	short                                           A_D_VCC_P2_5V_N2_5V; //
	short                                           PCB_Temperature; //
	unsigned short                                  ETI; //
	BIT_Result_1_BDT                                BIT_Result_1; //
	BIT_Result_2_BDT                                BIT_Result_2; //
	BIT_Result_3_BDT                                BIT_Result_3; //
	BIT_Result_4_BDT                                BIT_Result_4; //
	Warning_BIT_Result_5_BDT                        Warning_BIT_Result_5; //
	unsigned short                                  System_Serial_No_LS; //
	unsigned short                                  System_Serial_No_MS; //
	unsigned short                                  SW_Version_LS_No; //
	unsigned short                                  SW_Version_Mid_No; //
	unsigned short                                  SW_Version_MS_No; //
	SW_Version_Date_BDT                             SW_Vewrion_Date; //
	Part_Number_BDT                                 Part_Number; //
} MUX_Data_DT;
#endif /* _TYPE_MUX_Data_DT*/


#pragma pack(pop)

#ifdef __cplusplus
}
#endif

#endif
