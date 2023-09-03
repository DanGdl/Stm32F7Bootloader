/*
 * ADIS16547.h
 *
 *  Created on: 16 Feb 2022
 *      Author: itzhaki
 */

#ifndef INC_ADIS16547_H_
#define INC_ADIS16547_H_

#include <IMU-TRF-90M.h>
#include <RTG_main.h>
#include <sys/_stdint.h>
#include <CIB_Protocol.h>

/*********************************** IMU 16547 ******************************/
#define IMU_ADIS16547_DEVICE_ID 16547

#define  IMU_READ  			(uint16_t)0x0000
#define  IMU_WRITE  		(uint16_t)0x8000
#define  IMU_PAGE_ID_ADD	(uint16_t)0x0000

#define  IMU_FNCTION_CTRL_1	(uint16_t)0x0006
#define  IMU_FNCTION_CTRL_2	(uint16_t)0x0007

#define  IMU_SYNC_SCALE_1	(uint16_t)0x0010
#define  IMU_SYNC_SCALE_2	(uint16_t)0x0011

#define  IMU_SYNC_SCALE_D_1	(uint16_t)0x00A0
#define  IMU_SYNC_SCALE_D_2	(uint16_t)0x000F

#define  IMU_FNCTION_C_D_1	(uint16_t)0x00CD
#define  IMU_FNCTION_C_D_2	(uint16_t)0x0001

#define  IMU_PAGE_ID_0 		(uint16_t)0x0000
#define  IMU_PAGE_ID_3 		(uint16_t)0x0003
#define  IMU_PAGE_ID_4 		(uint16_t)0x0004
#define  IMU_SYS_E_FLAG 	(uint16_t)0x0008
#define  IMU_DIAG_STS 		(uint16_t)0x000A
#define  IMU_TEMP_OUT 		(uint16_t)0x000E
#define  IMU_PROD_ID 		(uint16_t)0x007E
#define  IMU_FIRM_REV 		(uint16_t)0x0078
#define  IMU_FIRM_DM 		(uint16_t)0x007A
#define  IMU_FIRM_Y 		(uint16_t)0x007C
#define  IMU_BOOT_REV 		(uint16_t)0x007E
#define  IMU_SERIAL_NUM 	(uint16_t)0x0020
#define  IMU_NULL 			(uint16_t)0x0000

#define  X_DELTANG_LOW 		(uint16_t)0x0040
#define  X_DELTANG_OUT 		(uint16_t)0x0042
#define  Y_DELTANG_LOW 		(uint16_t)0x0044
#define  Y_DELTANG_OUT 		(uint16_t)0x0046
#define  Z_DELTANG_LOW 		(uint16_t)0x0048
#define  Z_DELTANG_OUT 		(uint16_t)0x004A
#define  X_DELTVEL_LOW 		(uint16_t)0x004C
#define  X_DELTVEL_OUT 		(uint16_t)0x004E
#define  Y_DELTVEL_LOW 		(uint16_t)0x0050
#define  Y_DELTVEL_OUT 		(uint16_t)0x0052
#define  Z_DELTVEL_LOW 		(uint16_t)0x0054
#define  Z_DELTVEL_OUT 		(uint16_t)0x0056
#define  IMU_TIME_STAMP		(uint16_t)0x0028

//DATA COUNTER in page 0
#define  IMU_DATA_CNT_L		(uint16_t)0x0004
#define  IMU_DATA_CNT_H		(uint16_t)0x0005

#define  IMU_GLOB_CMD_1 	(uint16_t)0x0002
#define  IMU_GLOB_CMD_2 	(uint16_t)0x0003
#define  IMU_SELF_TEST 		(uint16_t)0x0002
#define  numImuOfCmd  		(uint8_t)13

#define  IMU_DECIMATION_CMD_1 	(uint16_t)0x000C
#define  IMU_DECIMATION_CMD_2 	(uint16_t)0x000D

#define  IMU_DECIMATION_400Hz 	(uint16_t)0x0009

#define IMU_CLOCK_SAMPLE 250

/*********************************** IMU 16467 ******************************/
#define IMU_ADIS16467_DEVICE_ID 16467

#define  IMU_ADIS16467__DIAG_STS 			(uint16_t)0x0002
#define  IMU_ADIS16467__TEMP_OUT 			(uint16_t)0x001C
#define  IMU_ADIS16467__PROD_ID 			(uint16_t)0x0072
#define  IMU_ADIS16467__FIRM_REV 			(uint16_t)0x006C
#define  IMU_ADIS16467__FIRM_DM 			(uint16_t)0x006E
#define  IMU_ADIS16467__FIRM_Y 				(uint16_t)0x0070
#define  IMU_ADIS16467__SERIAL_NUM 			(uint16_t)0x0074
#define  IMU_ADIS16467__NULL 				(uint16_t)0x0000

#define  IMU_ADIS16467_X_DELTANG_LOW 		(uint16_t)0x0024
#define  IMU_ADIS16467_X_DELTANG_OUT 		(uint16_t)0x0026
#define  IMU_ADIS16467_Y_DELTANG_LOW 		(uint16_t)0x0028
#define  IMU_ADIS16467_Y_DELTANG_OUT 		(uint16_t)0x002A
#define  IMU_ADIS16467_Z_DELTANG_LOW 		(uint16_t)0x002C
#define  IMU_ADIS16467_Z_DELTANG_OUT 		(uint16_t)0x002E
#define  IMU_ADIS16467_X_DELTVEL_LOW 		(uint16_t)0x0030
#define  IMU_ADIS16467_X_DELTVEL_OUT 		(uint16_t)0x0032
#define  IMU_ADIS16467_Y_DELTVEL_LOW 		(uint16_t)0x0034
#define  IMU_ADIS16467_Y_DELTVEL_OUT 		(uint16_t)0x0036
#define  IMU_ADIS16467_Z_DELTVEL_LOW 		(uint16_t)0x0038
#define  IMU_ADIS16467_Z_DELTVEL_OUT 		(uint16_t)0x003A

#define  IMU_ADIS16467__GLOB_CMD_1 			(uint16_t)0x0068
#define  IMU_ADIS16467__GLOB_CMD_2 			(uint16_t)0x0069
#define  IMU_ADIS16467__SELF_TEST 			(uint16_t)0x0008

#define  IMU_ADIS16467_DECIMATION_CMD_1 	(uint16_t)0x0064
#define  IMU_ADIS16467_DECIMATION_CMD_2 	(uint16_t)0x0065

#define  IMU_ADIS16467_MSC_CTL_1 			(uint16_t)0x0060
#define  IMU_ADIS16467_MSC_CTL_2 			(uint16_t)0x0061

#define  IMU_ADIS16467_UP_SCALE_1 			(uint16_t)0x0062
#define  IMU_ADIS16467_UP_SCALE_2 			(uint16_t)0x0063

#define  IMU_ADIS16467_MSC_CTL_D_1 			(uint16_t)0x00CB
#define  IMU_ADIS16467_MSC_CTL_D_2 			(uint16_t)0x0000

#define  IMU_ADIS16467_UP_SCALE_D_1 		(uint16_t)0x00D0
#define  IMU_ADIS16467_UP_SCALE_D_2 		(uint16_t)0x0007

#define  IMU_ADIS16467_DECIMATION_400Hz 	(uint16_t)0x0004

#pragma pack(push,1)

//typedef struct
//{
//	uint32_t TimeTag;
//	uint16_t DIAG_STS;
//	uint16_t TEMP_OUT;
//	uint16_t PROD_ID;
//	uint16_t FIRM_REV;
//	uint16_t FIRM_DM;
//	uint16_t FIRM_Y;
//	uint16_t SERIAL_NUM;
//} ADIS16467_VALUES_t;

//typedef struct
//{
//	uint32_t TimeTag;
//	uint16_t SYS_E_FLAG;
//	uint16_t DIAG_STS;
//	uint16_t TEMP_OUT;
//	uint16_t PROD_ID;
//	uint16_t FIRM_REV;
//	uint16_t FIRM_DM;
//	uint16_t FIRM_Y;
//	uint16_t BOOT_REV;
//	uint16_t SERIAL_NUM;
//} ADIS16547_VALUES_t;

/**************************************************************************************************************/


typedef struct
{
	uint16_t in;
	uint16_t out;
} CIB_Discretes_Data_t;



#pragma pack(pop)


extern uint16_t IMU_ADIS16547_CMD_Buffer[20];
extern uint16_t IMU_ADIS16547_CMD_Buffer_size;
extern uint16_t IMU_ADIS16467_CMD_Buffer[20];
extern uint16_t IMU_ADIS16467_CMD_Buffer_size;

extern CIB_400_Hz_Msg CIB_400_Hz;

/*********************************** IMU ******************************/
// Prepare the IMU ADIS16547 command
// R_W = IMU_READ for read ,R_W = IMU_WRITE for write
// return uint16_t
#define RTG_IMU_CMD(R_W, ADD , DATA) ( ( ( (uint16_t)ADD<<8 )  | (uint16_t)R_W ) | (uint8_t)DATA )


/*********************************** IMU 16547 ******************************/

void RTG_IMU_ADIS16547_INIT(void);
void RTG_IMU_ADIS16547_DIAG_STS(void);

uint16_t RTG_IMU_ADIS16547_PBIT(CIB_PBIT_IMU_ADIS16547 *mes); // call from RTG_IMU_ADIS16547_Gyro_Accalrtion
uint16_t RTG_IMU_ADIS16547_CBIT(CIB_CBIT_IMU_ADIS16547 *mes); // call from RTG_IMU_ADIS16547_Gyro_Accalrtion

void RTG_IMU_ADIS16547_CMD_Buffer_init();
void RTG_IMU_ADIS16547_Gyro_Accalrtion(void);

/*********************************** IMU 16467 ******************************/

void RTG_IMU_ADIS16467_INIT(void);
void RTG_IMU_ADIS16467_DIAG_STS(void);

uint16_t RTG_IMU_ADIS16467_PBIT(CIB_PBIT_IMU_ADIS16467 *mes); // call from RTG_IMU_ADIS16467_Gyro_Accalrtion
uint16_t RTG_IMU_ADIS16467_CBIT(CIB_CBIT_IMU_ADIS16467 *mes); // call from RTG_IMU_ADIS16467_Gyro_Accalrtion

void RTG_IMU_ADIS16467_CMD_Buffer_init();
void RTG_IMU_ADIS16467_Gyro_Accalrtion(void);



#endif /* INC_ADIS16547_H_ */
