/*
 * discrete.h
 *
 *  Created on: Nov 23, 2021
 *      Author: itzhaki
 *
  */

#ifndef SRC_DISCRETE_H_
#define SRC_DISCRETE_H_

#include <sys/_stdint.h>
#include <CIB_Protocol.h>

// Output Discretes Bit Offset
/*<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< OUTPUT >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/
#define GPIO_OUT_TVC_DSCNNCTD     	GPIOG, GPIO_PIN_6    	// GPIO_MODE_OUTPUT_PP pin_G_6 Future Use
#define GPIO_OUT_FUSE_1_IGNITION  	GPIOG, GPIO_PIN_3    	// GPIO_MODE_OUTPUT_PP pin_G_3:
#define GPIO_OUT_FUSE_2_IGNITION  	GPIOI, GPIO_PIN_14    	// GPIO_MODE_OUTPUT_PP pin_I_14 Future Use
#define GPIO_OUT_FUSE_3_IGNITION  	GPIOK, GPIO_PIN_2    	// GPIO_MODE_OUTPUT_PP GPIO_PULLDOWN
#define GPIO_OUT_RST_IMU_1_OUTPUT 	GPIOD, GPIO_PIN_11    	// GPIO_MODE_OUTPUT_PP pin_D_11
#define GPIO_OUT_RST_IMU_2_OUTPUT 	GPIOJ, GPIO_PIN_6  		// GPIO_MODE_OUTPUT_OD pin_J_6
#define GPIO_OUT_Ext_Com_Dis 		GPIOI, GPIO_PIN_15  	// GPIO_MODE_OUTPUT_OD pin_J_6
#define GPIO_OUT_WCS_END_COM 		GPIOI, GPIO_PIN_15  	// GPIO_MODE_OUTPUT_OD pin_A_11
// Not Used
#define GPIO_OUT_FTS2_Batt_Ign    	GPIOH, GPIO_PIN_2    	// GPIO_MODE_OUTPUT_PP pin_H_2:

//#define GPIO_OUT_Gui_En_2_FTS2    	GPIOA, GPIO_PIN_4    	// ???? pin_A_4: - Currently NOT used
//#define GPIO_OUT_MLL_IND_OUTPUT   	GPIOK, GPIO_PIN_4    	// GPIO_MODE_INPUT pin_K_4: - Error - this pin should be used as input! for MLL
//#define GPIO_OUT_SPI3_CS_0        	GPIOC, GPIO_PIN_14   	// GPIO_MODE_OUTPUT_OD pin_C_14
//#define GPIO_OUT_SPI3_CS_1        	GPIOJ, GPIO_PIN_15    	// GPIO_MODE_OUTPUT_OD pin_J_15


/*<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< INPUT >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/
#define GPIO_IN_SQUIB_BAT_FTS   	GPIOA, GPIO_PIN_8       // GPIO_MODE_INPUT pin_A_8  - Currently NOT used
#define GPIO_IN_PTL_SMPL        	GPIOK, GPIO_PIN_6       // GPIO_MODE_INPUT pin_K_6
#define GPIO_IN_MPE_2_IND       	GPIOG, GPIO_PIN_14      // GPIO_MODE_INPUT pin_G_14
#define GPIO_IN_SnA_ENA_SMPL    	GPIOK, GPIO_PIN_7       // GPIO_MODE_INPUT pin_K_7
#define GPIO_IN_MLL_IND_INPUT   	GPIOK, GPIO_PIN_4       // GPIO_MODE_INPUT pin_K_4
#define GPIO_IN_PPS_FPGA_2_CPU  	GPIOA, GPIO_PIN_0      	// GPIO_MODE_INPUT pin_A_0
#define GPIO_IN_CPU2FPGA_GPIO2  	GPIOE, GPIO_PIN_5       // GPIO_MODE_OUTPUT_PP pin_E_5
#define GPIO_IN_CPU2FPGA_GPIO3  	GPIOE, GPIO_PIN_6       // GPIO_MODE_OUTPUT_PP pin_E_6
#define GPIO_IN_CPU2FPGA_GPIO4  	GPIOE, GPIO_PIN_2       // GPIO_MODE_OUTPUT_PP pin_E_2
#define GPIO_IN_CPU2FPGA_GPIO1  	GPIOI, GPIO_PIN_12      // GPIO_MODE_OUTPUT_PP pin_I_12
#define GPIO_IN_DIO1_IMU2_SYNC  	GPIOJ, GPIO_PIN_5      	// GPIO_MODE_OUTPUT_PP pin_J_5
#define GPIO_IN_DIO2_IMU2_DTRDY 	GPIOJ, GPIO_PIN_7      	// GPIO_MODE_IT_RISING pin_J_7
#define GPIO_IN_DIO2_IMU1_DTRDY 	GPIOB, GPIO_PIN_14    	// GPIO_MODE_IT_RISING pin_B_14
#define GPIO_IN_DIO1_IMU1_SYNC  	GPIOB, GPIO_PIN_15     	// GPIO_MODE_OUTPUT_PP pin_B_15

//#define GPIO_IN_RST_IMU_2_INPUT 	GPIOJ, GPIO_PIN_6      	// GPIO_MODE_OUTPUT_PP pin_J_6  - - Error Currently NOT used
//#define GPIO_IN_RST_IMU_1_INPUT 	GPIOD, GPIO_PIN_11      // GPIO_MODE_OUTPUT_PP pin_D_11 - Currently NOT used

/*<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< INPUT >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/
//typedef enum
//{
//	DISC_IN_SQUIB_BAT_FTS = 	0,       // pin_A_8  - Currently NOT used
//	DISC_IN_PTL_SMPL = 			1,       // pin_K_6
//	DISC_IN_MPE_2_IND = 		2,       // pin_G_14
//	DISC_IN_SnA_ENA_SMPL = 		3,       // pin_K_7
//	DISC_IN_MLL_IND_INPUT = 	4,       // pin_K_4
//	DISC_IN_PPS_FPGA_2_CPU = 	5,       // pin_A_0
//	DISC_IN_CPU2FPGA_GPIO2 = 	6,       // pin_E_5
//	DISC_IN_CPU2FPGA_GPIO3 = 	7,       // pin_E_6
//	DISC_IN_CPU2FPGA_GPIO4 = 	8,       // pin_E_2
//	DISC_IN_CPU2FPGA_GPIO1 = 	9,       // pin_I_12
//	DISC_IN_DIO1_IMU2_SYNC = 	10,      // pin_J_5
//	DISC_IN_DIO2_IMU2_DTRDY = 	11,      // pin_J_7
//	DISC_IN_DIO2_IMU1_DTRDY = 	12,      // pin_B_14
//	DISC_IN_DIO1_IMU1_SYNC = 	13,      // pin_B_15
//
////	DISC_IN_RST_IMU_2_INPUT = 14,
////	DISC_IN_RST_IMU_1_INPUT = 15
//} CIB_INPUT_DISCRETES;
/*<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< INPUT >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/

#define GPIO_LED14 	GPIOJ, GPIO_PIN_14  		// GPIO_MODE_OUTPUT_OD
#define GPIO_LED12 	GPIOJ, GPIO_PIN_12  		// GPIO_MODE_OUTPUT_OD




#define SET_VALUE( VAR ,BIT, VAL ) if (VAL) VAR = VAR | (1<<BIT); else VAR = VAR & ~(1U<<BIT);
//#define GET_DISC(VAR,BIT) ( ( VAR & (1U<<BIT) )? 1:0 )
//#define RTG_GPIO_WritePin(GPIO,VAL)	HAL_GPIO_WritePin(GPIO, VAL);

#define IN_DISC_SIZE sizeof(uint16_t)
#define OUT_DISC_SIZE sizeof(uint16_t)

#define DISCRETR_COMPEAR_OUTPUT  (uint16_t)0x3A0
#define DISCRETR_COMPEAR_INPUT   (uint16_t)0xEF

#define DISCREAT_PASS_FAIL_CNT(DISCREAT_CNT , STRING , BUF) ((DISCREAT_CNT > 8) ? \
		(sprintf(BUF, STRING, DISCREAT_CNT, "PASS")) : (sprintf(BUF, STRING, DISCREAT_CNT, "FAIL")));

union FpgaVer
{
	uint8_t disceretReg;
	struct
	{
		uint16_t pin_E_6 :1;
		uint16_t pin_E_5 :1;
		uint16_t pin_I_12 :1;
	};
};

#pragma pack (push,1)

//________________________________________________
// CIB Command requests
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//typedef enum {
//	CIB_OUTPUT_DISCRETE_CLEAR  = 0,
//	CIB_OUTPUT_DISCRETE_ACTIVE = 1,
//	CIB_OUTPUT_DISCRETES_READ  = 2,
//	CIB_INPUT_DISCRETES_READ   = 3,
//	CIB_PBIT_REQUEST           = 4,
//	CIB_IMU_PRIME_MARGALIT     = 5,
//	CIB_IMU_PRIME_ADIS         = 6,
//	CIB_TOD			           = 7
//} CIB_COMMAND;

//typedef struct {
//	uint8_t cmd :4;
//	uint8_t discrete :4;
//	uint32_t year;
//	uint32_t month;
//	uint32_t day;
//	uint32_t sec;
//} CIB_Cmd_Data;
/////////////////////////////////////////////////////////

typedef struct
{
	uint32_t TimeTag;
	uint16_t IN_Disc_Size;
	uint16_t OUT_Disc_Size;
	uint8_t Error;
} BIT_Discrete_t;

//  Command from UDP to PORT UDP_DISCRETE_PORT_IN
// byte1 opcode, byte2 bit, byte3 value
#define DISCRITE_OPCODE_WRITE_OUT	1
#define DISCRITE_OPCODE_READ_OUT	2
#define DISCRITE_OPCODE_READ_IN		3
#define DISCRITE_OPCODE_PBIT		4

typedef struct {
	CIB_Header 	header;
	uint8_t 	opcode;
	uint16_t 	data;
}DicreteToUdp_t;

#pragma pack ( pop )

void RTG_DISCRETE_INT(void);
void RTG_UDP_Discrete(uint8_t *receive);

void RTG_writeOutputDiscrete(uint8_t bit,uint16_t value);

uint16_t RTG_readOutputDiscrete(void);
uint16_t RTG_readInputDiscrete(void);

uint8_t RTG_Fpga_Rev(void);
void RTG_Discrete_bit(BIT_Discrete_t *mes);

#endif /* SRC_DISCRETE_H_ */
