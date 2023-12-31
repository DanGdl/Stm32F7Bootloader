/*
 * FPGA_Version.h
 *
 *  Created on: 16 במרץ 2023
 *      Author: 95799
 */

#ifndef INC_FPGA_VERSION_H_
#define INC_FPGA_VERSION_H_


#define FPGA_VERSION_REG  	0x0000000
#define FPGA_DATE_REG  		0x0000001
#define FPGA_DATE_REG2  	0x0000002
#define FPGA_SYS_SYN_REG	0x000000A

#define FPGA_VER_TYPE_OP			0x01
#define FPGA_VER_TYPE_ATP			0x10
#define FPGA_VER_ID_MAIN			0x01
#define FPGA_VER_ID_MAIN_TO_CLONE	0x10
#define FPGA_VER_ID_CLONE			0x11
#define FPGA_VER_ID_RESERVED		0x00

#define FPGA_VER_OP					0x1910
#define FPGA_VER_ATP				0x1135
/*
 * When '0' then DIO2_IMU1_DTRDY_CPU2FPGA    <= DIO1_IMU1_DTRDY_FPGA2IMU;
When '1' then DIO2_IMU1_DTRDY_CPU2FPGA
<= RTC_MRGLT2CPUandFPGA;
 * */
#define DIO1_IMU1_DTRDY_FPGA2IMU	0
#define RTC_MRGLT2CPUandFPGA		1


#endif /* INC_FPGA_VERSION_H_ */
