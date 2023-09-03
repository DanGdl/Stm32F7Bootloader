/*
 * Battery.h
 *
 *  Created on: Feb 27, 2023
 *      Author: e_tas
 *
 *      The bq78350-R1 has a configurable addressing scheme
 *      that can be enabled or this feature can be
 *      disabled resulting in
 *      the slave address being fixed as 0x16/0x17.
 *
 *      Care should be taken in the setting of
 *      Addr Reads as the bq78350-R1 will only respond to address
 *      0x16/0x17 until at least Addr Read × 32 ms after POR.
 *      page 91
 */

#ifndef INC_BATTERY_H_
#define INC_BATTERY_H_

#define BATTERY_ADDRESS 			0x08
#define BATTERY_ADDRESS_W 			0x16
#define BATTERY_ADDRESS_R 			0x17

// SBS Commands Summary:: bytes 2
#define SBS_CMD_Temperature			0x08	// 0.1K float
#define SBS_CMD_Voltage				0x09	// mV unsigned
#define SBS_CMD_Current				0x0A	// 10 mA signed
#define SBS_CMD_AverageCurrent		0x0B	// 10 mA signed
#define SBS_CMD_AbsoluteStateOfCharge 0x0E //exept bytes1(0..100)
#define SBS_CMD_RemainingCapacity 	0x0F  // R/W U2
/*
 * If BatteryMode()[CAPM] = 0, then the data reports in 10 mAh.
If BatteryMode()[CAPM] = 1, then the data reports in 100 mWh.
 */
#define SBS_CMD_FullChargeCapacity 	0x10 	// U2
/*
 * If BatteryMode()[CAPM] = 0, then the data reports in 10 mAh.
If BatteryMode()[CAPM] = 1, then the data reports in 100 mWh.
 */
#define SBS_CMD_CycleCount		 	0x17	// U2 cycles
#define SBS_CMD_DesignCapacity	 	0x18	// U2
/*
 * If BatteryMode()[CAPM] = 0, then the data reports in  mAh.
If BatteryMode()[CAPM] = 1, then the data reports in 10 mWh.
 */
#define SBS_CMD_DesignVoltage	 	0x19 	// U2 mV

#define SBS_CMD_BatteryStatus	 	0x16

#define SBS_Active			1
#define SBS_Inactive		0
#define SBS_DISCHARGE		1
#define SBS_CHARGE			0
#define SBS_FULL_CHARGE		1
#define SBS_NOT_FULL_CHARGE	0
#define SBS_FULL_DISCHARGE	1
#define SBS_BATT_OK			0

#define SBS_ERR_OK				0x0
#define SBS_ERR_BUSY			0x1
#define SBS_ERR_RESERVED		0x2
#define SBS_ERR_Unsupported		0x3
#define SBS_ERR_AccessDenied	0x4
#define SBS_ERR_Overflow		0x5
#define SBS_ERR_BadSize			0x6
#define SBS_ERR_Unknown			0x7

//typedef struct BatteryStatus {
//	uint16_t EC3:4;		//EC3:0 (Bit 3–0): Error Code
//	uint16_t FD:1;		//FD (Bit 4): Fully Discharged
//	uint16_t FC:1;		//FC (Bit 5): Fully Charged
//	uint16_t DSG:1;		//DSG (Bit 6): Charge FET Test
//	uint16_t INIT:1;	//INIT (Bit 7): Initialization
//	uint16_t RTA:1;		//RTA (Bit 8): Remaining Time Alarm
//	uint16_t RCA:1;		//RCA (Bit 9): Remaining Capacity Alarm
//	uint16_t RSVD0:1;	//RSVD (Bit 10): Reserved
//	uint16_t TDA:1;		//TDA (Bit 11): Terminate Discharge Alarm
//	uint16_t OTA:1;		//OTA (Bit 12): Overtemperature Alarm
//	uint16_t RSVD1:1;	//RSVD (Bit 13): Reserved
//	uint16_t TCA:1;		//TCA (Bit 14): Terminate Charge Alarm
//	uint16_t OCA:1; 	//OCA (Bit 15): Overcharged Alarm
//}BatteryStatus_t;

typedef struct {
	uint32_t TIME_TAG;
	float 		Temperature;	// 0.1K
	uint16_t 	Voltage;		// mV
	short	 	Current;		// 10 mA
	short	 	AverageCurrent;	// 10 mA
	uint8_t  	AbsoluteStateOfCharge; // %
	uint16_t 	RemainingCapacity;  // in 10 mAh or 100 mWh
	uint16_t 	FullChargeCapacity;	// in 10 mAh or 100 mWh
	uint16_t 	CycleCount	; 		// cycles
	uint16_t 	DesignCapacity;		// in  mAh or 10 mWh
	uint16_t 	DesignVoltage; 		// mV
	BatteryStatus_t 	BatteryStatus;
}BattaryData_t;

uint8_t 		BattRead16(uint8_t reg,uint16_t *data);
uint8_t  		BattRead8(uint8_t reg,uint8_t *data);
uint8_t 		BattReadAll(CIB_CBIT_BMS *rez_data);

#endif /* INC_BATTERY_H_ */
