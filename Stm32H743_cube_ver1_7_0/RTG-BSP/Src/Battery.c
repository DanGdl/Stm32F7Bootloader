/*
 * Battery.c
 *
 *  Created on: Feb 27, 2023
 *      Author: e_tas
 */

#include "main.h"

//#include "i2c.h"
#include <string.h>
#include "timer.h"
#include "Battery.h"
extern SMBUS_HandleTypeDef hsmbus4;
int SmbusTxFlag;
int SmbusRxFlag;
int SmbusErFlag;


#define BAT_DELAY RT_DELAY_MS(1)

uint8_t BattReadAll(CIB_CBIT_BMS *rez_data){
	uint16_t get_data = 0;
	float float_data;
	uint8_t stat=0;
	rez_data->TIME_TAG = GetTimeTag;
	stat |= BattRead16(SBS_CMD_Temperature,&get_data);	// 0.1K
	if(!stat){
		float_data =  ( (float)(get_data*0.1) );
		rez_data->Temperature = float_data;	// 0.1K
	}
	stat |= BattRead16(SBS_CMD_Voltage, &rez_data->Voltage);		// mV
	BAT_DELAY;
	stat |= BattRead16(SBS_CMD_Current, (uint16_t*)&rez_data->Current);		// 10 mA
	BAT_DELAY;
	stat |= BattRead16(SBS_CMD_AverageCurrent,(uint16_t*) &rez_data->AverageCurrent);	// 10 mA
	BAT_DELAY;
	stat |= BattRead8(SBS_CMD_AbsoluteStateOfCharge, &rez_data->AbsoluteStateOfCharge); // %
	BAT_DELAY;
	stat |= BattRead16(SBS_CMD_RemainingCapacity, &rez_data->RemainingCapacity);  // in 10 mAh or 100 mWh
	BAT_DELAY;
	stat |= BattRead16(SBS_CMD_FullChargeCapacity, &rez_data->FullChargeCapacity);	// in 10 mAh or 100 mWh
	BAT_DELAY;
	stat |= BattRead16(SBS_CMD_CycleCount, &rez_data->CycleCount); 		// cycles
	BAT_DELAY;
	stat |= BattRead16(SBS_CMD_DesignCapacity, &rez_data->DesignCapacity);		// in  mAh or 10 mWh
	BAT_DELAY;
	stat |= BattRead16(SBS_CMD_DesignVoltage, &rez_data->DesignVoltage ); 		// mV
	BAT_DELAY;
	stat |= BattRead16(SBS_CMD_BatteryStatus, (uint16_t*)&rez_data->BatteryStatus);
	return stat;
}

void HAL_SMBUS_MasterRxCpltCallback(SMBUS_HandleTypeDef *hsmbus)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hsmbus);
  SmbusRxFlag=1;

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_SMBUS_MasterRxCpltCallback() could be implemented in the user file
   */
}
void HAL_SMBUS_MasterTxCpltCallback(SMBUS_HandleTypeDef *hsmbus)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hsmbus);
  SmbusTxFlag = 1;
  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_SMBUS_MasterRxCpltCallback() could be implemented in the user file
   */
}
void HAL_SMBUS_AddrCallback(SMBUS_HandleTypeDef *hsmbus, uint8_t TransferDirection, uint16_t AddrMatchCode)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hsmbus);
  UNUSED(TransferDirection);
  UNUSED(AddrMatchCode);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_SMBUS_AddrCallback() could be implemented in the user file
   */
}
void HAL_SMBUS_ListenCpltCallback(SMBUS_HandleTypeDef *hsmbus)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hsmbus);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_SMBUS_ListenCpltCallback() could be implemented in the user file
   */
}

/**
  * @brief  SMBUS error callback.
  * @param  hsmbus Pointer to a SMBUS_HandleTypeDef structure that contains
  *                the configuration information for the specified SMBUS.
  * @retval None
  */
void HAL_SMBUS_ErrorCallback(SMBUS_HandleTypeDef *hsmbus)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hsmbus);
  SmbusErFlag=1;
  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_SMBUS_ErrorCallback() could be implemented in the user file
   */
}

/*
 * return true:0 false :1
 */
uint8_t BattRead16(uint8_t reg,uint16_t*data){
	uint8_t buff[5]={0xff,0xff,0xff};
	HAL_StatusTypeDef stat;
	// write register
//	RTG_I2c_Error(&sI2Cx);
	SmbusTxFlag=0;
	SmbusRxFlag=0;
	stat  = HAL_SMBUS_Master_Transmit_IT(&hsmbus4, BATTERY_ADDRESS_W, &reg, 1,I2C_FIRST_FRAME);
	if(stat!= HAL_OK)
		return 1;
	while(!SmbusTxFlag)
	{
		osDelay(1);
	}
//	osDelay(10);
	// read data
	stat = HAL_SMBUS_Master_Receive_IT(&hsmbus4,BATTERY_ADDRESS_R,buff,2,I2C_FIRST_AND_LAST_FRAME);
	if(stat!= HAL_OK) 
		return 1;
	while(!SmbusRxFlag)
	{
		osDelay(1);
	}
//	*data = ( ( (uint16_t)buff[0] )<<8 )+buff[1];
	*data = *( (uint16_t*)buff );
	if(*data==0xffff){
//		*data=0xaaaa;
	}
	return 0;
}

/*
 * return true:0 false :1
 */
uint8_t BattRead8(uint8_t reg,uint8_t *data){
	uint8_t buff[5]={0xff,0xff,0xff};
	HAL_StatusTypeDef stat;
	// write register
//	RTG_I2c_Error(&sI2Cx);
	SmbusTxFlag=0;
	SmbusRxFlag=0;
	stat  = HAL_SMBUS_Master_Transmit_IT(&hsmbus4, BATTERY_ADDRESS_W, &reg, 1,I2C_FIRST_FRAME);
	if(stat!= HAL_OK)
		return 1;
	while(!SmbusTxFlag)
	{
		osDelay(1);
	}
//	osDelay(10);
	// read data
	stat = HAL_SMBUS_Master_Receive_IT(&hsmbus4,BATTERY_ADDRESS_R,buff,1,I2C_FIRST_AND_LAST_FRAME);
	if(stat!= HAL_OK)
		return 1;
	while(!SmbusRxFlag)
	{
		osDelay(1);
	}
	*data = buff[0] ;
	if(*data==0xff){
		*data=0xaa;
	}
	return 0;
}

