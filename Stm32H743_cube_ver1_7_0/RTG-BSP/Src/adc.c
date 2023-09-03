/*
 * adc.c
 *
 *  Created on: 31 Oct 2021
 *      Author: Sasha
 */

#include <adc.h>
#include <bit.h>
#include <eth_rtg.h>
#include <math.h>
#include <RTG_main.h>
#include <stdio.h>
#include <stm32h7xx_hal_adc.h>
#include <stm32h7xx_hal_adc_ex.h>
#include <stm32h7xx_hal_def.h>
#include <string.h>
#include <timer.h>

extern UDP_t ADC_udp;

int8_t RTG_ADC_int(void) {
//	HAL_SYSCFG_EnableVREFBUF();
//	HAL_SYSCFG_VREFBUF_VoltageScalingConfig(SYSCFG_VREFBUF_VOLTAGE_SCALE0);

	if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED) != HAL_OK) return -1;

	if (HAL_ADCEx_Calibration_Start(&hadc3, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED) != HAL_OK) return -1;

	return 0;
}

HAL_StatusTypeDef RTG_Channel_Select(ADC_HandleTypeDef *hadc, uint32_t Channel) {
	ADC_ChannelConfTypeDef sConfig = { 0 };

	sConfig.Channel = Channel;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_32CYCLES_5;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;

	if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK) {
		return HAL_ERROR;
	}

	return HAL_OK;
}

static uint32_t RTG_ADC_GetValue(ADC_HandleTypeDef *hadc) {
	uint32_t res;

	HAL_ADC_Start(hadc);
	HAL_ADC_PollForConversion(hadc, HAL_MAX_DELAY);
	res = HAL_ADC_GetValue(hadc);
	HAL_ADC_Stop(hadc);

	return res;
}

float RTG_Get_Cpu_Temperture(void) {
	float CpuTemp = 0;
	uint32_t T1_30;
	uint32_t T2_110;
	uint32_t adcConVal;

	if (RTG_Channel_Select(&hadc3, ADC_CHANNEL_TEMPSENSOR) != HAL_OK) return -200;

	adcConVal = RTG_ADC_GetValue(&hadc3);

	T1_30 = (int32_t) * TEMP30_CAL_ADDR;
	T2_110 = (int32_t) * TEMP110_CAL_ADDR;

	// Normalize to 1.8V
	T1_30 /= NORMALIZE_3V_TO_1_8V;
	T2_110 /= NORMALIZE_3V_TO_1_8V;

	CpuTemp = (float) (110 - 30) / (float) (T2_110 - T1_30);

	CpuTemp *= (float) (adcConVal - T1_30);

	CpuTemp += 30;

	return CpuTemp;
}

uint32_t RTG_Get_Raw_Adc_Value(ADC_HandleTypeDef *hadc, uint32_t Channel) {
	uint32_t res;

	if (RTG_Channel_Select(hadc, Channel) != HAL_OK) return -1;

	res = RTG_ADC_GetValue(hadc);

	return res;
}

float RTG_Get_Adc_Voltage(ADC_HandleTypeDef *hadc, uint32_t Channel) {
	uint32_t res;
	float voltage = 0;

	float ADCVrefPlus = 1.8;
	float ADCFullScale = 65536;

	res = RTG_Get_Raw_Adc_Value(hadc, Channel);

	voltage = ((float) res * ADCVrefPlus) / ADCFullScale;

	return voltage;
}

// External Temperature Sensor LM20
float RTG_TemperatureSensorLM20(void) {
	double vout;
	double Temperature;

	vout = (double) RTG_Get_Adc_Voltage(&hadc3, ADC_CHANNEL_14);

	// Vin ADC for temperature divided by 3 with resistors
	vout = 3 * vout;

	// 3.88 * 10^(-6)
	Temperature = 3.88 * pow(10, -6);

	// (1.8639 - vout) /(3.88 * 10^(-6))
	Temperature = ((1.8639 - vout) / Temperature);

	// (1.8639 - vout) /(3.88 * 10^(-6)) + (2.1962 * 10^6)
	Temperature += 2.1962 * pow(10, 6);

	// square root
	Temperature = -1481.96 + sqrt(Temperature);

	return (float) Temperature;
}

/********************************************************************************************/
// ADC PASS/FAIL Results register
//    11      10        9         8
// --------------------------------------->
// | FPGA2 | FPGA1 | PS_1_2V | PS_1_0V |
// --------------------------------------->
//     7        6        5           4            3            2            1           0
//>---------------------------------------------------------------------------------------------
//   PS_5_0V | PS_3_3V | PS_1_8V(1) | PS_1_5V | PS_1_8V_ETH_PS | PS_1_1V | EXT TEMP | CPU TEMP |
//>---------------------------------------------------------------------------------------------
CIB_CBIT_ADC RTG_Chech_Boundaris(CIB_CBIT_ADC ADC_Res) {

	if ((ADC_Res.cpu_temp > 95) || (ADC_Res.cpu_temp < -30)) ADC_Res.errors_flags.cpu_temp = 1;
	else ADC_Res.errors_flags.cpu_temp = 0;
	if ((ADC_Res.ext_temp > 80) || (ADC_Res.ext_temp < -30)) ADC_Res.errors_flags.ext_temp = 1;
	else ADC_Res.errors_flags.ext_temp = 0;
	ADC_Res.errors_flags.ps_1_1v 			= CHECK_BOUNDARIS_OP(ADC_Res.ps_1_1v, 1.1);
	ADC_Res.errors_flags.ps_1_8v_eth_ps 	= CHECK_BOUNDARIS_OP(ADC_Res.ps_1_8v_eth_ps, 1.8);
	ADC_Res.errors_flags.ps_1_5v 			= CHECK_BOUNDARIS_OP(ADC_Res.ps_1_5v, 1.5);
	ADC_Res.errors_flags.ps_1_8v 			= CHECK_BOUNDARIS_OP(ADC_Res.ps_1_8v, 1.8);
	ADC_Res.errors_flags.ps_3_3v 			= CHECK_BOUNDARIS_OP(ADC_Res.ps_3_3v, 3.3);
	ADC_Res.errors_flags.ps_5_0v 			= CHECK_BOUNDARIS_OP(ADC_Res.ps_5_0v, 5.0);
	ADC_Res.errors_flags.ps_1_0v 			= CHECK_BOUNDARIS_OP(ADC_Res.ps_1_0v, 1.0);
	ADC_Res.errors_flags.ps_1_2v 			= CHECK_BOUNDARIS_OP(ADC_Res.ps_1_2v, 1.2);
	//Disable for now
//				ADC_Res.Passfail.FPGA1 = CHECK_BOUNDARIS_OP(ADC_Res.FPGA1, 0);
	//Disable for now
//				ADC_Res.Passfail.FPGA2 = CHECK_BOUNDARIS_OP(ADC_Res.FPGA2, 0);
	// check if there was a error
	CBIT_CURRENT_BUFF.current_bit.ADC = !(!(ADC_Res.errors_flags.Results));
	CBIT_CURRENT_BUFF.sticky_bit.ADC |= !(!(ADC_Res.errors_flags.Results));

	return ADC_Res;
}

CIB_CBIT_ADC RTG_adc(void) {
	CIB_CBIT_ADC ADC_Res = { 0 };

	//Internal Temperature Sensor
	ADC_Res.cpu_temp = RTG_Get_Cpu_Temperture();

	// External Temperature Sensor
	ADC_Res.ext_temp = RTG_TemperatureSensorLM20();
	// 1.1V PS
	ADC_Res.ps_1_1v = RTG_Get_Adc_Voltage(&hadc3, ADC_CHANNEL_7);
	// 1.8V ETH PS
	ADC_Res.ps_1_8v_eth_ps = 1.5 * RTG_Get_Adc_Voltage(&hadc3, ADC_CHANNEL_2);
	// 1.5V PS
	ADC_Res.ps_1_5v = RTG_Get_Adc_Voltage(&hadc3, ADC_CHANNEL_6);
	// 1.8V PS
	ADC_Res.ps_1_8v = 1.5 * RTG_Get_Adc_Voltage(&hadc3, ADC_CHANNEL_0);
	// 3.3V PS
	ADC_Res.ps_3_3v = 3 * RTG_Get_Adc_Voltage(&hadc1, ADC_CHANNEL_5);
	// 5.0V PS
	ADC_Res.ps_5_0v = 3 * RTG_Get_Adc_Voltage(&hadc1, ADC_CHANNEL_3);
	// 1.0V PS
	ADC_Res.ps_1_0v = RTG_Get_Adc_Voltage(&hadc1, ADC_CHANNEL_9);
	// 1.2V PS
	ADC_Res.ps_1_2v = RTG_Get_Adc_Voltage(&hadc3, ADC_CHANNEL_1);
	// FPGA1 ?
	ADC_Res.fpga1 = RTG_Get_Adc_Voltage(&hadc1, ADC_CHANNEL_0);
	// FPGA2 ?
	ADC_Res.fpga2 = RTG_Get_Adc_Voltage(&hadc1, ADC_CHANNEL_1);

	ADC_Res.time_tag = GetTimeTag;

	return (RTG_Chech_Boundaris(ADC_Res));
}
