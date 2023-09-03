/*
 * adc.h
 *
 *  Created on: 31 Oct 2021
 *      Author: Sasha
 */

#ifndef INC_ADC_H_
#define INC_ADC_H_

#include <sys/_stdint.h>
#include <stdlib.h>
#include <main.h>
#include <CIB_Protocol.h>

extern ADC_HandleTypeDef hadc3;
extern ADC_HandleTypeDef hadc1;
//
#define TEMP110_CAL_ADDR   ((uint16_t*)((uint32_t)0x1FF1E840))
#define TEMP30_CAL_ADDR      ((uint16_t*)((uint32_t)0x1FF1E820))

#define NORMALIZE_3V_TO_1_8V 0.555

#define LOW_LIMIT(x) (((float)x) * ((float)0.9))
#define HIGH_LIMIT(x) (((float)x) * ((float)1.1))

#define CHECK_BOUNDARIS_OP(result,value) \
		((((float)result) > (HIGH_LIMIT(value))) \
	    || (((float)result) < (LOW_LIMIT(value))))? 1:0

int8_t 				RTG_ADC_int(void);
CIB_CBIT_ADC 		RTG_adc(void);
uint32_t 			RTG_Get_Raw_Adc_Value(ADC_HandleTypeDef *hadc, uint32_t Channel);
float 				RTG_Get_Adc_Voltage(ADC_HandleTypeDef *hadc, uint32_t Channel);
float 				RTG_Get_Cpu_Temperture(void);
HAL_StatusTypeDef 	RTG_Channel_Select(ADC_HandleTypeDef *hadc, uint32_t Channel);
float 				RTG_TemperatureSensorLM20(void);
CIB_CBIT_ADC 		RTG_Chech_Boundaris(CIB_CBIT_ADC ADC_Res);

#endif /* INC_ADC_H_ */

