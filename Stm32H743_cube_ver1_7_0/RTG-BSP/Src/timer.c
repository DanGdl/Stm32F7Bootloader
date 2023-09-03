/*
 * timers.c
 *
 *  Created on: May 1, 2021
 *      Author: itzhaki
 */

#include <main.h>
#include <bit.h>
#include <eth_rtg.h>
#include <timer.h>
#include <ADIS16547.h>
#include <GPS.h>


extern __IO uint32_t uwTick;
extern TIM_HandleTypeDef htim4;

Timers Timer4;


/*************************  One PPC ************************************/
TIMSt_t OnePPC_Time = {
		.TodCurrent = 0,
		.todError = TOD_MAX_ERROR+100
};


//timer Init
void RTG_TIM_Init(Timers *htim)
{
/*
 * timer4  25 mks
 */
	Timer4.handle = &htim4;
	Timer4.time = 25;
	Timer4.timeBase = microsecond;
	Timer4.counter = 0;
	Timer4.oc_channel = (uint32_t) TIM_CHANNEL_1;
//Enabled interrupt for timer
	 __HAL_TIM_ENABLE_IT(htim->handle, TIM_IT_CC1);

	 // configure timer
	 if (RTG_setTimerTime(htim) != HAL_OK)
	 {
		 Error_Handler();
	 }
	 HAL_TIM_Base_Start_IT(htim->handle);
}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(&htim4==htim)
	{
//		OnePPC_Time.dSec += 0.000025;
	OnePPC_Time.ticks25Mks ++;
	}
}

void updateTod(uint32_t tod){
	// if is first
	float TOD_TOLLERANCE_SCALE;
	OnePPC_Time.delta = RTC_DELTA;
	SET_TOD_CURRENT(tod);
	if( OnePPC_Time.todError > TOD_MAX_ERROR ){
		SET_RTC_BASE;
		SET_TOD_BASE;
		SET_OFFSET;
		OnePPC_Time.todError = 0;
		OnePPC_Time.todUpdate++;
	}
	else{
//		SET_TOD_CURRENT(tod);
		OnePPC_Time.deltaTod = TOD_DELTA;
		TOD_TOLLERANCE_SCALE = TOD_DELTA / 40000.0;
		if( TOD_VALIDATE ){
			SET_RTC_BASE;
			SET_TOD_BASE;
			SET_OFFSET;
			OnePPC_Time.todError = 0;
		}
		else {
			OnePPC_Time.todError++;
		}
	}
}

uint8_t CIB2SEEKER_CheckSum(char* str, uint16_t len){
	uint16_t i=1;
	uint8_t checkSum=0;
	for( i = 1; i<len; i++){
		checkSum ^=str[i];
	}
	return checkSum;
}
// Find the Number Of Digits that are not zero
static uint8_t RTG_NumberOfDigitsNotZero(uint32_t num)
{
	uint8_t count = 0, n, i;

	n = log10(num) + 1;

	for (i = 0; i < n; ++i, num /= 10)
	{
		if(num % 10) count++;
	}

	return count;
}

// Set timer parameter according to time value
// timeBase : 1 - usec, 2 - msec, 3 - sec
// value of time
HAL_StatusTypeDef RTG_setTimerTime(Timers *htim)
{
	uint32_t TIMInputFreq;
	uint32_t presNum, devNum, powXvalue;//, multiplayVal;
	uint8_t NumberOfDNotZ, NumberOfD, x;

	uint32_t convertTimeBase;

	//Prescale input freq
	TIMInputFreq = HAL_RCC_GetPCLK1Freq() ;

	//Number Of Digits that are not zero
	NumberOfDNotZ = RTG_NumberOfDigitsNotZero(TIMInputFreq);

	//Number Of Digits
	NumberOfD = log10(TIMInputFreq) + 1;

	switch(htim->timeBase)
	{
		case 1://usec
			convertTimeBase = 1;
			break;
		case 2://msec
			convertTimeBase = 1000;
			break;
		case 3://sec
			convertTimeBase = 1000000;
			break;
		default: convertTimeBase = 1000000;
			break;
	}

	x = NumberOfD - NumberOfDNotZ;

	powXvalue = pow(10, x);

	// Find Prescaler number
	presNum = powXvalue / 1000000;

	devNum = htim->time * convertTimeBase;

	if(devNum == 0 )
		return HAL_ERROR;

	if(presNum > 1)
		htim->handle->Init.Prescaler = presNum - 1;
	else return HAL_ERROR;


	htim->handle->Init.Period = devNum * 2 - 1;

	HAL_TIM_Base_Init(htim->handle);

	return HAL_OK;
}
