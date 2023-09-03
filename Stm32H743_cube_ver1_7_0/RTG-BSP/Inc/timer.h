/*
 * timers.h
 *
 *  Created on: May 1, 2021
 *      Author: itzhaki
 */

#ifndef INC_TIMERS_H_
#define INC_TIMERS_H_

#include "stm32h7xx_hal.h"

#pragma pack(push,1)

typedef enum timeBaseUnit
{
	microsecond = 1, millisecond = 2, second = 3
} timeBaseU;

typedef struct timerStruct
{
	TIM_HandleTypeDef *handle;
	uint32_t time;
	timeBaseU timeBase;
	uint32_t counter;
	uint32_t oc_channel;
} Timers;

typedef struct
{
	uint32_t ticks25Mks;
	uint32_t RtcLast;
	uint32_t TodBase;
	uint32_t TodCurrent;
	uint32_t RtcBase;
	uint32_t Offset;
	uint32_t todError;
	// for DBUG
	uint32_t RtcPrev;
	uint32_t todUpdate;
	uint32_t RTCdelta;
	uint32_t delta;
	uint32_t deltaTod;
} TIMSt_t;

#pragma pack(pop)

extern TIM_HandleTypeDef htim2;

/*
 * CIB_BASE_SEC
 * TOD_SEC
 * OFFSET = TOD_SEC-CIB_BASE_SEC
 *
 * TIMETAG = CURRENT - OFFSET
 */

extern TIMSt_t OnePPC_Time;
#define TOD_TOLLERANCE	8// t * 25 mks

#define DAY_IN_TCKS_25MKS  				( 60U*60U*24U*40000U )
#define DELTA_( curr_, prev_ ,limit_ )	(  ( ( limit_ ) + (curr_) -( prev_))%(limit_) )
#define DELTA( curr_, prev_ )			DELTA_(curr_,prev_,DAY_IN_TCKS_25MKS)

#define SET_TOD_CURRENT(tod)		    ({ OnePPC_Time.TodCurrent = tod; })
#define TOD_DELTA						DELTA(OnePPC_Time.TodCurrent, OnePPC_Time.TodBase)
#define RTC_DELTA 						DELTA(OnePPC_Time.RtcLast, OnePPC_Time.RtcBase)
#define DELTA_TOLLERANCE				( abs( TOD_DELTA - RTC_DELTA ) )

#define TICKS25MKS_TO_D_SEC(t25) 	    ( (double)t25 * 0.000025 )
#define SEC_TO_TICKS(sec) 			    ( (uint32_t)(sec * 40000) )
                                        
#define SET_RTC_BASE				    ({ OnePPC_Time.RtcBase = OnePPC_Time.RtcLast; })
#define SET_TOD_BASE				    ({ OnePPC_Time.TodBase = OnePPC_Time.TodCurrent; })
#define SET_OFFSET					    ({ OnePPC_Time.Offset = OnePPC_Time.TodBase - OnePPC_Time.RtcBase; })

#define TOD_VALIDATE					( DELTA_TOLLERANCE < TOD_TOLLERANCE_SCALE*TOD_TOLLERANCE )
#define TOD_MAX_ERROR					500

                                        
#define GetTimeTag 					    ( OnePPC_Time.ticks25Mks + OnePPC_Time.Offset )
                                        
#define GetTimeTagSec	 			    ( TICKS25MKS_TO_D_SEC(GetTimeTag) )
#define GetTimeCurrent				    ( TICKS25MKS_TO_D_SEC(OnePPC_Time.ticks25Mks) )
/*************************  One PPC ************************************/
enum {
	B_GGA = 0,
	B_GSA = 1,
	B_GSV = 2,
	B_ZDA = 3,
};

#define GP_GGA "$GPGGA,%02lu%02lu%02lu.00,5317.95462,N,00616.12095,W,1,4,5.78,48.3,M,54.9,M,,"
#define GP_GSA "$GPGSA,A,2,19,11,03,,,,,,,,,,5.87,5.78,1.00"
#define GP_GSV "$GPGSV,2,1,04,19,79,152,45,11,47,259,44,44,03,39,137,51,28,25,314"

void updateTod(uint32_t tod);
uint8_t CIB2SEEKER_CheckSum(char* str, uint16_t len);

// PA15 -output- change 1/0 per tick
void RTG_TIM_Init(Timers *htim);
HAL_StatusTypeDef RTG_setTimerTime(Timers *htim);


#endif /* INC_TIMERS_H_ */
