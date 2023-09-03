/*
 * bit.h
 *
 *  Created on: 26 Dec 2021
 *      Author: itzhaki
 */

#ifndef BIT_H_
#define BIT_H_

#include <main.h>

#define PBIT_MESSAGE_TYPE "PB" // for PBIT TEST
#define CBIT_MESSAGE_TYPE "CB" // for CBIT TEST

#define BIT_SEND_TO_SNA	0

#pragma pack (push,1)

typedef 	struct {
	uint32_t 	readed_reg;
	uint16_t	count;
}Dev_t;


typedef struct
{
	Dev_t 		Pbit;
	Dev_t 		Cbit;
	Dev_t 		Adis;
	Dev_t 		Dlu;
	Dev_t 		Gps;
	Dev_t 		Servo;
	Dev_t 		Saa;
	Dev_t 		Margalit;
} DEVICE_COUNT_t;

#pragma pack (pop)


extern DEVICE_COUNT_t 	DevCount;
extern CIB_BIT_Msg 		CBIT_mes_st;
extern CIB_BIT_Msg 		PBIT_mes_st;

#define PBIT_mes (PBIT_mes_st)
#define pPbitMes ((CIB_PBIT_data*) &PBIT_mes_st.data.pbit)
/*
 *
 */
#define CBIT_CURRENT_BUFF  		CBIT_mes_st.data.cbit
#define CBIT_FULL_BUFF  		CBIT_mes_st
#define CBIT_CURRENT_BUFF_SIZE 	sizeof(CIB_CBIT_data)
#define PBIT_CURRENT_BUFF_SIZE 	sizeof(CIB_PBIT_data)
#define CBIT_FULL_BUFF_SIZE 	( CBIT_CURRENT_BUFF_SIZE + sizeof( CBIT_mes_st.hdr ) )
#define PBIT_FULL_BUFF_SIZE 	( PBIT_CURRENT_BUFF_SIZE + sizeof( PBIT_mes_st.hdr ) )


#define DEV_COUNT_ADD(DEV)  ({ ( DEV = (uint32_t)DEV%0xfffffff0 +1 ); })

#define SET_REG(DEV,NUM)		 ( DevCount.DEV.readed_reg |= ( 1<<NUM ) )
#define GET_REG(DEV,NUM)		 ( ( DevCount.DEV.readed_reg & ( 1<<NUM ) ) != 0 )
#define GET_REG_FULL(DEV)		 ( DevCount.DEV.readed_reg  )
#define ADD_COUNT(DEV)			DEV_COUNT_ADD( DevCount.DEV.count )
#define GET_COUNT(DEV)			( DevCount.DEV.count )

#define SET_PBIT_ENDED		   SET_REG( Pbit   	,31)
#define PBIT_IS_ENDED 		   GET_REG( Pbit	,31)

#define PBIT_ADD_COUNT			ADD_COUNT( Pbit )
#define CBIT_ADD_COUNT			ADD_COUNT( Cbit )

#define SET_CBIT_ENDED		   SET_REG( Cbit   	,31)
#define CBIT_IS_ENDED 		   GET_REG( Cbit	,31)

#define SAA_IS_READED 			GET_REG(Saa       ,31 )
#define DLU_IS_READED 			GET_REG(Dlu       ,31 )
#define GPS_IS_READED			GET_REG(Gps       ,31 )
#define GPS_IS_READ_V			GET_REG(Gps       ,30 )
#define MGL_IS_READED			GET_REG(Margalit  ,31 )
#define ADS_IS_READED			GET_REG(Adis 	  ,31 )
#define SRV_IS_READED			GET_REG(Servo 	  ,31 )

#define GPS_ADD_COUNT			ADD_COUNT( Gps )
#define SAA_ADD_COUNT			ADD_COUNT( Saa )
#define DLU_ADD_COUNT			ADD_COUNT( Dlu )
#define MGL_ADD_COUNT			ADD_COUNT( Margalit )
#define ADS_ADD_COUNT			ADD_COUNT( Adis )
#define SRV_ADD_COUNT			ADD_COUNT( Servo )

#define GPS_SET_READED         SET_REG(	Gps		,31 )
#define GPS_SET_READ_V         SET_REG(	Gps		,30 )
#define SAA_SET_READED         SET_REG( Saa		,31 )
#define DLU_SET_READED         SET_REG( Dlu		,31 )
#define MGL_SET_READED         SET_REG( Margalit	,31 )
#define ADS_SET_READED         SET_REG( Adis		,31 )
#define SRV_SET_READED         SET_REG( Servo	    ,31 )

#define MGL_SET_R_INEX(IND)    SET_REG( Margalit   ,IND)
#define MGL_PBIB_MASK		   ( ( 1<<( ID_System_Part_No - ID_Warning_BIT_Results_5 ) ) - 1 )
#define MGL_R_IS_FULL		   ( ( GET_REG_FULL( Margalit ) & MGL_PBIB_MASK ) == MGL_PBIB_MASK )


void RTG_PBIT_FUNC();
void RTG_CBIT_FUNC();
uint16_t RTG_CpuInternalRamTest(CIB_PBIT_INTERNAL_RAMS *mes);

#endif /* BIT_H_ */
