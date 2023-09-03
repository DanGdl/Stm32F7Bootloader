/*
 * IMU-TRF-90M.c
 *
 *  Created on: 17 Apr 2022
 *      Author: itzhaki
 */
#include <ethernetif.h>
#include <semphr.h>

#include <bit.h>
#include <IMU-TRF-90M.h>
#include <string.h>
#include <timer.h>


void RTG_Update_Margalit_PBIT(IMU_MGLT_Msg *pMARSrc, CIB_PBIT_Margalit *pDist)
{
	static uint8_t index=0;
	if ( 	( pMARSrc->Header == MGLT_READ_START_16BIT )
			&& ( pMARSrc->MUX_Status.Mux_Count > ID_Warning_BIT_Results_5 )
			&& ( pMARSrc->MUX_Status.Mux_Count < 32  )
		)
	{
		index = pMARSrc->MUX_Status.Mux_Count-ID_Warning_BIT_Results_5 - 1;
		((uint16_t*)pDist)[index] = *((uint16_t*) &pMARSrc->MUX_Data) ;
		MGL_SET_R_INEX(index);
	}
	else return;

	pPbitMes->error_bits.TRF =  !!(*(uint16_t*)&pMARSrc->BIT_Status);
}
void RTG_Update_Margalit_CBIT(IMU_MGLT_Msg *pMARSrc, CIB_CBIT_Margalit *pDist)
{
	if ( ( pMARSrc->Header == MGLT_READ_START_16BIT )
			&& ( pMARSrc->MUX_Status.Mux_Count <= ID_Warning_BIT_Results_5 ) )
	{
		// Copy status bit
		taskENTER_CRITICAL();
		((uint16_t*)&pDist->GYRO_X_Temperature)[pMARSrc->MUX_Status.Mux_Count]
						   = *((uint16_t*) &pMARSrc->MUX_Data) ;
		pDist->time_tag = GetTimeTag;
		taskEXIT_CRITICAL();

		CBIT_CURRENT_BUFF.current_bit.TRF =  !!(*(uint16_t*)&pMARSrc->BIT_Status);
		CBIT_CURRENT_BUFF.sticky_bit.TRF |=  !!(*(uint16_t*)&pMARSrc->BIT_Status);
	}
}

uint32_t int_24_to_32 (unsigned char* int24) {
	int32_t msb = (signed char)int24[2];
	uint32_t mdb = int24[1];
	uint32_t lsb = int24[0];
	return ((msb << 16) | (mdb << 8) | lsb);
} // int_24_to_32

void RTG_MargalitTo_400Hz(IMU_MGLT_Msg src, CIB_IMU_Data *dist)
{
	CIB_IMU_Data temp ={0};
	temp.X_D_Ang_i = int_24_to_32 ( src.G_X_Delta_Ang ) ;
	temp.Y_D_Ang_i = int_24_to_32 ( src.G_Y_Delta_Ang ) ;
	temp.Z_D_Ang_i = int_24_to_32 ( src.G_Z_Delta_Ang ) ;
	temp.X_D_Vel_i = int_24_to_32 ( src.A_X_Delta_Vel ) ;
	temp.Y_D_Vel_i = int_24_to_32 ( src.A_Y_Delta_Vel ) ;
	temp.Z_D_Vel_i = int_24_to_32 ( src.A_Z_Delta_Vel ) ;
	temp.TIME_STAMP = GetTimeTag;
	taskENTER_CRITICAL();
	*dist = temp;
	taskEXIT_CRITICAL();
}

