/*
 * IMU - TRF-90M.h
 *
 *  Created on: 17 Apr 2022
 *      Author: itzhaki
 */

#ifndef INC_IMU_TRF_90M_H_
#define INC_IMU_TRF_90M_H_

#include <CIB_Protocol.h>


//######################################
//############## CONSTATNS #############
//######################################

// OPCODES
#define  MGLT_READ_START_16BIT			0xed88




uint32_t int_24_to_32 (unsigned char* int24);
void RTG_MargalitTo_400Hz(IMU_MGLT_Msg src, CIB_IMU_Data *dist);

void RTG_Update_Margalit_PBIT( IMU_MGLT_Msg *pMARSrc, CIB_PBIT_Margalit *pDist );
void RTG_Update_Margalit_CBIT( IMU_MGLT_Msg *pMARSrc, CIB_CBIT_Margalit *pDist );

#endif /* INC_IMU_TRF_90M_H_ */
