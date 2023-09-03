/*
 * RTG_includes.h
 *
 *  Created on: 6 ביוני 2021
 *      Author: e_tas
 */

#ifndef INC_RTG_MAIN_H_
#define INC_RTG_MAIN_H_

#include <sys/_stdint.h>

void RTG_MAIN_INIT();
void RTG_Entry_Point(void);
/*
 * Pack data source and copy to destination
 */
uint16_t RTG_Copy_Data_to_Message(uint8_t *source, uint8_t *dist, uint16_t len, uint8_t count );
uint16_t RTG_CalculateChecksum(uint8_t *dataToCalculate, uint32_t size);
/*
 *  Insert to source header UDP to start
 */
uint16_t RTG_Prepare_pack_msg(uint8_t *source, uint16_t len, uint8_t count, uint8_t spare);

#endif /* INC_RTG_MAIN_H_ */
