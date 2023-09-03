/*
 * GPS.c
 *
 *  Created on: Feb 27, 2022
 *      Author: itzhaki
 */

#include <bit.h>
#include <GPS.h>
#include <string.h>
#include <sys/_stdint.h>
#include <timer.h>
#include <uarts.h>

OEM7_Hader_t oem_7={
		.Sync = {0xaa,0x44,0x12},
		.HeaderLength = 28,
};
// OEM7<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
OEM7_Hader_t Oem7;
OEM7_Hader_t* oem7=(OEM7_Hader_t*) &Oem7;


void GPS_SendVersion(){
	uint8_t gps_mes[] = "log com3 versionb once\n\r";
	memcpy(sUart [UART_GPS].TXbuf,gps_mes,sizeof(gps_mes));
	sUart [UART_GPS].dataTx_length=sizeof(gps_mes);
	RTG_Uart_SendMessege(&sUart [UART_GPS]);
}
/* --------------------------------------------------------------------------
Calculate a CRC value to be used by CRC calculation functions.
-------------------------------------------------------------------------- */
#define CRC32_POLYNOMIAL 0xEDB88320L
unsigned long CRC32Value(int i) {
	int j;
	unsigned long ulCRC;
		 ulCRC = i;
	for ( j = 8 ; j > 0; j-- ) {
	if ( ulCRC & 1 )
				   ulCRC = ( ulCRC >> 1 ) ^ CRC32_POLYNOMIAL;
	else
				   ulCRC >>= 1;
	}
	return ulCRC;
}
/* --------------------------------------------------------------------------
Calculates the CRC-32 of a block of data all at once
ulCount - Number of bytes in the data block
ucBuffer - Data block
-------------------------------------------------------------------------- */
unsigned long CalculateBlockCRC32( unsigned long ulCount, unsigned char*ucBuffer ) {
	unsigned long ulTemp1;
	unsigned long ulTemp2;
	unsigned long ulCRC = 0;
	while ( ulCount-- != 0 ) {
			  ulTemp1 = ( ulCRC >> 8 ) & 0x00FFFFFFL;
			  ulTemp2 = CRC32Value( ((int) ulCRC ^ *ucBuffer++ ) & 0xFF );
			  ulCRC = ulTemp1 ^ ulTemp2;
	}
	return( ulCRC );
}

// OEM7<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

