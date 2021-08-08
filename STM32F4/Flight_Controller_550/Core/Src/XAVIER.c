/*
 * Xavier.c
 *
 *  Created on: Jul 16, 2021
 *      Author: wesle
 */
#include "XAVIER.h"
XAVIER_RX XAVIER;

void XAVIER_Parsing(unsigned char* data, XAVIER_RX* XAVIER)
{
	XAVIER->mode = data[2];
	XAVIER->lat = (data[3]<<24 | data[4]<<16 | data[5]<<8 | data[6]) & 0xFFFF;
	XAVIER->lon = (data[7]<<24 | data[8]<<16 | data[9]<<8 | data[10]) & 0xFFFF;
	XAVIER->yaw_error = data[11];
}
