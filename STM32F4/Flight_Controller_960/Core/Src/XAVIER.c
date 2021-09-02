/*
 * Xavier.c
 *
 *  Created on: Jul 16, 2021
 *      Author: wesle
 */
#include "XAVIER.h"
XAVIER_RX XAVIER_rx;
XAVIER_TX XAVIER_tx;

void XAVIER_RX_Parsing(unsigned char* data, XAVIER_RX* XAVIER_rx)
{
	XAVIER_rx->mode = data[2];
	XAVIER_rx->lat = (data[3]<<24 | data[4]<<16 | data[5]<<8 | data[6]) ;
	XAVIER_rx->lon = (data[7]<<24 | data[8]<<16 | data[9]<<8 | data[10]) ;
	XAVIER_rx->target_yaw = data[11] << 8 | data[12];
	XAVIER_rx->lidar = data[14] << 8 | data[13];
}
