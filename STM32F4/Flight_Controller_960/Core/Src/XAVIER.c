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
	XAVIER_rx->lat = (data[3]<<24 | data[4]<<16 | data[5]<<8 | data[6]) & 0xFFFF;
	XAVIER_rx->lon = (data[7]<<24 | data[8]<<16 | data[9]<<8 | data[10]) & 0xFFFF;
	XAVIER_rx->yaw_error = data[11];
	XAVIER_rx->lidar = data[13] << 8 | data[12];
}

//
//void XAVIER_TX_Parsing(unsigned char* data, XAVIER_TX* XAVIER_tx)
//{
//	XAVIER_tx->mode = data[2];
//	XAVIER_tx->lat = (data[3]<<24 | data[4]<<16 | data[5]<<8 | data[6]) & 0xFFFF;
//	XAVIER_tx->lon = (data[7]<<24 | data[8]<<16 | data[9]<<8 | data[10]) & 0xFFFF;
//	XAVIER_tx->iTow = (data[11]<<24 | data[12]<<16 | data[13]<<8 | data[14]) & 0xFFFF;
//	XAVIER_tx->roll = (data[15]<<24 | data[16]<<16 | data[17]<<8 | data[18]) & 0xFFFF;
//	XAVIER_tx->pitch = (data[19]<<24 | data[20]<<16 | data[21]<<8 | data[22]) & 0xFFFF;
//	XAVIER_tx->yaw = (data[23]<<24 | data[24]<<16 | data[25]<<8 | data[26]) & 0xFFFF;
//	XAVIER_tx->altitude = (data[27]<<24 | data[28]<<16 | data[29]<<8 | data[30]) & 0xFFFF;
//}
