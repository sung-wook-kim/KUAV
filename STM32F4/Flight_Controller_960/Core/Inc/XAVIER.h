#ifndef _XAVIER_H
#define _XAVIER_H
#ifdef _cplusplus
	extern "C" {
#endif

#include "main.h"


	typedef struct _XAVIER_RX
	{
		unsigned char mode;
		signed int lat;
		signed int lon;
		int yaw_error;
		unsigned short lidar;
	}XAVIER_RX;

	typedef struct _XAVIER_TX
	{
		unsigned char mode;
		signed int lat;
		signed int lon;
		unsigned int iTow;
		int roll;
		int pitch;
		int yaw;
		int altitude;
	}XAVIER_TX;

	extern XAVIER_RX XAVIER_rx;
	extern XAVIER_TX XAVIER_tx;

	void XAVIER_RX_Parsing(unsigned char* data, XAVIER_RX* XAVIER_rx);
//	void XAVIER_TX_Parsing(unsigned char* data, XAVIER_TX* XAVIER_tx);

#ifdef _cplusplus
}
#endif
#endif /* _XAVIER_H */
