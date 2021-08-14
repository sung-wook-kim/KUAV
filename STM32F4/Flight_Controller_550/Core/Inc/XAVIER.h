#ifndef _XAVIER_H
#define _XAVIER_H
#ifdef _cplusplus
	extern "C" {
#endif

#include "main.h"


	typedef struct _XAVIER_RX
	{
		int mode;
		unsigned short lat;
		unsigned short lon;
		int yaw_error;
	}XAVIER_RX;

	extern XAVIER_RX XAVIER;

	void XAVIER_Parsing(unsigned char* data, XAVIER_RX* XAVIER);










#ifdef _cplusplus
}
#endif
#endif /* _XAVIER_H */
