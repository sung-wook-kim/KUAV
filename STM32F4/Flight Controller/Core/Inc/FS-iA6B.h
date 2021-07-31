#ifndef _FSIA6B_H
#define _FSIA6B_H
#ifdef _cplusplus
	extern "C" {
#endif

#include "main.h"

typedef struct _FSiA6B_iBus
{
	unsigned short RH;
	unsigned short RV;
	unsigned short LV;
	unsigned short LH;
	unsigned short SwA;
	unsigned short SwB;
	unsigned short SwC;
	unsigned short SwD;
	unsigned short VrA;
	unsigned short VrB;

	unsigned char FailSafe;
}FSiA6B_iBus;

extern FSiA6B_iBus iBus;

unsigned char iBus_Check_CHKSUM(unsigned char* data, unsigned char len);
void iBus_Parsing(unsigned char* data, FSiA6B_iBus* iBus);
void FSiA6B_UART5_Initialization();
unsigned char iBus_isActiveFailSafe(FSiA6B_iBus* iBus);

#ifdef _cplusplus
}
#endif
#endif /* _FSIA6B_H */
