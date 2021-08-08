/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __M8P_H__
#define __M8P_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

typedef struct _M8P_UBX_NAV_POSLLH
{
	unsigned char CLASS;
	unsigned char ID;
	unsigned short length;

	unsigned int iTOW;
	signed int lon;
	signed int lon_prev;
	signed int lat;
	signed int lat_prev;
	signed int height;
	signed int hMSL;
	unsigned int hAcc;
	unsigned int vAcc;

	double lon_f64;
	double lat_f64;
}M8P_UBX_NAV_POSLLH;


typedef struct _M8P_UBX_NAV_PVT
{
	unsigned char CLASS;
	unsigned char ID;
	unsigned short length;

	unsigned int iTOW;
	unsigned short year;
	unsigned char month;
	unsigned char day;
	unsigned char hour;
	unsigned char min;
	unsigned char sec;
	unsigned char valid; //X1 date type
	unsigned int tAcc;
	signed int nano;
	unsigned char fixType;
	unsigned char flags; //X1 date type
	unsigned char flags2; //X1 date type
	unsigned char numSV;
	signed int lon;
	signed int lon_prev;
	signed int lat;
	signed int lat_prev;
	signed int height;
	signed int hMSL;
	unsigned int hAcc;
	unsigned int vAcc;
	signed int velN;
	signed int velE;
	signed int velD;
	signed int gSpeed;
	signed int headMot;
	unsigned int sAcc;
	unsigned int headAcc;
	unsigned short pDOP;
	unsigned short flags3; //X2 date type
	unsigned int reserved1;
	signed int headVeh;
	signed short magDec;
	unsigned short magAcc;

}M8P_UBX_NAV_PVT;

extern M8P_UBX_NAV_POSLLH posllh;
extern M8P_UBX_NAV_PVT pvt;


unsigned char M8P_UBX_CHKSUM_Check(unsigned char* data, unsigned char len);
void M8P_UBX_NAV_POSLLH_Parsing(unsigned char* data, M8P_UBX_NAV_POSLLH* posllh);
void M8P_UBX_NAV_PVT_Parsing(unsigned char* data, M8P_UBX_NAV_PVT* pvt);

void M8P_TransmitDate(unsigned char* data, unsigned char len);
void M8P_UART4_Initialization(void);
void M8P_Initialization(void);

#ifdef __cplusplus
}
#endif

#endif /* __M8P_H__ */
