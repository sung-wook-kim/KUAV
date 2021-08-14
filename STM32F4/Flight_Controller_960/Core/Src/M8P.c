#include "M8P.h"

M8P_UBX_NAV_POSLLH posllh;
M8P_UBX_NAV_PVT pvt;


const unsigned char M8P_UBX_CFG_PRT[] = {
0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x80, 0x25, 0x00,
0x00, 0x20, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0xB9, 0x71
}; //UBX Protocol In, Out, UART1, 8N1-9600

const unsigned char M8P_UBX_CFG_MSG[] = {
0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x02, 0x00, 0x01,
0x00, 0x00, 0x00, 0x00, 0x13, 0xBE
}; //NAV-POSLLH(01-02), UART1

const unsigned char M8P_UBX_CFG_RATE[] = {
0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00,
0x01, 0x00, 0xDE, 0x6A
}; //GPS Time, 5Hz Navigation Frequency

const unsigned char M8P_UBX_CFG_CFG[] = {
0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00, 0x00, 0x00, 0x00, 0x00,
0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x17, 0x31,
0xBF
}; //Save current configuration, Devices: BBR, FLASH, I2C-EEPROM, SPI-FLASH,

const unsigned char M8P_UBX_CFG_MSGPVT[] = {
0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x07, 0x00, 0x01,
0x00, 0x00, 0x00, 0x00, 0x18, 0xE1
}; //NAV-POSLLH(01-07), UART1

const unsigned char M8P_UBX_CFG_TMODE3[] = {
0xB5, 0x62, 0x06, 0x71, 0x28, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x9F, 0x93
}; //TIME MODE 3, Disabled

void M8P_TransmitData(unsigned char* data, unsigned char len)
{
	for(int i=0; i<len; i++)
	{
		while(!LL_USART_IsActiveFlag_TXE(UART4));
		LL_USART_TransmitData8(UART4, *(data+i));
	}
}

void M8P_UART4_Initialization(void)
{
	  LL_USART_InitTypeDef USART_InitStruct = {0};

	  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

	  /* Peripheral clock enable */
	  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_UART4);

	  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
	  /**UART4 GPIO Configuration
	  PC10   ------> UART4_TX
	  PC11   ------> UART4_RX
	  */
	  GPIO_InitStruct.Pin = LL_GPIO_PIN_10|LL_GPIO_PIN_11;
	  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
	  GPIO_InitStruct.Alternate = LL_GPIO_AF_8;
	  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	  /* UART4 interrupt Init */
	  NVIC_SetPriority(UART4_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
	  NVIC_EnableIRQ(UART4_IRQn);

	  /* USER CODE BEGIN UART4_Init 1 */

	  /* USER CODE END UART4_Init 1 */
	  USART_InitStruct.BaudRate = 9600;
	  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
	  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
	  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
	  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
	  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
	  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
	  LL_USART_Init(UART4, &USART_InitStruct);
	  LL_USART_ConfigAsyncMode(UART4);
	  LL_USART_Enable(UART4);
}

void M8P_Initialization(void)
{
	M8P_UART4_Initialization();
	M8P_TransmitData(&M8P_UBX_CFG_PRT[0], sizeof(M8P_UBX_CFG_PRT));
	HAL_Delay(100);
	M8P_TransmitData(&M8P_UBX_CFG_TMODE3[0], sizeof(M8P_UBX_CFG_TMODE3));
	HAL_Delay(100);
	M8P_TransmitData(&M8P_UBX_CFG_MSGPVT[0], sizeof(M8P_UBX_CFG_MSGPVT));
	HAL_Delay(100);
	M8P_TransmitData(&M8P_UBX_CFG_RATE[0], sizeof(M8P_UBX_CFG_RATE));
	HAL_Delay(100);
	M8P_TransmitData(&M8P_UBX_CFG_CFG[0], sizeof(M8P_UBX_CFG_CFG));
	HAL_Delay(100);
}

unsigned char M8P_UBX_CHKSUM_Check(unsigned char* data, unsigned char len)
{
	unsigned char CK_A = 0, CK_B = 0;

	for(int i=2;i<len-2;i++)
	{
		CK_A = CK_A + data[i];
		CK_B = CK_B + CK_A;
	}

	return ((CK_A == data[len-2]) && (CK_B == data[len-1]));
}

void M8P_UBX_NAV_POSLLH_Parsing(unsigned char* data, M8P_UBX_NAV_POSLLH* posllh)
{
	posllh->CLASS = data[2];
	posllh->ID = data[3];
	posllh->length = data[4] | data [5]<<8;

	posllh->iTOW = data[6] | data[7]<<8 | data[8]<<16 | data[9]<<24 ;
	posllh->lon = data[10] | data[11]<<8 | data[12]<<16 | data[13]<<24 ;
	posllh->lat = data[14] | data[15]<<8 | data[16]<<16 | data[17]<<24 ;
	posllh->height = data[18] | data[19]<<8 | data[20]<<16 | data[21]<<24 ;
	posllh->hMSL = data[22] | data[23]<<8 | data[24]<<16 | data[25]<<24 ;
	posllh->hAcc = data[26] | data[27]<<8 | data[28]<<16 | data[29]<<24 ;
	posllh->vAcc = data[30] | data[31]<<8 | data[32]<<16 | data[33]<<24 ;

//	posllh->lon_f64 = posllh->lon/ 10000000.;
//	posllh->lat_f64 = posllh->lat/ 10000000.;

}


void M8P_UBX_NAV_PVT_Parsing(unsigned char* data, M8P_UBX_NAV_PVT* pvt)
{
	pvt->CLASS = data[2];
	pvt->ID = data[3];
	pvt->length = data[4] | data [5]<<8;

	pvt->iTOW = data[6] | data[7]<<8 | data[8]<<16 | data[9]<<24 ;
	pvt->year =  data[10] | data[11]<<8 ;
	pvt->month = data[12] ;
	pvt->day = data[13] ;
	pvt->hour = data[14] ;
	pvt->min = data[15] ;
	pvt->sec = data[16] ;
	pvt->valid = data[17] ;
	pvt->tAcc = data[18] | data[19]<<8 | data[20]<<16 | data[21]<<24 ;
	pvt->nano = data[22] | data[23]<<8 | data[24]<<16 | data[25]<<24 ;
	pvt->fixType = data[26] ;
	pvt->flags = data[27] ;
	pvt->flags2 = data[28] ;
	pvt->numSV = data[29] ;
	pvt->lon = data[30] | data[31]<<8 | data[32]<<16 | data[33]<<24 ;
	pvt->lat = data[34] | data[35]<<8 | data[36]<<16 | data[37]<<24 ;
	pvt->height = data[38] | data[39]<<8 | data[40]<<16 | data[41]<<24 ;
	pvt->hMSL = data[42] | data[43]<<8 | data[44]<<16 | data[45]<<24 ;
	pvt->hAcc = data[46] | data[47]<<8 | data[48]<<16 | data[49]<<24 ;
	pvt->vAcc = data[50] | data[51]<<8 | data[52]<<16 | data[53]<<24 ;
	pvt->velN = data[54] | data[55]<<8 | data[56]<<16 | data[57]<<24 ;
	pvt->velE = data[58] | data[59]<<8 | data[60]<<16 | data[61]<<24 ;
	pvt->velD = data[62] | data[63]<<8 | data[64]<<16 | data[65]<<24 ;
	pvt->gSpeed = data[66] | data[67]<<8 | data[68]<<16 | data[69]<<24 ;
	pvt->headMot = data[70] | data[71]<<8 | data[72]<<16 | data[73]<<24 ;
	pvt->sAcc = data[74] | data[75]<<8 | data[76]<<16 | data[77]<<24 ;
	pvt->headAcc = data[78] | data[79]<<8 | data[80]<<16 | data[81]<<24 ;
	pvt->pDOP = data[82] | data[83]<<8 ;
	pvt->flags3 = data[84] | data[85]<<8 ;
	pvt->reserved1 = data[86] | data[87]<<8 | data[88]<<16 | data[89]<<24 ;
	pvt->headVeh = data[90] | data[91]<<8 | data[92]<<16 | data[93]<<24 ;
	pvt->magDec = data[94] | data[95]<<8 ;
	pvt->magAcc = data[96] | data[97]<<8 ;
	//	pvt->lon_f64 = posllh->lon/ 10000000.;
	//	pvt->lat_f64 = posllh->lat/ 10000000.;
}
