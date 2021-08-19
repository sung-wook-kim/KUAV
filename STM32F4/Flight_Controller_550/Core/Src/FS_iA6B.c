/*
 * FS_iA6B.c
 *
 *  Created on: 2021. 7. 6.
 *      Author: muk20
 */
#include "FS-iA6B.h"

FSiA6B_iBus iBus;

unsigned char iBus_Check_CHKSUM(unsigned char* data, unsigned char len)
{
	unsigned short chksum = 0xffff;

	for(int i=0; i<len-2;i++)
	{
		chksum = chksum - data[i];
	}

	return ((chksum&0x00ff)==data[30]) && ((chksum>>8) && data[31]);

}

void iBus_Parsing(unsigned char* data, FSiA6B_iBus* iBus)
{
	iBus->RH = (data[2] | data[3]<<8) & 0x0FFF;
	iBus->RV = (data[4] | data[5]<<8) & 0x0FFF;
	iBus->LV = (data[6] | data[7]<<8) & 0x0FFF;
	iBus->LH = (data[8] | data[9]<<8) & 0x0FFF;
	iBus->SwA = (data[10] | data[11]<<8) & 0x0FFF;
	iBus->SwC = (data[12] | data[13]<<8) & 0x0FFF;
	iBus->SwB = (data[14] | data[15]<<8) & 0x0FFF;
	iBus->SwD = (data[16] | data[17]<<8) & 0x0FFF;
	iBus->VrA = (data[18] | data[19]<<8) & 0x0FFF;
	iBus->VrB = (data[20] | data[21]<<8) & 0x0FFF;

	iBus->FailSafe = (iBus->SwB == 1500) && (iBus->SwD == 1500);
}

unsigned char iBus_isActiveFailSafe(FSiA6B_iBus* iBus)
{
	return iBus->FailSafe != 0;
}

void FSiA6B_UART5_Initialization()
{
	LL_USART_InitTypeDef USART_InitStruct = {0};

	LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* Peripheral clock enable */
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_UART5);

	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOD);
	/**UART5 GPIO Configuration
	PC12   ------> UART5_TX
	PD2   ------> UART5_RX
	*/
	GPIO_InitStruct.Pin = LL_GPIO_PIN_12;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
	GPIO_InitStruct.Alternate = LL_GPIO_AF_8;
	LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = LL_GPIO_PIN_2;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
	GPIO_InitStruct.Alternate = LL_GPIO_AF_8;
	LL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/* UART5 interrupt Init */
	NVIC_SetPriority(UART5_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
	NVIC_EnableIRQ(UART5_IRQn);

	/* USER CODE BEGIN UART5_Init 1 */

	/* USER CODE END UART5_Init 1 */
	USART_InitStruct.BaudRate = 115200;
	USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
	USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
	USART_InitStruct.Parity = LL_USART_PARITY_NONE;
	USART_InitStruct.TransferDirection = LL_USART_DIRECTION_RX;
	USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
	USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
	LL_USART_Init(UART5, &USART_InitStruct);
	LL_USART_ConfigAsyncMode(UART5);
	LL_USART_Enable(UART5);
}
