/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bno080.h"
#include "quaternion.h"
#include "icm20602.h"
#include "FS-iA6B.h"
#include "pid control.h"
#include <string.h>
#include "AT24C08.h"
#include "lps22hh.h"
#include "M8N.h"
#include "XAVIER.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
int _write(int file, char* p, int len)
{
	for(int i=0;i<len;i++)
	{
		while(!LL_USART_IsActiveFlag_TXE(USART6));
		LL_USART_TransmitData8(USART6, *(p+i));
		HAL_Delay(1);
	}
}

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// Communication variables
extern uint8_t uart6_rx_flag;
extern uint8_t uart6_rx_data;

extern uint8_t m8n_rx_buf[100];
extern uint8_t m8n_rx_cplt_flag;

extern uint8_t ibus_rx_buf[32];
extern uint8_t ibus_rx_cplt_flag;

extern uint8_t uart1_rx_data;
uint8_t telemetry_tx_buf[220];
uint8_t telemetry_rx_buf[30];
uint8_t telemetry_rx_cplt_flag;
unsigned int chksum_pid = 0xffffffff;

extern uint8_t nx_rx_cplt_flag;
extern uint8_t nx_rx_buf;

// Timer variables
extern uint8_t tim7_1ms_flag;
extern uint8_t tim7_10ms_flag;
extern uint8_t tim7_20ms_flag;
extern uint8_t tim7_100ms_flag;
extern uint8_t tim7_1000ms_flag;

// System flag
unsigned char motor_arming_flag = 0;
unsigned char failsafe_flag = 0;
unsigned char low_bat_flag = 0;
unsigned char flight_mode = 0; // 1 : manual, 2 : Altitude Hold, 3 : Gps Hold + Altitude hold 4 : Return to Home

// Altitude Value
float altitude_setpoint;
float altitude_filt;
float baro_offset = 0;
int baro_cnt = 0;
float pressure_total_average = 0;
float pressure_rotating_mem[20] = { 0,};
int pressure_rotating_mem_location = 0;
float actual_pressure_diff;
float actual_pressure_fast = 0, actual_pressure_slow = 0;
float actual_pressure;

// Gps Value
#define DECLINATION 8.5f
double lat_gps_previous;
double lon_gps_previous;
double lat_gps_actual;
double lon_gps_actual;
double lat_add;
double lon_add;
double lat_gps;
double lon_gps;
double lat_waypoint;
double lon_waypoint;
double lat_diff;
double lon_diff;
float gps_roll_adjust;
float gps_pitch_adjust;
#define GPS_PD_MAX 200
#define GPS_PD_MIN -GPS_PD_MAX

// Return to home value
unsigned char return_to_home_step = 0;
signed int lat_gps_home = 0;
signed int lon_gps_home = 0;
float return_to_home_decrease;
float return_to_home_lat_factor = 0, return_to_home_lon_factor = 0,return_to_home_move_factor = 0;
float l_lat_gps_float_adjust = 0, l_lon_gps_float_adjust = 0;
unsigned char is_lat_nearby = 0, is_lon_nearby = 0;

// Motor Value
uint8_t ccr[18];
unsigned int ccr1 ,ccr2, ccr3, ccr4;
unsigned int takeoff_throttle;

// Extra
float theta, theta_radian;
float batVolt = 0;
float dummy = 0;
float batVolt_prev = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
int abs_int(int, int);
float abs_float(float, float);
int Is_iBus_Throttle_min(void);
void ESC_Calibration(void);
int Is_iBus_Received(void);
void Receive_Pid_Gain(void);
void BNO080_Calibration(void);
void return_to_home(void);
void Calculate_Takeoff_Throttle(void);

void Encode_Msg_PID_Gain(unsigned char* telemetry_tx_buf, unsigned char id, float p, float i, float d);
void Encode_Msg_AHRS(unsigned char* telemetry_tx_buf);
void Encode_Msg_Altitude(unsigned char* telemetry_tx_buf);
void Encode_Msg_Gps(unsigned char* telemetry_tx_buf);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
#define MOTOR_FREQ_ADJUST 0.9f
#define BNO080_PITCH_OFFSET 2.38f
#define BNO080_ROLL_OFFSET -1.8f

float q[4];
float quatRadianAccuracy;
unsigned short iBus_SwA_Prev = 0;
unsigned char iBus_rx_cnt = 0;
unsigned char iBus_VrB_flag = 1; //0 : 1000~1100, 1 : 1100 ~ 1900, 2 : 1900 ~ 2000
unsigned char iBus_VrB_Prev_flag = 1;

float yaw_heading_reference;

unsigned char is_throttle_middle = 0;
unsigned char is_yaw_middle = 0;
unsigned short adcVal;

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM3_Init();
  MX_USART6_UART_Init();
  MX_SPI2_Init();
  MX_SPI1_Init();
  MX_UART5_Init();
  MX_TIM5_Init();
  MX_TIM7_Init();
  MX_USART1_UART_Init();
  MX_SPI3_Init();
  MX_I2C1_Init();
  MX_UART4_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  LL_TIM_EnableCounter(TIM3); //Buzzer

  LL_USART_EnableIT_RXNE(UART4); //GPS
  LL_USART_EnableIT_RXNE(UART5); //FS-iA6B;
  LL_USART_EnableIT_RXNE(USART6); //Debug UART

  HAL_UART_Receive_IT(&huart1, &uart1_rx_data, 1); // Telemetry

  LL_TIM_EnableCounter(TIM5); //Motor PWM
  LL_TIM_CC_EnableChannel(TIM5, LL_TIM_CHANNEL_CH1); //Enable Timer Counting
  LL_TIM_CC_EnableChannel(TIM5, LL_TIM_CHANNEL_CH2); //Enable Timer Counting
  LL_TIM_CC_EnableChannel(TIM5, LL_TIM_CHANNEL_CH3); //Enable Timer Counting
  LL_TIM_CC_EnableChannel(TIM5, LL_TIM_CHANNEL_CH4); //Enable Timer Counting

  LL_TIM_EnableCounter(TIM7); //10Hz, 50Hz, 1kHz loop
  LL_TIM_EnableIT_UPDATE(TIM7);


  TIM3->PSC = 1000;
  LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH4);
  HAL_Delay(60);
  LL_TIM_CC_DisableChannel(TIM3, LL_TIM_CHANNEL_CH4);
  HAL_Delay(60);
  LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH4);
  HAL_Delay(60);
  LL_TIM_CC_DisableChannel(TIM3, LL_TIM_CHANNEL_CH4);
  HAL_Delay(60);

  HAL_ADC_Start_DMA(&hadc1, &adcVal, 1); //battery

  printf("Checking sensor connection!\n");

  if(BNO080_Initialization() != 0)
  {
	  LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH4);

	  TIM3->PSC = 1000;
	  HAL_Delay(100);
	  TIM3->PSC = 1500;
	  HAL_Delay(100);
	  TIM3->PSC = 2000;
	  HAL_Delay(100);

	  LL_TIM_CC_DisableChannel(TIM3, LL_TIM_CHANNEL_CH4);

	  printf("\nBNO080 failed. Program shutting down...");
	  while(1)
	  {
		  LL_TIM_CC_DisableChannel(TIM3, LL_TIM_CHANNEL_CH4);
		  LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_0);
		  HAL_Delay(200);
		  LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH4);
		  LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_0);
		  HAL_Delay(200);
	  }
  }
  BNO080_enableRotationVector(2500);

  if(ICM20602_Initialization() !=0 )
  {
	  LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH4); //Enable Timer Counting

	  	  TIM3->PSC = 1000;
	  	  HAL_Delay(100);
	  	  TIM3->PSC = 1500;
	  	  HAL_Delay(100);
	  	  TIM3->PSC = 2000;
	  	  HAL_Delay(100);

	  	  LL_TIM_CC_DisableChannel(TIM3, LL_TIM_CHANNEL_CH4);

	  	  printf("\nICM20602 failed. Program shutting down...");
	  	  while(1)
	  	  {
	  		  LL_TIM_CC_DisableChannel(TIM3, LL_TIM_CHANNEL_CH4);
	  		  LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_1);
	  		  HAL_Delay(200);
	  		  LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH4);
	  		  LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_1);
	  		  HAL_Delay(200);
	  	  }
  }

  /*LPS22HH Initialization*/
  if(LPS22HH_Initialization() != 0)
    {
  	  LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH4);

  	  TIM3->PSC = 1000;
  	  HAL_Delay(100);
  	  TIM3->PSC = 1500;
  	  HAL_Delay(100);
  	  HAL_Delay(100);

  	  LL_TIM_CC_DisableChannel(TIM3, LL_TIM_CHANNEL_CH4);

  	  printf("\nLPS22HH failed. Program shutting down...");
  	  while(1)
  	  {
  		  LL_TIM_CC_DisableChannel(TIM3, LL_TIM_CHANNEL_CH4);
  		  LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_2);
  		  HAL_Delay(200);
  		  LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH4);
  		  LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_2);
  		  HAL_Delay(200);
  	  }
    }

  /*GNSS Initialization*/
  M8N_Initialization();

  // Correct ICM20602 bias
  ICM20602_Writebyte(0x13, (gyro_x_offset*-2)>>8);
  ICM20602_Writebyte(0x14, (gyro_x_offset*-2));

  ICM20602_Writebyte(0x15, (gyro_y_offset*-2)>>8);
  ICM20602_Writebyte(0x16, (gyro_y_offset*-2));

  ICM20602_Writebyte(0x17, (gyro_z_offset*-2)>>8);
  ICM20602_Writebyte(0x18, (gyro_z_offset*-2));

  printf("All sensor OK!\n\n");

  /*************Save Initial Gain into EEPROM**************/
  // Roll
  EP_PIDGain_Read(0, &roll.in.kp, &roll.in.ki, &roll.in.kd);
  Encode_Msg_PID_Gain(&telemetry_tx_buf[0], 0, roll.in.kp, roll.in.ki, roll.in.kd);
  HAL_UART_Transmit(&huart1, &telemetry_tx_buf[0], 20, 10);

  EP_PIDGain_Read(1, &roll.out.kp, &roll.out.ki, &roll.out.kd);
  Encode_Msg_PID_Gain(&telemetry_tx_buf[0], 1, roll.out.kp, roll.out.ki, roll.out.kd);
  HAL_UART_Transmit(&huart1, &telemetry_tx_buf[0], 20, 10);
  //Pitch
  EP_PIDGain_Read(2, &pitch.in.kp, &pitch.in.ki, &pitch.in.kd);
  Encode_Msg_PID_Gain(&telemetry_tx_buf[0], 2, pitch.in.kp, pitch.in.ki, pitch.in.kd);
  HAL_UART_Transmit(&huart1, &telemetry_tx_buf[0], 20, 10);

  EP_PIDGain_Read(3, &pitch.out.kp, &pitch.out.ki, &pitch.out.kd);
  Encode_Msg_PID_Gain(&telemetry_tx_buf[0], 3, pitch.out.kp, pitch.out.ki, pitch.out.kd);
  HAL_UART_Transmit(&huart1, &telemetry_tx_buf[0], 20, 10);
  // Yaw
  EP_PIDGain_Read(4, &yaw_heading.kp, &yaw_heading.ki, &yaw_heading.kd);
  Encode_Msg_PID_Gain(&telemetry_tx_buf[0], 4, yaw_heading.kp, yaw_heading.ki, yaw_heading.kd);
  HAL_UART_Transmit(&huart1, &telemetry_tx_buf[0], 20, 10);

  EP_PIDGain_Read(5, &yaw_rate.kp, &yaw_rate.ki, &yaw_rate.kd);
  Encode_Msg_PID_Gain(&telemetry_tx_buf[0], 5, yaw_rate.kp, yaw_rate.ki, yaw_rate.kd);
  HAL_UART_Transmit(&huart1, &telemetry_tx_buf[0], 20, 10);
  // Altitude Gain
  EP_PIDGain_Read(6, &altitude.in.kp, &altitude.in.ki, &altitude.in.kd);
  Encode_Msg_PID_Gain(&telemetry_tx_buf[0], 6, altitude.in.kp, altitude.in.ki, altitude.in.kd);
  HAL_UART_Transmit(&huart1, &telemetry_tx_buf[0], 19, 10);

  EP_PIDGain_Read(7, &altitude.out.kp, &altitude.out.ki, &altitude.out.kd);
  Encode_Msg_PID_Gain(&telemetry_tx_buf[0], 7, altitude.out.kp, altitude.out.ki, altitude.out.kd);
  HAL_UART_Transmit(&huart1, &telemetry_tx_buf[0], 19, 10);
  // Latitude
  EP_PIDGain_Read(8, &lat.in.kp, &lat.in.ki, &lat.in.kd);
  Encode_Msg_PID_Gain(&telemetry_tx_buf[0], 8, lat.in.kp, lat.in.ki, lat.in.kd);
  HAL_UART_Transmit(&huart1, &telemetry_tx_buf[0], 20, 10);

  EP_PIDGain_Read(9, &lat.out.kp, &lat.out.ki, &lat.out.kd);
  Encode_Msg_PID_Gain(&telemetry_tx_buf[0], 9, lat.out.kp, lat.out.ki, lat.out.kd);
  HAL_UART_Transmit(&huart1, &telemetry_tx_buf[0], 19, 10);
  // Longitude
  EP_PIDGain_Read(10, &lon.in.kp, &lon.in.ki, &lon.in.kd);
  Encode_Msg_PID_Gain(&telemetry_tx_buf[0], 10, lon.in.kp, lon.in.ki, lon.in.kd);
  HAL_UART_Transmit(&huart1, &telemetry_tx_buf[0], 19, 10);

  EP_PIDGain_Read(11, &lon.out.kp, &lon.out.ki, &lon.out.kd);
  Encode_Msg_PID_Gain(&telemetry_tx_buf[0], 11, lon.out.kp, lon.out.ki, lon.out.kd);
  HAL_UART_Transmit(&huart1, &telemetry_tx_buf[0], 19, 10);

// Altitude Hold PID Gain
//altitude.out.kp = 2;
//altitude.out.ki = 0;
//altitude.out.kd = 0.01;
//altitude.in.kp = 1000;
//altitude.in.ki = 10;
//altitude.in.kd = 0;
//
//// GPS Hold PID Gain
//lat.out.kp = 0.01;
//lat.out.ki = 0.f;
//lat.out.kd = 0.001;
//
//lat.in.kp = 1;
//lat.in.ki = 1;
//lat.in.kd = 0.f;
//
//lon.out.kp = 0.01;
//lon.out.ki = 0.f;
//lon.out.kd = 0.001;
//
//lon.in.kp = 1;
//lon.in.ki = 1;
//lon.in.kd = 0;

/*Receiver Detection*/
  while(Is_iBus_Received() == 0)
  {
	  LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH4); //Enable Timer Counting
	  TIM3->PSC = 3000;
	  HAL_Delay(200);
	  LL_TIM_CC_DisableChannel(TIM3, LL_TIM_CHANNEL_CH4);

	  HAL_Delay(200);
  }

  /**************************ESC Calibration***********************************/
  if(iBus.SwC == 2000)
  {
	  LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH4); //Enable Timer Counting
	  TIM3->PSC = 1500;
	  HAL_Delay(500);
	  TIM3->PSC = 2000;
	  HAL_Delay(500);
	  TIM3->PSC = 1500;
	  HAL_Delay(500);
	  TIM3->PSC = 2000;
	  HAL_Delay(500);
	  LL_TIM_CC_DisableChannel(TIM3, LL_TIM_CHANNEL_CH4);

	  ESC_Calibration();
	  while(iBus.SwC != 1000)
	  {
		  Is_iBus_Received();

		  LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH4);
		  TIM3->PSC = 1500;
		  HAL_Delay(200);
		  TIM3->PSC = 2000;
		  HAL_Delay(200);
		  LL_TIM_CC_DisableChannel(TIM3, LL_TIM_CHANNEL_CH4);
	  }
  }
  /**************************BNO080 Calibration********************************/
  else if(iBus.SwC == 1500)
  {
	  LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH4); //Enable Timer Counting
	  TIM3->PSC = 1500;
	  HAL_Delay(500);
	  TIM3->PSC = 2000;
	  HAL_Delay(500);
	  TIM3->PSC = 1500;
	  HAL_Delay(500);
	  TIM3->PSC = 2000;
	  HAL_Delay(500);
	  LL_TIM_CC_DisableChannel(TIM3, LL_TIM_CHANNEL_CH4);

	  BNO080_Calibration();
	  while(iBus.SwC != 1000)
	  	  {
	  		  Is_iBus_Received();

	  		  LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH4);
	  		  TIM3->PSC = 1500;
	  		  HAL_Delay(200);
	  		  TIM3->PSC = 2000;
	  		  HAL_Delay(200);
	  		  LL_TIM_CC_DisableChannel(TIM3, LL_TIM_CHANNEL_CH4);
	  	  }
  }

  /*********************Check Throttle value is minimum************************/
  while(Is_iBus_Throttle_min() == 0 || iBus.SwA == 2000)
  {
	  LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH4); //Enable Timer Counting
	  TIM3->PSC = 1000;
	  HAL_Delay(70);
	  LL_TIM_CC_DisableChannel(TIM3, LL_TIM_CHANNEL_CH4);
	  HAL_Delay(70);
  }

  /*LPS22HH Initial Offset*/
  for(int i=0; i<20; i++)
  {
	  if(LPS22HH_DataReady() == 1)
	  {
		  LPS22HH_GetPressure(&LPS22HH.pressure_raw);
		  LPS22HH_GetTemperature(&LPS22HH.temperature_raw);

		  //Default Unit = 1m
		  LPS22HH.baroAlt = getAltitude1(LPS22HH.pressure_raw/4096.f/*, LPS22HH.temperature_raw/100.f*/);
		  baro_offset += LPS22HH.baroAlt;
		  HAL_Delay(100);

		  baro_cnt++;
	  }
  }
  baro_offset = baro_offset / baro_cnt;

//  batVolt = adcVal * 0.00699563f;

  /********************* FC Ready to Fly ************************/

  LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH4); //Enable Timer Counting

  TIM3->PSC = 2000;
  HAL_Delay(100);
  TIM3->PSC = 1500;
  HAL_Delay(100);
  TIM3->PSC = 1000;
  HAL_Delay(100);

  LL_TIM_CC_DisableChannel(TIM3, LL_TIM_CHANNEL_CH4);


  printf("Start\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  /********************* NX Message Parsing ************************/
//	  if(nx_rx_cplt_flag==1)
//	  {
//		  nx_rx_cplt_flag=0;
//
//		  XAVIER_Parsing(&nx_rx_buf, &XAVIER);
//	  }

	  /********************* Telemetry Communication ************************/
	  Receive_Pid_Gain();

	  /********************* Flight Mode Detection / ESC Control / PID Calculation ************************/
	  if(tim7_1ms_flag==1)
	  {
		  tim7_1ms_flag = 0;

		  lat_gps += lat_add;
		  lon_gps += lon_add;

		  Double_Roll_Pitch_PID_Calculation(&pitch, (iBus.RV + gps_pitch_adjust - 1500)*0.07f, BNO080_Pitch, ICM20602.gyro_x);
		  Double_Roll_Pitch_PID_Calculation(&roll, (iBus.RH + gps_roll_adjust - 1500)*0.07f, BNO080_Roll, ICM20602.gyro_y);

		  gps_pitch_adjust = 0;
		  gps_roll_adjust = 0;

		  // Choose Flight Mode
		  if(iBus.LV < 1550 && iBus.LV > 1400) is_throttle_middle = 1;
		  else is_throttle_middle = 0;

		  if(iBus.LH > 1485 && iBus.LH < 1515) is_yaw_middle = 1;
		  else is_yaw_middle = 0;

		  flight_mode = 1;
		  if(iBus.SwA == 2000 && iBus.SwB == 1000 && iBus.SwD == 2000 && is_throttle_middle == 1) flight_mode = 2;
		  else if(iBus.SwA == 2000 && iBus.SwB == 2000 && is_throttle_middle == 1 && (pvt.fixType == 2 || pvt.fixType == 3)) flight_mode = 3;
		  else if(iBus.SwA == 2000 && iBus.VrB > 1900) flight_mode = 4;

		  if(flight_mode == 2) //Altitude Holding Mode
		  {
//			  if(iBus.VrB < 1100) iBus_VrB_flag = 0;
//			  else if(iBus.VrB > 1900) iBus_VrB_flag = 2;
//			  else iBus_VrB_flag = 1;
//
//			  if(iBus_VrB_flag==0 && iBus_VrB_Prev_flag==1) altitude_setpoint -= 0.5f;
//			  else if(iBus_VrB_flag==2 && iBus_VrB_Prev_flag==1) altitude_setpoint += 0.5f;
//
//			  iBus_VrB_Prev_flag = iBus_VrB_flag;

			  Double_Altitude_PID_Calculation(&altitude, altitude_setpoint, actual_pressure_fast);

			  if(is_yaw_middle == 0)
			  {
				  yaw_heading_reference = BNO080_Yaw;
				  Single_Yaw_Rate_PID_Calculation(&yaw_rate, (iBus.LH-1500), ICM20602.gyro_z);
				  ccr1 = 84000 + takeoff_throttle - pitch.in.pid_result + roll.in.pid_result -yaw_rate.pid_result+altitude.in.pid_result;
				  ccr2 = 84000 + takeoff_throttle + pitch.in.pid_result + roll.in.pid_result +yaw_rate.pid_result+altitude.in.pid_result;
				  ccr3 = 84000 + takeoff_throttle + pitch.in.pid_result - roll.in.pid_result -yaw_rate.pid_result+altitude.in.pid_result;
				  ccr4 = 84000 + takeoff_throttle - pitch.in.pid_result - roll.in.pid_result +yaw_rate.pid_result+altitude.in.pid_result;
				  ccr2 = (unsigned int)((float)ccr2 * MOTOR_FREQ_ADJUST);
			  }
			  else
			  {
				  Single_Yaw_Heading_PID_Calculation(&yaw_heading, yaw_heading_reference, BNO080_Yaw, ICM20602.gyro_z);
				  ccr1 = 84000 + takeoff_throttle - pitch.in.pid_result + roll.in.pid_result - yaw_heading.pid_result + altitude.in.pid_result;
				  ccr2 = 84000 + takeoff_throttle + pitch.in.pid_result + roll.in.pid_result + yaw_heading.pid_result + altitude.in.pid_result;
				  ccr3 = 84000 + takeoff_throttle + pitch.in.pid_result - roll.in.pid_result - yaw_heading.pid_result + altitude.in.pid_result;
				  ccr4 = 84000 + takeoff_throttle - pitch.in.pid_result - roll.in.pid_result + yaw_heading.pid_result + altitude.in.pid_result;
				  ccr2 = (unsigned int)((float)ccr2 * MOTOR_FREQ_ADJUST);
			  }
		  }
		  else if(flight_mode == 3 ) //GPS holding Mode
		  {
			  Double_Altitude_PID_Calculation(&altitude, altitude_setpoint, actual_pressure_fast);

			  Double_GPS_PID_Calculation(&lat, lat_waypoint, lat_gps);
			  Double_GPS_PID_Calculation(&lon, lon_waypoint, lon_gps);

			  //Because the correction is calculated as if the nose was facing north, we need to convert it for the current heading.
			  gps_roll_adjust = ((float)lon.in.pid_result * cos((360.f - BNO080_Yaw) * 0.017453)) + ((float)lat.in.pid_result * sin((360.f - BNO080_Yaw) * 0.017453));
			  gps_pitch_adjust = ((float)lat.in.pid_result * cos((360.f - BNO080_Yaw) * 0.017453)) - ((float)lon.in.pid_result * sin((360.f - BNO080_Yaw) * 0.017453));

			  //Limit the maximum correction to 300. This way we still have full control with the pitch and roll stick on the transmitter.
			  if (gps_roll_adjust > GPS_PD_MAX) gps_roll_adjust = GPS_PD_MAX;
			  if (gps_roll_adjust < GPS_PD_MIN) gps_roll_adjust = GPS_PD_MIN;
			  if (gps_pitch_adjust > GPS_PD_MAX) gps_pitch_adjust = GPS_PD_MAX;
			  if (gps_pitch_adjust < GPS_PD_MIN) gps_pitch_adjust = GPS_PD_MIN;

			  if(is_yaw_middle == 0)
			  {
				  yaw_heading_reference = BNO080_Yaw;
				  Single_Yaw_Rate_PID_Calculation(&yaw_rate, (iBus.LH-1500), ICM20602.gyro_z);
				  ccr1 = 84000 + takeoff_throttle - pitch.in.pid_result + roll.in.pid_result - yaw_rate.pid_result + altitude.in.pid_result;
				  ccr2 = 84000 + takeoff_throttle + pitch.in.pid_result + roll.in.pid_result + yaw_rate.pid_result + altitude.in.pid_result;
				  ccr3 = 84000 + takeoff_throttle + pitch.in.pid_result - roll.in.pid_result - yaw_rate.pid_result + altitude.in.pid_result;
				  ccr4 = 84000 + takeoff_throttle - pitch.in.pid_result - roll.in.pid_result + yaw_rate.pid_result + altitude.in.pid_result;
				  ccr2 = (unsigned int)((float)ccr2 * MOTOR_FREQ_ADJUST);
			  }
			  else
			  {
				  Single_Yaw_Heading_PID_Calculation(&yaw_heading, yaw_heading_reference , BNO080_Yaw, ICM20602.gyro_z);
				  ccr1 = 84000 + takeoff_throttle - pitch.in.pid_result + roll.in.pid_result - yaw_heading.pid_result + altitude.in.pid_result;
				  ccr2 = 84000 + takeoff_throttle + pitch.in.pid_result + roll.in.pid_result + yaw_heading.pid_result + altitude.in.pid_result;
				  ccr3 = 84000 + takeoff_throttle + pitch.in.pid_result - roll.in.pid_result - yaw_heading.pid_result + altitude.in.pid_result;
				  ccr4 = 84000 + takeoff_throttle - pitch.in.pid_result - roll.in.pid_result + yaw_heading.pid_result + altitude.in.pid_result;
				  ccr2 = (unsigned int)((float)ccr2 * MOTOR_FREQ_ADJUST);
			  }
		  }
		  else if(flight_mode == 4 ) // Return to Home Mode
		  {
			  Double_Altitude_PID_Calculation(&altitude, altitude_setpoint, actual_pressure_fast);

			  if(altitude.out.error < 0.1 && altitude.out.error > -0.1) altitude_setpoint += 0.3;
			  if(altitude_setpoint > 5) altitude_setpoint = 5;

			  Double_GPS_PID_Calculation(&lat, lat_waypoint, lat_gps);
			  Double_GPS_PID_Calculation(&lon, lon_waypoint, lon_gps);

			  //Because the correction is calculated as if the nose was facing north, we need to convert it for the current heading.
			  gps_roll_adjust = ((float)lon.in.pid_result * cos((360.f - BNO080_Yaw) * 0.017453)) + ((float)lat.in.pid_result * sin((360.f - BNO080_Yaw) * 0.017453));
			  gps_pitch_adjust = ((float)lat.in.pid_result * cos((360.f - BNO080_Yaw) * 0.017453)) - ((float)lon.in.pid_result * sin((360.f - BNO080_Yaw) * 0.017453));

			  //Limit the maximum correction to 300. This way we still have full control with the pitch and roll stick on the transmitter.
			  if (gps_roll_adjust > GPS_PD_MAX) gps_roll_adjust = GPS_PD_MAX;
			  if (gps_roll_adjust < GPS_PD_MIN) gps_roll_adjust = GPS_PD_MIN;
			  if (gps_pitch_adjust > GPS_PD_MAX) gps_pitch_adjust = GPS_PD_MAX;
			  if (gps_pitch_adjust < GPS_PD_MIN) gps_pitch_adjust = GPS_PD_MIN;

			  Single_Yaw_Heading_PID_Calculation(&yaw_heading, yaw_heading_reference , BNO080_Yaw, ICM20602.gyro_z);
			  ccr1 = 84000 + takeoff_throttle - pitch.in.pid_result + roll.in.pid_result - yaw_heading.pid_result + altitude.in.pid_result;
			  ccr2 = 84000 + takeoff_throttle + pitch.in.pid_result + roll.in.pid_result + yaw_heading.pid_result + altitude.in.pid_result;
			  ccr3 = 84000 + takeoff_throttle + pitch.in.pid_result - roll.in.pid_result - yaw_heading.pid_result + altitude.in.pid_result;
			  ccr4 = 84000 + takeoff_throttle - pitch.in.pid_result - roll.in.pid_result + yaw_heading.pid_result + altitude.in.pid_result;
			  ccr2 = (unsigned int)((float)ccr2 * MOTOR_FREQ_ADJUST);
		  }
		  else// Default Manual Mode
		  {
			  if(is_yaw_middle == 0)
			  {
				  yaw_heading_reference = BNO080_Yaw;
				  Single_Yaw_Rate_PID_Calculation(&yaw_rate, (iBus.LH-1500), ICM20602.gyro_z);
				  ccr1 = 84000 + (iBus.LV - 1000) * 83.9 - pitch.in.pid_result + roll.in.pid_result -yaw_rate.pid_result;
				  ccr2 = 84000 + (iBus.LV - 1000) * 83.9 + pitch.in.pid_result + roll.in.pid_result +yaw_rate.pid_result;
				  ccr3 = 84000 + (iBus.LV - 1000) * 83.9 + pitch.in.pid_result - roll.in.pid_result -yaw_rate.pid_result;
				  ccr4 = 84000 + (iBus.LV - 1000) * 83.9 - pitch.in.pid_result - roll.in.pid_result +yaw_rate.pid_result;
				  ccr2 = (unsigned int)((float)ccr2 * MOTOR_FREQ_ADJUST);
			  }
			  else
			  {
				  Single_Yaw_Heading_PID_Calculation(&yaw_heading, yaw_heading_reference, BNO080_Yaw, ICM20602.gyro_z);
				  ccr1 = 84000 + (iBus.LV - 1000) * 83.9 - pitch.in.pid_result + roll.in.pid_result -yaw_heading.pid_result;
				  ccr2 = 84000 + (iBus.LV - 1000) * 83.9 + pitch.in.pid_result + roll.in.pid_result +yaw_heading.pid_result;
				  ccr3 = 84000 + (iBus.LV - 1000) * 83.9 + pitch.in.pid_result - roll.in.pid_result -yaw_heading.pid_result;
				  ccr4 = 84000 + (iBus.LV - 1000) * 83.9 - pitch.in.pid_result - roll.in.pid_result +yaw_heading.pid_result;
				  ccr2 = (unsigned int)((float)ccr2 * MOTOR_FREQ_ADJUST);
			  }

			  altitude_setpoint = actual_pressure_fast;
			  Reset_PID_Integrator(&altitude.out);
			  Reset_PID_Integrator(&altitude.in);

			  lat_waypoint = lat_gps;
			  lon_waypoint = lon_gps;
			  Reset_PID_Integrator(&lat.out);
			  Reset_PID_Integrator(&lat.in);
			  Reset_PID_Integrator(&lon.out);
			  Reset_PID_Integrator(&lon.in);
		  }
	  }


	  if(iBus.LV < 1030 || motor_arming_flag == 0)
	  {
		  Reset_All_PID_Integrator();
	  }


	  /********************* Motor Arming State ************************/
	  if(iBus.SwA == 2000 && iBus_SwA_Prev != 2000)
	  {
		  if(iBus.LV < 1010)
		  {
			  motor_arming_flag = 1;
			  yaw_heading_reference = BNO080_Yaw;
		  }
		  else
		  {
			  while(Is_iBus_Throttle_min() == 0 || iBus.SwA == 2000)
			  {
				  LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH4); //Enable Timer Counting
				  TIM3->PSC = 1000;
				  HAL_Delay(70);
				  LL_TIM_CC_DisableChannel(TIM3, LL_TIM_CHANNEL_CH4);
				  HAL_Delay(70);
			  }
		  }
	  }
	  iBus_SwA_Prev = iBus.SwA;
	  if(iBus.SwA != 2000)
	  {
		  motor_arming_flag = 0;
	  }

	  // Write Motor PWM
	  if(motor_arming_flag == 1)
	  {
		  if(failsafe_flag == 0)
		  {
			  if(iBus.LV > 1050)
			  {
				  TIM5->CCR1 = ccr1 > 167999 ? 167999 : ccr1 < 84000 ? 84000 : ccr1;
				  TIM5->CCR2 = ccr2 > 167999 ? 167999 : ccr2 < 84000 ? 84000 : ccr2;
				  TIM5->CCR3 = ccr3 > 167999 ? 167999 : ccr3 < 84000 ? 84000 : ccr3;
				  TIM5->CCR4 = ccr4 > 167999 ? 167999 : ccr4 < 84000 ? 84000 : ccr4;
			  }
			  else
			  {
				  TIM5->CCR1 = 84000;
				  TIM5->CCR2 = 84000;
				  TIM5->CCR3 = 84000;
				  TIM5->CCR4 = 84000;
			  }
		  }
		  else
		  {
			  TIM5->CCR1 = 84000;
			  TIM5->CCR2 = 84000;
			  TIM5->CCR3 = 84000;
			  TIM5->CCR4 = 84000;
		  }
	  }

	  else
	  {
		  TIM5->CCR1 = 84000;
		  TIM5->CCR2 = 84000;
		  TIM5->CCR3 = 84000;
		  TIM5->CCR4 = 84000;
	  }


	  /********************* Telemetry Communication ************************/
	  if(tim7_10ms_flag == 1)
	  {
		  tim7_10ms_flag = 0;
	  }

	  if(tim7_20ms_flag == 1 && tim7_100ms_flag == 0)
	  {
		  tim7_20ms_flag = 0;
		  if(motor_arming_flag == 1)
		  {
	  		  Encode_Msg_Gps(&telemetry_tx_buf[0]);
	  		  HAL_UART_Transmit_DMA(&huart1, &telemetry_tx_buf[0], 55); // altitude : 26, gps : 35, pid : 75
//			  Encode_Msg_Temp(&telemetry_tx_buf[0]);
//			  HAL_UART_Transmit_DMA(&huart1, &telemetry_tx_buf[0], 28); // altitude : 28, gps : 35, pid : 210
			  //		  Encode_Msg_AHRS(&telemetry_tx_buf[0]);
			  //		  HAL_UART_Transmit_IT(&huart1, &telemetry_tx_buf[0], 20);
		  }

	  }
	  else if(tim7_20ms_flag == 1 && tim7_100ms_flag == 1)
	  {
		  tim7_20ms_flag = 0;
		  tim7_100ms_flag = 0;
		  if(motor_arming_flag == 1)
		  {
			  //			  Encode_Msg_PID(&telemetry_tx_buf[0]);
			  //			  HAL_UART_Transmit_DMA(&huart1, &telemetry_tx_buf[0], 210); // altitude : 28, gps : 35, pid : 201
			  //		  Encode_Msg_AHRS(&telemetry_tx_buf[0]);
			  //		  HAL_UART_Transmit_IT(&huart1, &telemetry_tx_buf[0], 40);
			  //		  Encode_Msg_Altitude(&telemetry_tx_buf[0]);

		  }
	  }

	  /***********************************************************************************************
	----------------------------Check BNO080 Sensor Value(current Angle Data)-----------------------
	   ***********************************************************************************************/
	  if(BNO080_dataAvailable() == 1)
	  {
		  q[0] = BNO080_getQuatI();
		  q[1] = BNO080_getQuatJ();
		  q[2] = BNO080_getQuatK();
		  q[3] = BNO080_getQuatReal();
		  quatRadianAccuracy = BNO080_getQuatRadianAccuracy();

		  Quaternion_Update(&q[0]);

		  BNO080_Roll = -BNO080_Roll;
		  BNO080_Roll -= BNO080_ROLL_OFFSET;
		  BNO080_Pitch = -BNO080_Pitch;
		  BNO080_Pitch -= BNO080_PITCH_OFFSET;
		  BNO080_Yaw -= DECLINATION;
	  }

	  /***********************************************************************************************
	----------------------Check ICM20602 Sensor Value(current Angular Velocity Data)------------------
	   ***********************************************************************************************/
	  if(ICM20602_DataReady() == 1)
	  {
		  ICM20602_Get3AxisGyroRawData(&ICM20602.gyro_x_raw);

		  // Gyro FS=2 (+500dps max)
		  ICM20602.gyro_x = ICM20602.gyro_x_raw / 65.5f;
		  ICM20602.gyro_y = ICM20602.gyro_y_raw / 65.5f;
		  ICM20602.gyro_z = ICM20602.gyro_z_raw / 65.5f;

		  ICM20602.gyro_x = -ICM20602.gyro_x;
		  ICM20602.gyro_z = -ICM20602.gyro_z;
	  }

	  if(LPS22HH_DataReady() == 1)
	  {
		  LPS22HH_GetPressure(&LPS22HH.pressure_raw);
		  LPS22HH_GetTemperature(&LPS22HH.temperature_raw);

		  LPS22HH.baroAlt = getAltitude1(LPS22HH.pressure_raw/4096.f/*, LPS22HH.temperature_raw/100.f*/); //Default Unit = 1m
		  LPS22HH.baroAltGround = LPS22HH.baroAlt - baro_offset;

		  //moving average of altitude
		  pressure_total_average -= pressure_rotating_mem[pressure_rotating_mem_location];
		  pressure_rotating_mem[pressure_rotating_mem_location] = LPS22HH.baroAltGround;
		  pressure_total_average += pressure_rotating_mem[pressure_rotating_mem_location];
		  pressure_rotating_mem_location++;
		  if(pressure_rotating_mem_location == 5) pressure_rotating_mem_location = 0;
		  actual_pressure_fast = pressure_total_average / 5.0f;
	  }

	  if(m8n_rx_cplt_flag == 1) // GPS receive checking
	  {
		  m8n_rx_cplt_flag = 0;

		  if(M8N_UBX_CHKSUM_Check(&m8n_rx_buf[0], 100) == 1)
		  {
			  M8N_UBX_NAV_PVT_Parsing(&m8n_rx_buf[0], &pvt);

			  if(pvt.fixType == 2 || pvt.fixType == 3)
			  {
				  if(lat_gps_previous == 0 || lon_gps_previous == 0)
				  {
					  lat_gps_previous = (double)pvt.lat;
					  lon_gps_previous = (double)pvt.lon;
				  }
				  else
				  {
					  lat_gps_previous = lat_gps_actual;
					  lon_gps_previous = lon_gps_actual;
				  }

				  if((double)pvt.lat > lat_gps_previous)lat_diff = (double)pvt.lat - lat_gps_previous;
				  else lat_diff = lat_gps_previous - (double)pvt.lat;
				  if((double)pvt.lon > lon_gps_previous)lon_diff = (double)pvt.lon - lon_gps_previous;
				  else lon_diff = lon_gps_previous - (double)pvt.lon;

				  if(lat_diff < 10000) lat_gps_actual = (double)pvt.lat;
				  if(lon_diff < 10000) lon_gps_actual = (double)pvt.lon;
			  }
		  }
		  else
		  {
			  lat_gps_actual = lat_gps_previous;
			  lon_gps_actual = lon_gps_previous;
		  }

		  lat_add = (lat_gps_actual - lat_gps_previous) / 200.00;
		  lon_add = (lon_gps_actual - lon_gps_previous) / 200.00;

		  lat_gps = lat_gps_previous;
		  lon_gps = lon_gps_previous;
	  }


	  /***********************************************************************************************
	------------------------------Toggle Led if Checksum Data is right------------------------------
	   ***********************************************************************************************/
	  // Transmitter - Receiver failsafe
	  if(ibus_rx_cplt_flag==1)
	  {
		  ibus_rx_cplt_flag=0;
		  if(iBus_Check_CHKSUM(&ibus_rx_buf[0],32)==1)
		  {
			  LL_GPIO_TogglePin(GPIOC,LL_GPIO_PIN_2);

			  iBus_Parsing(&ibus_rx_buf, &iBus);
			  iBus_rx_cnt++;

			  if(iBus_isActiveFailSafe(&iBus) == 1)
			  {
				  failsafe_flag = 1;
			  }
			  else
			  {
				  failsafe_flag = 0;
			  }
		  }
	  }

	  // FC - Receiver failsafe
	  if(tim7_1000ms_flag == 1)
	  {
		  tim7_1000ms_flag = 0;
		  if(iBus_rx_cnt == 0)
		  {
			  failsafe_flag = 2;
		  }
		  iBus_rx_cnt = 0;
	  }

	  if(failsafe_flag == 1 || failsafe_flag ==2 || low_bat_flag == 1)
	  {
		  LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH4); //Buzzer On
	  }
	  else
	  {
		  LL_TIM_CC_DisableChannel(TIM3, LL_TIM_CHANNEL_CH4); //Buzzer Off
	  }

//	  if(batVolt == 0) batVolt = adcVal * 0.00699563f;
//	  batVolt = 0.98 * batVolt_prev + 0.02 * (adcVal * 0.00699563f);
//	  if(batVolt < 18) batVolt = 18;
//	  batVolt_prev = batVolt;
	  Calculate_Takeoff_Throttle();
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
int abs_int(int a, int b)
{
	if(a > b) return (a-b);
	else return (b-a);
}

float abs_float(float a, float b)
{
	if(a > b) return (a-b);
	else return (b-a);
}

int Is_iBus_Throttle_min(void)
{
	if(ibus_rx_cplt_flag==1)
	{
		ibus_rx_cplt_flag=0;
		if(iBus_Check_CHKSUM(&ibus_rx_buf[0],32)==1)
		{
			iBus_Parsing(&ibus_rx_buf[0], &iBus);
			if(iBus.LV < 1010) return 1;
		}
	}

	return 0;
}

void ESC_Calibration(void)
{
	  TIM5->CCR1 = 167999;
	  TIM5->CCR2 = 167999;
	  TIM5->CCR3 = 167999;
	  TIM5->CCR4 = 167999;
	  HAL_Delay(7000);

	  TIM5->CCR1 = 84000;
	  TIM5->CCR2 = 84000;
	  TIM5->CCR3 = 84000;
	  TIM5->CCR4 = 84000;
	  HAL_Delay(8000);
}

int Is_iBus_Received(void)
{
	if(ibus_rx_cplt_flag==1)
		{
			ibus_rx_cplt_flag=0;
			if(iBus_Check_CHKSUM(&ibus_rx_buf[0],32)==1)
			{
				iBus_Parsing(&ibus_rx_buf[0], &iBus);
				return 1;
			}
		}
		return 0;
}

void BNO080_Calibration(void)
{
	//Resets BNO080 to disable All output
	BNO080_Initialization();

	//BNO080/BNO085 Configuration
	//Enable dynamic calibration for accelerometer, gyroscope, and magnetometer
	//Enable Game Rotation Vector output
	//Enable Magnetic Field output
	BNO080_calibrateAll(); //Turn on cal for Accel, Gyro, and Mag
	BNO080_enableGameRotationVector(20000); //Send data update every 20ms (50Hz)
	BNO080_enableMagnetometer(20000); //Send data update every 20ms (50Hz)

	//Once magnetic field is 2 or 3, run the Save DCD Now command
  	printf("Calibrating BNO080. Pull up FS-i6 SWC to end calibration and save to flash\n");
  	printf("Output in form x, y, z, in uTesla\n\n");

	//while loop for calibration procedure
	//Iterates until iBus.SwC is mid point (1500)
	//Calibration procedure should be done while this loop is in iteration.
	while(iBus.SwC == 1500)
	{
		if(BNO080_dataAvailable() == 1)
		{
			//Observing the status bit of the magnetic field output
			float x = BNO080_getMagX();
			float y = BNO080_getMagY();
			float z = BNO080_getMagZ();
			unsigned char accuracy = BNO080_getMagAccuracy();

			float quatI = BNO080_getQuatI();
			float quatJ = BNO080_getQuatJ();
			float quatK = BNO080_getQuatK();
			float quatReal = BNO080_getQuatReal();
			unsigned char sensorAccuracy = BNO080_getQuatAccuracy();




			printf("%f,%f,%f,", x, y, z);
			if (accuracy == 0) printf("Unreliable\t");
			else if (accuracy == 1) printf("Low\t");
			else if (accuracy == 2) printf("Medium\t");
			else if (accuracy == 3) printf("High\t");

			printf("\t%f,%f,%f,%f,", quatI, quatI, quatI, quatReal);
			if (sensorAccuracy == 0) printf("Unreliable\n");
			else if (sensorAccuracy == 1) printf("Low\n");
			else if (sensorAccuracy == 2) printf("Medium\n");
			else if (sensorAccuracy == 3) printf("High\n");

			//Turn the LED and buzzer on when both accuracy and sensorAccuracy is high
			if(accuracy == 3 && sensorAccuracy == 3)
			{
				LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_0 | LL_GPIO_PIN_1 | LL_GPIO_PIN_2);
				TIM3->PSC = 65000; //Very low frequency
				LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH4);
			}
			else
			{
				LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_0 | LL_GPIO_PIN_1 | LL_GPIO_PIN_2);
				LL_TIM_CC_DisableChannel(TIM3, LL_TIM_CHANNEL_CH4);
			}
		}

		Is_iBus_Received(); //Refreshes iBus Data for iBus.SwC
		HAL_Delay(100);
	}

	//Ends the loop when iBus.SwC is not mid point
	//Turn the LED and buzzer off
	LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_0 | LL_GPIO_PIN_1 | LL_GPIO_PIN_2);
	LL_TIM_CC_DisableChannel(TIM3, LL_TIM_CHANNEL_CH4);

	//Saves the current dynamic calibration data (DCD) to memory
	//Sends command to get the latest calibration status
	BNO080_saveCalibration();
	BNO080_requestCalibrationStatus();

	//Wait for calibration response, timeout if no response
	int counter = 100;
	while(1)
	{
		if(--counter == 0) break;
		if(BNO080_dataAvailable())
		{
			//The IMU can report many different things. We must wait
			//for the ME Calibration Response Status byte to go to zero
			if(BNO080_calibrationComplete() == 1)
			{

				printf("\nCalibration data successfully stored\n");
				LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH4);
				TIM3->PSC = 2000;
				HAL_Delay(300);
				TIM3->PSC = 1500;
				HAL_Delay(300);
				LL_TIM_CC_DisableChannel(TIM3, LL_TIM_CHANNEL_CH4);
				HAL_Delay(1000);
				break;
			}
		}
		HAL_Delay(10);
	}
	if(counter == 0)
	{
		printf("\nCalibration data failed to store. Please try again.\n");
		LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH4);
		TIM3->PSC = 1500;
		HAL_Delay(300);
		TIM3->PSC = 2000;
		HAL_Delay(300);
		LL_TIM_CC_DisableChannel(TIM3, LL_TIM_CHANNEL_CH4);
		HAL_Delay(1000);
	}

	//BNO080_endCalibration(); //Turns off all calibration
	//In general, calibration should be left on at all times. The BNO080
	//auto-calibrates and auto-records cal data roughly every 5 minutes

	//Resets BNO080 to disable Game Rotation Vector and Magnetometer
	//Enables Rotation Vector
	BNO080_Initialization();
	BNO080_enableRotationVector(2500); //Send data update every 2.5ms (400Hz)
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	static unsigned char cnt = 0;
	if(huart->Instance = USART1)
	{
		HAL_UART_Receive_IT(&huart1, &uart1_rx_data, 1);

		switch(cnt)
				{
				case 0:
					if(uart1_rx_data==0x46)
					{
						telemetry_rx_buf[cnt]=uart1_rx_data;
						cnt++;
					}
					break;
				case 1:
					if(uart1_rx_data==0x43)
					{
						telemetry_rx_buf[cnt]=uart1_rx_data;
						cnt++;
					}
					else
						cnt=0;
					break;

				case 19:
					telemetry_rx_buf[cnt]=uart1_rx_data;
					cnt=0;
					telemetry_rx_cplt_flag = 1;
					break;

				default:
					telemetry_rx_buf[cnt]=uart1_rx_data;
					cnt++;
					break;
				}
	}

}

void Encode_Msg_AHRS(unsigned char* telemetry_tx_buf)
{
	  telemetry_tx_buf[0] = 0x46;
	  telemetry_tx_buf[1] = 0x43;
	  telemetry_tx_buf[2] = 0x10;

	  telemetry_tx_buf[3] = (short)(BNO080_Roll*100);
	  telemetry_tx_buf[4] = ((short)(BNO080_Roll*100))>>8;

	  telemetry_tx_buf[5] = (short)(BNO080_Pitch*100);
	  telemetry_tx_buf[6] = ((short)(BNO080_Pitch*100))>>8;

	  telemetry_tx_buf[7] = (unsigned short)(BNO080_Yaw*100);
	  telemetry_tx_buf[8] = ((unsigned short)(BNO080_Yaw*100))>>8;

	  telemetry_tx_buf[9] = (short)(LPS22HH.baroAltFilt*10);
	  telemetry_tx_buf[10] = ((short)(LPS22HH.baroAltFilt*10))>>8;

	  telemetry_tx_buf[11] = (short)((iBus.RH-1500)*0.07f*100);
	  telemetry_tx_buf[12] = ((short)((iBus.RH-1500)*0.07f*100))>>8;

	  telemetry_tx_buf[13] = (short)((iBus.RV-1500)*0.07f*100);
	  telemetry_tx_buf[14] = ((short)((iBus.RV-1500)*0.07f*100))>>8;

	  telemetry_tx_buf[15] = (unsigned short)((iBus.LH-1000)*0.36f*100);
	  telemetry_tx_buf[16] = ((unsigned short)((iBus.LH-1000)*0.36f*100))>>8;

	  telemetry_tx_buf[17] = 0x00;
	  telemetry_tx_buf[18] = 0x00;

	  telemetry_tx_buf[19] = 0xff;

	  for(int i=0; i<19; i++)
	  {
		  telemetry_tx_buf[19] = telemetry_tx_buf[19] - telemetry_tx_buf[i];
	  }
}

void Encode_Msg_PID_Gain(unsigned char* telemetry_tx_buf, unsigned char id, float p, float i, float d)
{
	  telemetry_tx_buf[0] = 0x46;
	  telemetry_tx_buf[1] = 0x43;
	  telemetry_tx_buf[2] = id;

//	  memcpy(telemetry_tx_buf[3], &p, 4);
//	  memcpy(telemetry_tx_buf[7], &i, 4);
//	  memcpy(telemetry_tx_buf[11], &d, 4);

	  *(float*)&telemetry_tx_buf[3] = p;
	  *(float*)&telemetry_tx_buf[7] = i;
	  *(float*)&telemetry_tx_buf[11] = d;

	  telemetry_tx_buf[15] = 0x00;
	  telemetry_tx_buf[16] = 0x00;
	  telemetry_tx_buf[17] = 0x00;
	  telemetry_tx_buf[18] = 0x00;

//	  telemetry_tx_buf[19] = 0xff;
//
//	  for(int i=0; i<19; i++)
//	  {
//		  telemetry_tx_buf[19] = telemetry_tx_buf[19] - telemetry_tx_buf[i];
//	  }
}

void Encode_Msg_BatVolt(unsigned char* telemetry_tx_buf, unsigned char id, float batVolt)
{
	  telemetry_tx_buf[0] = 0x46;
	  telemetry_tx_buf[1] = 0x43;
	  telemetry_tx_buf[2] = id;

	  *(float*)&telemetry_tx_buf[3] = batVolt;

//	  telemetry_tx_buf[7] = 0xff;
//
//	  for(int i=0; i<7; i++)
//	  {
//		  telemetry_tx_buf[7] = telemetry_tx_buf[7] - telemetry_tx_buf[i];
//	  }
}

void Receive_Pid_Gain(void)
{
	if(telemetry_rx_cplt_flag == 1) //Receive GCS Message
	{
		telemetry_rx_cplt_flag = 0;

		if(iBus.SwA == 1000) //Check FS-i6 Switch A
		{

			LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH4);

			TIM3->PSC = 1000;
			HAL_Delay(10);

			LL_TIM_CC_DisableChannel(TIM3, LL_TIM_CHANNEL_CH4);

			switch(telemetry_rx_buf[2]) //Check ID of GCS Message
			{
			case 0:
				roll.in.kp = (*(int*)&telemetry_rx_buf[3]) / 100.f;
				roll.in.ki = (*(int*)&telemetry_rx_buf[7]) / 100.f;
				roll.in.kd = (*(int*)&telemetry_rx_buf[11]) / 100.f;
				EP_PIDGain_Write(telemetry_rx_buf[2], roll.in.kp, roll.in.ki, roll.in.kd);
				EP_PIDGain_Read(telemetry_rx_buf[2], &roll.in.kp, &roll.in.ki, &roll.in.kd);
				Encode_Msg_PID_Gain(&telemetry_tx_buf[0], telemetry_rx_buf[2], roll.in.kp, roll.in.ki, roll.in.kd);
				HAL_UART_Transmit_IT(&huart1, &telemetry_tx_buf[0], 19);
				break;
			case 1:
				roll.out.kp = *(int*)&telemetry_rx_buf[3] / 100.f;
				roll.out.ki = *(int*)&telemetry_rx_buf[7] / 100.f;
				roll.out.kd = *(int*)&telemetry_rx_buf[11] / 100.f;
				EP_PIDGain_Write(telemetry_rx_buf[2], roll.out.kp, roll.out.ki, roll.out.kd);
				EP_PIDGain_Read(telemetry_rx_buf[2], &roll.out.kp, &roll.out.ki, &roll.out.kd);
				Encode_Msg_PID_Gain(&telemetry_tx_buf[0], telemetry_rx_buf[2], roll.out.kp, roll.out.ki, roll.out.kd);
				HAL_UART_Transmit_IT(&huart1, &telemetry_tx_buf[0], 19);
				break;
			case 2:
				pitch.in.kp = *(int*)&telemetry_rx_buf[3] / 100.f;
				pitch.in.ki = *(int*)&telemetry_rx_buf[7] / 100.f;
				pitch.in.kd = *(int*)&telemetry_rx_buf[11] / 100.f;
				EP_PIDGain_Write(telemetry_rx_buf[2], pitch.in.kp, pitch.in.ki, pitch.in.kd);
				EP_PIDGain_Read(telemetry_rx_buf[2], &pitch.in.kp, &pitch.in.ki, &pitch.in.kd);
				Encode_Msg_PID_Gain(&telemetry_tx_buf[0], telemetry_rx_buf[2], pitch.in.kp, pitch.in.ki, pitch.in.kd);
				HAL_UART_Transmit_IT(&huart1, &telemetry_tx_buf[0], 19);
				break;
			case 3:
				pitch.out.kp = *(int*)&telemetry_rx_buf[3] / 100.f;
				pitch.out.ki = *(int*)&telemetry_rx_buf[7] / 100.f;
				pitch.out.kd = *(int*)&telemetry_rx_buf[11] / 100.f;
				EP_PIDGain_Write(telemetry_rx_buf[2], pitch.out.kp, pitch.out.ki, pitch.out.kd);
				EP_PIDGain_Read(telemetry_rx_buf[2], &pitch.out.kp, &pitch.out.ki, &pitch.out.kd);
				Encode_Msg_PID_Gain(&telemetry_tx_buf[0], telemetry_rx_buf[2], pitch.out.kp, pitch.out.ki, pitch.out.kd);
				HAL_UART_Transmit_IT(&huart1, &telemetry_tx_buf[0], 19);
				break;
			case 4:
				yaw_heading.kp = *(int*)&telemetry_rx_buf[3] / 100.f;
				yaw_heading.ki = *(int*)&telemetry_rx_buf[7] / 100.f;
				yaw_heading.kd = *(int*)&telemetry_rx_buf[11] / 100.f;
				EP_PIDGain_Write(telemetry_rx_buf[2], yaw_heading.kp, yaw_heading.ki, yaw_heading.kd);
				EP_PIDGain_Read(telemetry_rx_buf[2], &yaw_heading.kp, &yaw_heading.ki, &yaw_heading.kd);
				Encode_Msg_PID_Gain(&telemetry_tx_buf[0], telemetry_rx_buf[2], yaw_heading.kp, yaw_heading.ki, yaw_heading.kd);
				HAL_UART_Transmit_IT(&huart1, &telemetry_tx_buf[0], 19);
				break;
			case 5:
				yaw_rate.kp = *(int*)&telemetry_rx_buf[3] / 100.f;
				yaw_rate.ki = *(int*)&telemetry_rx_buf[7] / 100.f;
				yaw_rate.kd = *(int*)&telemetry_rx_buf[11] / 100.f;
				EP_PIDGain_Write(telemetry_rx_buf[2], yaw_rate.kp, yaw_rate.ki, yaw_rate.kd);
				EP_PIDGain_Read(telemetry_rx_buf[2], &yaw_rate.kp, &yaw_rate.ki, &yaw_rate.kd);
				Encode_Msg_PID_Gain(&telemetry_tx_buf[0], telemetry_rx_buf[2], yaw_rate.kp, yaw_rate.ki, yaw_rate.kd);
				HAL_UART_Transmit_IT(&huart1, &telemetry_tx_buf[0], 19);
				break;
			case 6:
				altitude.in.kp = *(int*)&telemetry_rx_buf[3] / 100.f;
				altitude.in.ki = *(int*)&telemetry_rx_buf[7] / 100.f;
				altitude.in.kd = *(int*)&telemetry_rx_buf[11] / 100.f;
				EP_PIDGain_Write(telemetry_rx_buf[2], altitude.in.kp, altitude.in.ki, altitude.in.kd);
				EP_PIDGain_Read(telemetry_rx_buf[2], &altitude.in.kp, &altitude.in.ki, &altitude.in.kd);
				Encode_Msg_PID_Gain(&telemetry_tx_buf[0], telemetry_rx_buf[2], altitude.in.kp, altitude.in.ki, altitude.in.kd);
				HAL_UART_Transmit_IT(&huart1, &telemetry_tx_buf[0], 19);
				break;
			case 7:
				altitude.out.kp = *(int*)&telemetry_rx_buf[3] / 100.f;
				altitude.out.ki = *(int*)&telemetry_rx_buf[7] / 100.f;
				altitude.out.kd = *(int*)&telemetry_rx_buf[11] / 100.f;
				EP_PIDGain_Write(telemetry_rx_buf[2], altitude.out.kp, altitude.out.ki, altitude.out.kd);
				EP_PIDGain_Read(telemetry_rx_buf[2], &altitude.out.kp, &altitude.out.ki, &altitude.out.kd);
				Encode_Msg_PID_Gain(&telemetry_tx_buf[0], telemetry_rx_buf[2], altitude.out.kp, altitude.out.ki, altitude.out.kd);
				HAL_UART_Transmit_IT(&huart1, &telemetry_tx_buf[0], 19);
				break;
			case 8:
				lat.in.kp = *(int*)&telemetry_rx_buf[3] / 100.f;
				lat.in.ki = *(int*)&telemetry_rx_buf[7] / 100.f;
				lat.in.kd = *(int*)&telemetry_rx_buf[11] / 100.f;
				EP_PIDGain_Write(telemetry_rx_buf[2], lat.in.kp, lat.in.ki, lat.in.kd);
				EP_PIDGain_Read(telemetry_rx_buf[2], &lat.in.kp, &lat.in.ki, &lat.in.kd);
				Encode_Msg_PID_Gain(&telemetry_tx_buf[0], telemetry_rx_buf[2], lat.in.kp, lat.in.ki, lat.in.kd);
				HAL_UART_Transmit_IT(&huart1, &telemetry_tx_buf[0], 19);
				break;
			case 9:
				lat.out.kp = *(int*)&telemetry_rx_buf[3] / 100.f;
				lat.out.ki = *(int*)&telemetry_rx_buf[7] / 100.f;
				lat.out.kd = *(int*)&telemetry_rx_buf[11] / 100.f;
				EP_PIDGain_Write(telemetry_rx_buf[2], lat.out.kp, lat.out.ki, lat.out.kd);
				EP_PIDGain_Read(telemetry_rx_buf[2], &lat.out.kp, &lat.out.ki, &lat.out.kd);
				Encode_Msg_PID_Gain(&telemetry_tx_buf[0], telemetry_rx_buf[2], lat.out.kp, lat.out.ki, lat.out.kd);
				HAL_UART_Transmit_IT(&huart1, &telemetry_tx_buf[0], 19);
				break;
			case 10:
				lon.in.kp = *(int*)&telemetry_rx_buf[3] / 100.f;
				lon.in.ki = *(int*)&telemetry_rx_buf[7] / 100.f;
				lon.in.kd = *(int*)&telemetry_rx_buf[11] / 100.f;
				EP_PIDGain_Write(telemetry_rx_buf[2], lon.in.kp, lon.in.ki, lon.in.kd);
				EP_PIDGain_Read(telemetry_rx_buf[2], &lon.in.kp, &lon.in.ki, &lon.in.kd);
				Encode_Msg_PID_Gain(&telemetry_tx_buf[0], telemetry_rx_buf[2], lon.in.kp, lon.in.ki, lon.in.kd);
				HAL_UART_Transmit_IT(&huart1, &telemetry_tx_buf[0], 19);
				break;
			case 11:
				lon.out.kp = *(int*)&telemetry_rx_buf[3] / 100.f;
				lon.out.ki = *(int*)&telemetry_rx_buf[7] / 100.f;
				lon.out.kd = *(int*)&telemetry_rx_buf[11] / 100.f;
				EP_PIDGain_Write(telemetry_rx_buf[2], lon.out.kp, lon.out.ki, lon.out.kd);
				EP_PIDGain_Read(telemetry_rx_buf[2], &lon.out.kp, &lon.out.ki, &lon.out.kd);
				Encode_Msg_PID_Gain(&telemetry_tx_buf[0], telemetry_rx_buf[2], lon.out.kp, lon.out.ki, lon.out.kd);
				HAL_UART_Transmit_IT(&huart1, &telemetry_tx_buf[0], 19);
				break;
			case 12:
				batVolt = *(int*)&telemetry_rx_buf[3] / 100.f;
				EP_PIDGain_Write(telemetry_rx_buf[2], batVolt, 0.f, 0.f);
				EP_PIDGain_Read(telemetry_rx_buf[2], &batVolt, &dummy , &dummy);
				Encode_Msg_BatVolt(&telemetry_tx_buf[0], telemetry_rx_buf[2], batVolt);
				HAL_UART_Transmit_IT(&huart1, &telemetry_tx_buf[0], 7);
				break;
			case 13:
				Encode_Msg_PID_Gain(&telemetry_tx_buf[0], 0, roll.in.kp, roll.in.ki, roll.in.kd);
				HAL_UART_Transmit(&huart1, &telemetry_tx_buf[0], 19, 10);
				Encode_Msg_PID_Gain(&telemetry_tx_buf[0], 1, roll.out.kp, roll.out.ki, roll.out.kd);
				HAL_UART_Transmit(&huart1, &telemetry_tx_buf[0], 19, 10);
				Encode_Msg_PID_Gain(&telemetry_tx_buf[0], 2, pitch.in.kp, pitch.in.ki, pitch.in.kd);
				HAL_UART_Transmit(&huart1, &telemetry_tx_buf[0], 19, 10);
				Encode_Msg_PID_Gain(&telemetry_tx_buf[0], 3, pitch.out.kp, pitch.out.ki, pitch.out.kd);
				HAL_UART_Transmit(&huart1, &telemetry_tx_buf[0], 19, 10);
				Encode_Msg_PID_Gain(&telemetry_tx_buf[0], 4, yaw_heading.kp, yaw_heading.ki, yaw_heading.kd);
				HAL_UART_Transmit(&huart1, &telemetry_tx_buf[0], 19, 10);
				Encode_Msg_PID_Gain(&telemetry_tx_buf[0], 5, yaw_rate.kp, yaw_rate.ki, yaw_rate.kd);
				HAL_UART_Transmit(&huart1, &telemetry_tx_buf[0], 19, 10);
				Encode_Msg_PID_Gain(&telemetry_tx_buf[0], 6, altitude.in.kp, altitude.in.ki, altitude.in.kd);
				HAL_UART_Transmit(&huart1, &telemetry_tx_buf[0], 19, 10);
				Encode_Msg_PID_Gain(&telemetry_tx_buf[0], 7, altitude.out.kp, altitude.out.ki, altitude.out.kd);
				HAL_UART_Transmit(&huart1, &telemetry_tx_buf[0], 19, 10);
				Encode_Msg_PID_Gain(&telemetry_tx_buf[0], 8, lat.in.kp, lat.in.ki, lat.in.kd);
				HAL_UART_Transmit(&huart1, &telemetry_tx_buf[0], 19, 10);
				Encode_Msg_PID_Gain(&telemetry_tx_buf[0], 9, lat.out.kp, lat.out.ki, lat.out.kd);
				HAL_UART_Transmit(&huart1, &telemetry_tx_buf[0], 19, 10);
				Encode_Msg_PID_Gain(&telemetry_tx_buf[0], 10, lon.in.kp, lon.in.ki, lon.in.kd);
				HAL_UART_Transmit(&huart1, &telemetry_tx_buf[0], 19, 10);
				Encode_Msg_PID_Gain(&telemetry_tx_buf[0], 11, lon.out.kp, lon.out.ki, lon.out.kd);
				HAL_UART_Transmit(&huart1, &telemetry_tx_buf[0], 19, 10);
				break;
			}
		}
	}
}


void Encode_Msg_Motor(unsigned char* telemetry_tx_buf)
{
	telemetry_tx_buf[0] = 0x88;
	telemetry_tx_buf[1] = 0x18;

	telemetry_tx_buf[2] = ccr1 >> 24;
	telemetry_tx_buf[3] = ccr1 >> 16;
	telemetry_tx_buf[4] = ccr1 >> 8;
	telemetry_tx_buf[5] = ccr1;

	telemetry_tx_buf[6] = ccr2 >> 24;
	telemetry_tx_buf[7] = ccr2 >> 16;
	telemetry_tx_buf[8] = ccr2 >> 8;
	telemetry_tx_buf[9] = ccr2;

	telemetry_tx_buf[10] = ccr3 >> 24;
	telemetry_tx_buf[11] = ccr3 >> 16;
	telemetry_tx_buf[12] = ccr3 >> 8;
	telemetry_tx_buf[13] = ccr3;

	telemetry_tx_buf[14] = ccr4 >> 24;
	telemetry_tx_buf[15] = ccr4 >> 16;
	telemetry_tx_buf[16] = ccr4 >> 8;
	telemetry_tx_buf[17] = ccr4;

	telemetry_tx_buf[18] = iBus.LV >> 8;
	telemetry_tx_buf[19] = iBus.LV;

	telemetry_tx_buf[20] = ((int)pitch.in.pid_result) >> 24;
	telemetry_tx_buf[21] = ((int)pitch.in.pid_result) >> 16;
	telemetry_tx_buf[22] = ((int)pitch.in.pid_result) >> 8;
	telemetry_tx_buf[23] = ((int)pitch.in.pid_result);

	telemetry_tx_buf[24] = ((int)roll.in.pid_result) >> 24;
	telemetry_tx_buf[25] = ((int)roll.in.pid_result) >> 16;
	telemetry_tx_buf[26] = ((int)roll.in.pid_result) >> 8;
	telemetry_tx_buf[27] = ((int)roll.in.pid_result);

	telemetry_tx_buf[28] = ((int)yaw_heading.pid_result) >> 24;
	telemetry_tx_buf[29] = ((int)yaw_heading.pid_result) >> 16;
	telemetry_tx_buf[30] = ((int)yaw_heading.pid_result) >> 8;
	telemetry_tx_buf[31] = ((int)yaw_heading.pid_result);

//     telemetry_tx_buf[32] = ((int)altitude_filt) >> 24;
//     telemetry_tx_buf[33] = ((int)altitude_filt) >> 16;
//     telemetry_tx_buf[34] = ((int)altitude_filt) >> 8;
//     telemetry_tx_buf[35] = ((int)altitude_filt);

     telemetry_tx_buf[32] = ((int)actual_pressure_fast) >> 24;
     telemetry_tx_buf[33] = ((int)actual_pressure_fast) >> 16;
     telemetry_tx_buf[34] = ((int)actual_pressure_fast) >> 8;
     telemetry_tx_buf[35] = ((int)actual_pressure_fast);
}

void Encode_Msg_Altitude(unsigned char* telemetry_tx_buf)
{
	telemetry_tx_buf[0] = 0x88;
	telemetry_tx_buf[1] = 0x18;

	telemetry_tx_buf[2] = (int)(actual_pressure_fast * 100) >> 24;
	telemetry_tx_buf[3] = (int)(actual_pressure_fast * 100) >> 16;
	telemetry_tx_buf[4] = (int)(actual_pressure_fast * 100) >> 8;
	telemetry_tx_buf[5] = (int)(actual_pressure_fast * 100);

	telemetry_tx_buf[6] = (int)(altitude_setpoint * 100) >> 24;
	telemetry_tx_buf[7] = (int)(altitude_setpoint * 100) >> 16;
	telemetry_tx_buf[8] = (int)(altitude_setpoint * 100) >> 8;
	telemetry_tx_buf[9] = (int)(altitude_setpoint * 100);

	telemetry_tx_buf[10] = ((int)(altitude.out.error * 100.f)) >> 24;
	telemetry_tx_buf[11] = ((int)(altitude.out.error * 100.f)) >> 16;
	telemetry_tx_buf[12] = ((int)(altitude.out.error * 100.f)) >> 8;
	telemetry_tx_buf[13] = ((int)(altitude.out.error * 100.f));

	telemetry_tx_buf[14] = ((int)(altitude.in.pid_result)) >> 24;
	telemetry_tx_buf[15] = ((int)(altitude.in.pid_result)) >> 16;
	telemetry_tx_buf[16] = ((int)(altitude.in.pid_result)) >> 8;
	telemetry_tx_buf[17] = ((int)(altitude.in.pid_result));

	telemetry_tx_buf[18] = ((int)(iBus.LV)) >> 24;
	telemetry_tx_buf[19] = ((int)(iBus.LV)) >> 16;
	telemetry_tx_buf[20] = ((int)(iBus.LV)) >> 8;
	telemetry_tx_buf[21] = ((int)(iBus.LV));

	telemetry_tx_buf[22] = ((int)(batVolt * 100)) >> 24;
	telemetry_tx_buf[23] = ((int)(batVolt * 100)) >> 16;
	telemetry_tx_buf[24] = ((int)(batVolt * 100)) >> 8;
	telemetry_tx_buf[25] = ((int)(batVolt * 100));
}

void Encode_Msg_Gps(unsigned char* telemery_tx_buf)
{
	telemetry_tx_buf[0] = 0x77;
	telemetry_tx_buf[1] = 0x17;

	telemetry_tx_buf[2] = ((int)(batVolt * 100)) >> 24;
	telemetry_tx_buf[3] = ((int)(batVolt * 100)) >> 16;
	telemetry_tx_buf[4] = ((int)(batVolt * 100)) >> 8;
	telemetry_tx_buf[5] = ((int)(batVolt * 100));

	telemetry_tx_buf[6] = pvt.fixType;

	telemetry_tx_buf[7] = (int)BNO080_Yaw >> 24;
	telemetry_tx_buf[8] = (int)BNO080_Yaw >> 16;
	telemetry_tx_buf[9] = (int)BNO080_Yaw >> 8;
	telemetry_tx_buf[10] = (int)BNO080_Yaw;

	telemetry_tx_buf[11] = (long long int)lat_gps >> 56;
	telemetry_tx_buf[12] = (long long int)lat_gps >> 48;
	telemetry_tx_buf[13] = (long long int)lat_gps >> 40;
	telemetry_tx_buf[14] = (long long int)lat_gps >> 32;
	telemetry_tx_buf[15] = (long long int)lat_gps >> 24;
	telemetry_tx_buf[16] = (long long int)lat_gps >> 16;
	telemetry_tx_buf[17] = (long long int)lat_gps >> 8;
	telemetry_tx_buf[18] = (long long int)lat_gps;

	telemetry_tx_buf[19] = (long long int)lon_gps >> 56;
	telemetry_tx_buf[20] = (long long int)lon_gps >> 48;
	telemetry_tx_buf[21] = (long long int)lon_gps >> 40;
	telemetry_tx_buf[22] = (long long int)lon_gps >> 32;
	telemetry_tx_buf[23] = (long long int)lon_gps >> 24;
	telemetry_tx_buf[24] = (long long int)lon_gps >> 16;
	telemetry_tx_buf[25] = (long long int)lon_gps >> 8;
	telemetry_tx_buf[26] = (long long int)lon_gps;

	telemetry_tx_buf[27] = (long long int)lat_waypoint >> 56;
	telemetry_tx_buf[28] = (long long int)lat_waypoint >> 48;
	telemetry_tx_buf[29] = (long long int)lat_waypoint >> 40;
	telemetry_tx_buf[30] = (long long int)lat_waypoint >> 32;
	telemetry_tx_buf[31] = (long long int)lat_waypoint >> 24;
	telemetry_tx_buf[32] = (long long int)lat_waypoint >> 16;
	telemetry_tx_buf[33] = (long long int)lat_waypoint >> 8;
	telemetry_tx_buf[34] = (long long int)lat_waypoint;

	telemetry_tx_buf[35] = (long long int)lon_waypoint >> 56;
	telemetry_tx_buf[36] = (long long int)lon_waypoint >> 48;
	telemetry_tx_buf[37] = (long long int)lon_waypoint >> 40;
	telemetry_tx_buf[38] = (long long int)lon_waypoint >> 32;
	telemetry_tx_buf[39] = (long long int)lon_waypoint >> 24;
	telemetry_tx_buf[40] = (long long int)lon_waypoint >> 16;
	telemetry_tx_buf[41] = (long long int)lon_waypoint >> 8;
	telemetry_tx_buf[42] = (long long int)lon_waypoint;

	telemetry_tx_buf[43] = (int)gps_pitch_adjust >> 24;
	telemetry_tx_buf[44] = (int)gps_pitch_adjust >> 16;
	telemetry_tx_buf[45] = (int)gps_pitch_adjust >> 8;
	telemetry_tx_buf[46] = (int)gps_pitch_adjust;

	telemetry_tx_buf[47] = (int)gps_roll_adjust >> 24;
	telemetry_tx_buf[48] = (int)gps_roll_adjust >> 16;
	telemetry_tx_buf[49] = (int)gps_roll_adjust >> 8;
	telemetry_tx_buf[50] = (int)gps_roll_adjust;

	chksum_pid = 0xffffffff;

	for(int i=0; i<51; i++)
	{

		chksum_pid = chksum_pid - telemetry_tx_buf[i];
	}

	telemetry_tx_buf[51] = chksum_pid >> 24;
	telemetry_tx_buf[52] = chksum_pid >> 16;
	telemetry_tx_buf[53] = chksum_pid >> 8;
	telemetry_tx_buf[54] = chksum_pid;
}

void Encode_Msg_PID(unsigned char* telemery_tx_buf)
{
	telemetry_tx_buf[0] = 0x11;
	telemetry_tx_buf[1] = 0x03;

	telemetry_tx_buf[2] = (int)lat.out.reference >> 24;
	telemetry_tx_buf[3] = (int)lat.out.reference >> 16;
	telemetry_tx_buf[4] = (int)lat.out.reference >> 8;
	telemetry_tx_buf[5] = (int)lat.out.reference;

	telemetry_tx_buf[6] = (int)lat.out.meas_value >> 24;
	telemetry_tx_buf[7] = (int)lat.out.meas_value >> 16;
	telemetry_tx_buf[8] = (int)lat.out.meas_value >> 8;
	telemetry_tx_buf[9] = (int)lat.out.meas_value;

	telemetry_tx_buf[10] = (int)lat.out.error >> 24;
	telemetry_tx_buf[11] = (int)lat.out.error >> 16;
	telemetry_tx_buf[12] = (int)lat.out.error >> 8;
	telemetry_tx_buf[13] = (int)lat.out.error;

	telemetry_tx_buf[14] = (int)lat.out.error_deriv >> 24;
	telemetry_tx_buf[15] = (int)lat.out.error_deriv >> 16;
	telemetry_tx_buf[16] = (int)lat.out.error_deriv >> 8;
	telemetry_tx_buf[17] = (int)lat.out.error_deriv;

	telemetry_tx_buf[18] = (int)lat.out.error_sum >> 24;
	telemetry_tx_buf[19] = (int)lat.out.error_sum >> 16;
	telemetry_tx_buf[20] = (int)lat.out.error_sum >> 8;
	telemetry_tx_buf[21] = (int)lat.out.error_sum;

	telemetry_tx_buf[22] = (int)lat.out.p_result >> 24;
	telemetry_tx_buf[23] = (int)lat.out.p_result >> 16;
	telemetry_tx_buf[24] = (int)lat.out.p_result >> 8;
	telemetry_tx_buf[25] = (int)lat.out.p_result;

	telemetry_tx_buf[26] = (int)lat.out.i_result >> 24;
	telemetry_tx_buf[27] = (int)lat.out.i_result >> 16;
	telemetry_tx_buf[28] = (int)lat.out.i_result >> 8;
	telemetry_tx_buf[29] = (int)lat.out.i_result;

	telemetry_tx_buf[30] = (int)lat.out.d_result >> 24;
	telemetry_tx_buf[31] = (int)lat.out.d_result >> 16;
	telemetry_tx_buf[32] = (int)lat.out.d_result >> 8;
	telemetry_tx_buf[33] = (int)lat.out.d_result;

	telemetry_tx_buf[34] = (int)lat.out.pid_result >> 24;
	telemetry_tx_buf[35] = (int)lat.out.pid_result >> 16;
	telemetry_tx_buf[36] = (int)lat.out.pid_result >> 8;
	telemetry_tx_buf[37] = (int)lat.out.pid_result;

	telemetry_tx_buf[38] = (int)lat.in.reference >> 24;
	telemetry_tx_buf[39] = (int)lat.in.reference >> 16;
	telemetry_tx_buf[40] = (int)lat.in.reference >> 8;
	telemetry_tx_buf[41] = (int)lat.in.reference;

	telemetry_tx_buf[42] = (int)lat.in.meas_value >> 24;
	telemetry_tx_buf[43] = (int)lat.in.meas_value >> 16;
	telemetry_tx_buf[44] = (int)lat.in.meas_value >> 8;
	telemetry_tx_buf[45] = (int)lat.in.meas_value;

	telemetry_tx_buf[46] = (int)lat.in.error >> 24;
	telemetry_tx_buf[47] = (int)lat.in.error >> 16;
	telemetry_tx_buf[48] = (int)lat.in.error >> 8;
	telemetry_tx_buf[49] = (int)lat.in.error;

	telemetry_tx_buf[50] = (int)lat.in.error_deriv >> 24;
	telemetry_tx_buf[51] = (int)lat.in.error_deriv >> 16;
	telemetry_tx_buf[52] = (int)lat.in.error_deriv >> 8;
	telemetry_tx_buf[53] = (int)lat.in.error_deriv;

	telemetry_tx_buf[54] = (int)lat.in.error_sum >> 24;
	telemetry_tx_buf[55] = (int)lat.in.error_sum >> 16;
	telemetry_tx_buf[56] = (int)lat.in.error_sum >> 8;
	telemetry_tx_buf[57] = (int)lat.in.error_sum;

	telemetry_tx_buf[58] = (int)lat.in.p_result >> 24;
	telemetry_tx_buf[59] = (int)lat.in.p_result >> 16;
	telemetry_tx_buf[60] = (int)lat.in.p_result >> 8;
	telemetry_tx_buf[61] = (int)lat.in.p_result;

	telemetry_tx_buf[62] = (int)lat.in.i_result >> 24;
	telemetry_tx_buf[63] = (int)lat.in.i_result >> 16;
	telemetry_tx_buf[64] = (int)lat.in.i_result >> 8;
	telemetry_tx_buf[65] = (int)lat.in.i_result;

	telemetry_tx_buf[66] = (int)lat.in.d_result >> 24;
	telemetry_tx_buf[67] = (int)lat.in.d_result >> 16;
	telemetry_tx_buf[68] = (int)lat.in.d_result >> 8;
	telemetry_tx_buf[69] = (int)lat.in.d_result;

	telemetry_tx_buf[70] = (int)lat.in.pid_result >> 24;
	telemetry_tx_buf[71] = (int)lat.in.pid_result >> 16;
	telemetry_tx_buf[72] = (int)lat.in.pid_result >> 8;
	telemetry_tx_buf[73] = (int)lat.in.pid_result;

	telemetry_tx_buf[74] = (long long int)lat_gps >> 56;
	telemetry_tx_buf[75] = (long long int)lat_gps >> 48;
	telemetry_tx_buf[76] = (long long int)lat_gps >> 40;
	telemetry_tx_buf[77] = (long long int)lat_gps >> 32;
	telemetry_tx_buf[78] = (long long int)lat_gps >> 24;
	telemetry_tx_buf[79] = (long long int)lat_gps >> 16;
	telemetry_tx_buf[80] = (long long int)lat_gps >> 8;
	telemetry_tx_buf[81] = (long long int)lat_gps;

	telemetry_tx_buf[82] = (long long int)pvt.lat >> 56;
	telemetry_tx_buf[83] = (long long int)pvt.lat >> 48;
	telemetry_tx_buf[84] = (long long int)pvt.lat >> 40;
	telemetry_tx_buf[85] = (long long int)pvt.lat >> 32;
	telemetry_tx_buf[86] = (long long int)pvt.lat >> 24;
	telemetry_tx_buf[87] = (long long int)pvt.lat >> 16;
	telemetry_tx_buf[88] = (long long int)pvt.lat >> 8;
	telemetry_tx_buf[89] = (long long int)pvt.lat;

	telemetry_tx_buf[90] = (int)lon.out.reference >> 24;
	telemetry_tx_buf[91] = (int)lon.out.reference >> 16;
	telemetry_tx_buf[92] = (int)lon.out.reference >> 8;
	telemetry_tx_buf[93] = (int)lon.out.reference;

	telemetry_tx_buf[94] = (int)lon.out.meas_value >> 24;
	telemetry_tx_buf[95] = (int)lon.out.meas_value >> 16;
	telemetry_tx_buf[96] = (int)lon.out.meas_value >> 8;
	telemetry_tx_buf[97] = (int)lon.out.meas_value;

	telemetry_tx_buf[98] = (int)lon.out.error >> 24;
	telemetry_tx_buf[99] = (int)lon.out.error >> 16;
	telemetry_tx_buf[100] = (int)lon.out.error >> 8;
	telemetry_tx_buf[101] = (int)lon.out.error;

	telemetry_tx_buf[102] = (int)lon.out.error_deriv >> 24;
	telemetry_tx_buf[103] = (int)lon.out.error_deriv >> 16;
	telemetry_tx_buf[104] = (int)lon.out.error_deriv >> 8;
	telemetry_tx_buf[105] = (int)lon.out.error_deriv;

	telemetry_tx_buf[106] = (int)lon.out.error_sum >> 24;
	telemetry_tx_buf[107] = (int)lon.out.error_sum >> 16;
	telemetry_tx_buf[108] = (int)lon.out.error_sum >> 8;
	telemetry_tx_buf[109] = (int)lon.out.error_sum;

	telemetry_tx_buf[110] = (int)lon.out.p_result >> 24;
	telemetry_tx_buf[111] = (int)lon.out.p_result >> 16;
	telemetry_tx_buf[112] = (int)lon.out.p_result >> 8;
	telemetry_tx_buf[113] = (int)lon.out.p_result;

	telemetry_tx_buf[114] = (int)lon.out.i_result >> 24;
	telemetry_tx_buf[115] = (int)lon.out.i_result >> 16;
	telemetry_tx_buf[116] = (int)lon.out.i_result >> 8;
	telemetry_tx_buf[117] = (int)lon.out.i_result;

	telemetry_tx_buf[118] = (int)lon.out.d_result >> 24;
	telemetry_tx_buf[119] = (int)lon.out.d_result >> 16;
	telemetry_tx_buf[120] = (int)lon.out.d_result >> 8;
	telemetry_tx_buf[121] = (int)lon.out.d_result;

	telemetry_tx_buf[122] = (int)lon.out.pid_result >> 24;
	telemetry_tx_buf[123] = (int)lon.out.pid_result >> 16;
	telemetry_tx_buf[124] = (int)lon.out.pid_result >> 8;
	telemetry_tx_buf[125] = (int)lon.out.pid_result;

	telemetry_tx_buf[126] = (int)lon.in.reference >> 24;
	telemetry_tx_buf[127] = (int)lon.in.reference >> 16;
	telemetry_tx_buf[128] = (int)lon.in.reference >> 8;
	telemetry_tx_buf[129] = (int)lon.in.reference;

	telemetry_tx_buf[130] = (int)lon.in.meas_value >> 24;
	telemetry_tx_buf[131] = (int)lon.in.meas_value >> 16;
	telemetry_tx_buf[132] = (int)lon.in.meas_value >> 8;
	telemetry_tx_buf[133] = (int)lon.in.meas_value;

	telemetry_tx_buf[134] = (int)lon.in.error >> 24;
	telemetry_tx_buf[135] = (int)lon.in.error >> 16;
	telemetry_tx_buf[136] = (int)lon.in.error >> 8;
	telemetry_tx_buf[137] = (int)lon.in.error;

	telemetry_tx_buf[138] = (int)lon.in.error_deriv >> 24;
	telemetry_tx_buf[139] = (int)lon.in.error_deriv >> 16;
	telemetry_tx_buf[140] = (int)lon.in.error_deriv >> 8;
	telemetry_tx_buf[141] = (int)lon.in.error_deriv;

	telemetry_tx_buf[142] = (int)lon.in.error_sum >> 24;
	telemetry_tx_buf[143] = (int)lon.in.error_sum >> 16;
	telemetry_tx_buf[144] = (int)lon.in.error_sum >> 8;
	telemetry_tx_buf[145] = (int)lon.in.error_sum;

	telemetry_tx_buf[146] = (int)lon.in.p_result >> 24;
	telemetry_tx_buf[147] = (int)lon.in.p_result >> 16;
	telemetry_tx_buf[148] = (int)lon.in.p_result >> 8;
	telemetry_tx_buf[149] = (int)lon.in.p_result;

	telemetry_tx_buf[150] = (int)lon.in.i_result >> 24;
	telemetry_tx_buf[151] = (int)lon.in.i_result >> 16;
	telemetry_tx_buf[152] = (int)lon.in.i_result >> 8;
	telemetry_tx_buf[153] = (int)lon.in.i_result;

	telemetry_tx_buf[154] = (int)lon.in.d_result >> 24;
	telemetry_tx_buf[155] = (int)lon.in.d_result >> 16;
	telemetry_tx_buf[156] = (int)lon.in.d_result >> 8;
	telemetry_tx_buf[157] = (int)lon.in.d_result;

	telemetry_tx_buf[158] = (int)lon.in.pid_result >> 24;
	telemetry_tx_buf[159] = (int)lon.in.pid_result >> 16;
	telemetry_tx_buf[160] = (int)lon.in.pid_result >> 8;
	telemetry_tx_buf[161] = (int)lon.in.pid_result;

	telemetry_tx_buf[162] = (long long int)lon_gps >> 56;
	telemetry_tx_buf[163] = (long long int)lon_gps >> 48;
	telemetry_tx_buf[164] = (long long int)lon_gps >> 40;
	telemetry_tx_buf[165] = (long long int)lon_gps >> 32;
	telemetry_tx_buf[166] = (long long int)lon_gps >> 24;
	telemetry_tx_buf[167] = (long long int)lon_gps >> 16;
	telemetry_tx_buf[168] = (long long int)lon_gps >> 8;
	telemetry_tx_buf[169] = (long long int)lon_gps;

	telemetry_tx_buf[170] = (long long int)pvt.lon >> 56;
	telemetry_tx_buf[171] = (long long int)pvt.lon >> 48;
	telemetry_tx_buf[172] = (long long int)pvt.lon >> 40;
	telemetry_tx_buf[173] = (long long int)pvt.lon >> 32;
	telemetry_tx_buf[174] = (long long int)pvt.lon >> 24;
	telemetry_tx_buf[175] = (long long int)pvt.lon >> 16;
	telemetry_tx_buf[176] = (long long int)pvt.lon >> 8;
	telemetry_tx_buf[177] = (long long int)pvt.lon;

	telemetry_tx_buf[178] = (int)gps_roll_adjust >> 24;
	telemetry_tx_buf[179] = (int)gps_roll_adjust >> 16;
	telemetry_tx_buf[180] = (int)gps_roll_adjust >> 8;
	telemetry_tx_buf[181] = (int)gps_roll_adjust;

	telemetry_tx_buf[182] = (int)gps_pitch_adjust >> 24;
	telemetry_tx_buf[183] = (int)gps_pitch_adjust >> 16;
	telemetry_tx_buf[184] = (int)gps_pitch_adjust >> 8;
	telemetry_tx_buf[185] = (int)gps_pitch_adjust;

	telemetry_tx_buf[186] = ccr1 >> 24;
	telemetry_tx_buf[187] = ccr1 >> 16;
	telemetry_tx_buf[188] = ccr1 >> 8;
	telemetry_tx_buf[189] = ccr1;

	telemetry_tx_buf[190] = ccr2 >> 24;
	telemetry_tx_buf[191] = ccr2 >> 16;
	telemetry_tx_buf[192] = ccr2 >> 8;
	telemetry_tx_buf[193] = ccr2;

	telemetry_tx_buf[194] = ccr3 >> 24;
	telemetry_tx_buf[195] = ccr3 >> 16;
	telemetry_tx_buf[196] = ccr3 >> 8;
	telemetry_tx_buf[197] = ccr3;

	telemetry_tx_buf[198] = ccr4 >> 24;
	telemetry_tx_buf[199] = ccr4 >> 16;
	telemetry_tx_buf[200] = ccr4 >> 8;
	telemetry_tx_buf[201] = ccr4;

	telemetry_tx_buf[202] = (int)BNO080_Yaw >> 24;
	telemetry_tx_buf[203] = (int)BNO080_Yaw >> 16;
	telemetry_tx_buf[204] = (int)BNO080_Yaw >> 8;
	telemetry_tx_buf[205] = (int)BNO080_Yaw;

	chksum_pid = 0xffffffff;

	for(int i=0; i<186; i++)
	{

		chksum_pid = chksum_pid - telemetry_tx_buf[i];
	}

	telemetry_tx_buf[206] = chksum_pid >> 24;
	telemetry_tx_buf[207] = chksum_pid >> 16;
	telemetry_tx_buf[208] = chksum_pid >> 8;
	telemetry_tx_buf[209] = chksum_pid;
}

void Encode_Msg_Temp(unsigned char* telemery_tx_buf)
{
	telemetry_tx_buf[0] = 0x88;
	telemetry_tx_buf[1] = 0x18;

	telemetry_tx_buf[2] = ((int)(actual_pressure_fast * 100)) >> 24;
	telemetry_tx_buf[3] = ((int)(actual_pressure_fast * 100)) >> 16;
	telemetry_tx_buf[4] = ((int)(actual_pressure_fast * 100)) >> 8;
	telemetry_tx_buf[5] = ((int)(actual_pressure_fast * 100));

	telemetry_tx_buf[6] = ((int)(altitude_setpoint * 100)) >> 24;
	telemetry_tx_buf[7] = ((int)(altitude_setpoint * 100)) >> 16;
	telemetry_tx_buf[8] = ((int)(altitude_setpoint * 100)) >> 8;
	telemetry_tx_buf[9] = ((int)(altitude_setpoint * 100));

	telemetry_tx_buf[10] = ((int)(altitude.out.error * 100)) >> 24;
	telemetry_tx_buf[11] = ((int)(altitude.out.error * 100)) >> 16;
	telemetry_tx_buf[12] = ((int)(altitude.out.error * 100)) >> 8;
	telemetry_tx_buf[13] = ((int)(altitude.out.error * 100));

	telemetry_tx_buf[14] = (iBus.LV) >> 8;
	telemetry_tx_buf[15] = (iBus.LV);

	telemetry_tx_buf[16] = ((int)(altitude.in.pid_result)) >> 24;
	telemetry_tx_buf[17] = ((int)(altitude.in.pid_result)) >> 16;
	telemetry_tx_buf[18] = ((int)(altitude.in.pid_result)) >> 8;
	telemetry_tx_buf[19] = ((int)(altitude.in.pid_result));

	telemetry_tx_buf[20] = ((int)(batVolt * 1000)) >> 24;
	telemetry_tx_buf[21] = ((int)(batVolt * 1000)) >> 16;
	telemetry_tx_buf[22] = ((int)(batVolt * 1000)) >> 8;
	telemetry_tx_buf[23] = ((int)(batVolt * 1000));

	telemetry_tx_buf[24] = ((int)(LPS22HH.temperature_raw/100.f)) >> 24;
	telemetry_tx_buf[25] = ((int)(LPS22HH.temperature_raw/100.f)) >> 16;
	telemetry_tx_buf[26] = ((int)(LPS22HH.temperature_raw/100.f)) >> 8;
	telemetry_tx_buf[27] = ((int)(LPS22HH.temperature_raw/100.f));
}

void return_to_home(void) {

	if (flight_mode == 4) {
		/////////////////////////////////////////////////////////////////////////////////////////////////////////
		//Step 0 - make some basic calculations
		/////////////////////////////////////////////////////////////////////////////////////////////////////////
		if (return_to_home_step == 0) {
			//Is the quadcopter nearby? Then land without returning to home.
			if( lat_waypoint - lat_gps_home < 100 && lat_waypoint - lat_gps_home > -100 ) is_lat_nearby = 1;
			if( lon_waypoint - lon_gps_home < 100 && lon_waypoint - lon_gps_home > -100 ) is_lon_nearby = 1;

			if (is_lat_nearby == 1 && is_lon_nearby == 1)return_to_home_step = 3;
			else {
				return_to_home_move_factor = 0.0;
				if (return_to_home_lat_factor == 1 || return_to_home_lon_factor == 1)return_to_home_step = 1;
				//cos(((float)l_lat_gps / 1000000.0)
				if (abs_int(lat_gps_home, lat_waypoint) >= abs_int(lon_gps_home, lon_waypoint)) {
					return_to_home_lon_factor = (float)abs_int(lon_gps_home, lon_waypoint) / (float)abs_int(lat_gps_home, lat_waypoint);
					return_to_home_lat_factor = 1;
				}
				else {
					return_to_home_lon_factor = 1;
					return_to_home_lat_factor = (float)abs_int(lat_gps_home, lat_waypoint) / (float)abs_int(lon_gps_home, lon_waypoint);
				}

				if (actual_pressure_fast < 20)return_to_home_decrease = 20 - actual_pressure_fast;
				else return_to_home_decrease = 0;
			}
		}
		/////////////////////////////////////////////////////////////////////////////////////////////////////////
		//Step - 1 increase the altitude to 20 meter above ground level
		/////////////////////////////////////////////////////////////////////////////////////////////////////////
		if (return_to_home_step == 1) {
			if (return_to_home_decrease <= 0)return_to_home_step = 2;
			if (return_to_home_decrease > 0) {
				altitude_setpoint += 0.05;
				return_to_home_decrease -= 0.05;
			}
		}
		/////////////////////////////////////////////////////////////////////////////////////////////////////////
		//Step 2 - Return to the home position
		/////////////////////////////////////////////////////////////////////////////////////////////////////////
		if (return_to_home_step == 2) {
			if (lat_gps_home == lat_waypoint && lon_gps_home == lon_waypoint)return_to_home_step = 3;
			if (abs_int(lat_gps_home, lat_waypoint) < 160 && abs_int(lon_gps_home, lon_waypoint) < 160 && return_to_home_move_factor > 0.05)return_to_home_move_factor -= 0.00015;
			else if (return_to_home_move_factor < 0.20)return_to_home_move_factor += 0.0001;

			if (lat_gps_home != lat_waypoint) {
				if (lat_gps_home > lat_waypoint) l_lat_gps_float_adjust += return_to_home_move_factor * return_to_home_lat_factor;
				if (lat_gps_home < lat_waypoint) l_lat_gps_float_adjust -= return_to_home_move_factor * return_to_home_lat_factor;
			}
			if (lon_gps_home != lon_waypoint) {
				if (lon_gps_home > lon_waypoint) l_lon_gps_float_adjust += return_to_home_move_factor * return_to_home_lon_factor;
				if (lon_gps_home < lon_waypoint) l_lon_gps_float_adjust -= return_to_home_move_factor * return_to_home_lon_factor;
			}
		}
		/////////////////////////////////////////////////////////////////////////////////////////////////////////
		//Step - 3 decrease the altitude by increasing the pressure setpoint
		/////////////////////////////////////////////////////////////////////////////////////////////////////////
		if (return_to_home_step == 3) {
			if (altitude_setpoint < 1 )return_to_home_step = 4;
			altitude_setpoint -= 0.01;
		}
		/////////////////////////////////////////////////////////////////////////////////////////////////////////
		//Step - 4 Stop the motors
		/////////////////////////////////////////////////////////////////////////////////////////////////////////
		if (return_to_home_step == 4) {
			motor_arming_flag = 0;
			return_to_home_step = 5;
		}

	}
}

void Calculate_Takeoff_Throttle()
{
	takeoff_throttle = (2 - cos(BNO080_Roll * 0.017453) * cos(BNO080_Pitch * 0.017453)) * 84 * ( batVolt * (-32.632) + 2112.4 - 1000);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
