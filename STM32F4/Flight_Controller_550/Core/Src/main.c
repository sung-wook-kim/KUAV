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
uint8_t telemetry_tx_buf[40];
uint8_t telemetry_rx_buf[20];
uint8_t telemetry_rx_cplt_flag;

extern uint8_t nx_rx_cplt_flag;
extern uint8_t nx_rx_buf;

// Timer variables
extern uint8_t tim7_1ms_flag;
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
signed int lat_prev = 0;
signed int lon_prev = 0;
unsigned char gps_add_counter;
unsigned char new_gps_data_counter;
signed int l_lat_waypoint;
signed int l_lon_waypoint;
signed int l_lat_gps;
signed int l_lon_gps;
signed int lat_gps_previous;
signed int lon_gps_previous;
signed int lat_gps_actual;
signed int lon_gps_actual;
float lat_gps_loop_add;
float lon_gps_loop_add;
float lat_gps_add;
float lon_gps_add;
unsigned char new_gps_data_available;
float gps_roll_adjust;
float gps_pitch_adjust;
#define GPS_PD_MAX 6000
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
void Read_Gps(void);
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

EP_PIDGain_Read(0, &roll.in.kp, &roll.in.ki, &roll.in.kd);
Encode_Msg_PID_Gain(&telemetry_tx_buf[0], 0, roll.in.kp, roll.in.ki, roll.in.kd);
HAL_UART_Transmit(&huart1, &telemetry_tx_buf[0], 20, 10);

EP_PIDGain_Read(1, &roll.out.kp, &roll.out.ki, &roll.out.kd);
Encode_Msg_PID_Gain(&telemetry_tx_buf[0], 1, roll.out.kp, roll.out.ki, roll.out.kd);
HAL_UART_Transmit(&huart1, &telemetry_tx_buf[0], 20, 10);

EP_PIDGain_Read(2, &pitch.in.kp, &pitch.in.ki, &pitch.in.kd);
Encode_Msg_PID_Gain(&telemetry_tx_buf[0], 2, pitch.in.kp, pitch.in.ki, pitch.in.kd);
HAL_UART_Transmit(&huart1, &telemetry_tx_buf[0], 20, 10);

EP_PIDGain_Read(3, &pitch.out.kp, &pitch.out.ki, &pitch.out.kd);
Encode_Msg_PID_Gain(&telemetry_tx_buf[0], 3, pitch.out.kp, pitch.out.ki, pitch.out.kd);
HAL_UART_Transmit(&huart1, &telemetry_tx_buf[0], 20, 10);

EP_PIDGain_Read(4, &yaw_heading.kp, &yaw_heading.ki, &yaw_heading.kd);
Encode_Msg_PID_Gain(&telemetry_tx_buf[0], 4, yaw_heading.kp, yaw_heading.ki, yaw_heading.kd);
HAL_UART_Transmit(&huart1, &telemetry_tx_buf[0], 20, 10);

EP_PIDGain_Read(5, &yaw_rate.kp, &yaw_rate.ki, &yaw_rate.kd);
Encode_Msg_PID_Gain(&telemetry_tx_buf[0], 5, yaw_rate.kp, yaw_rate.ki, yaw_rate.kd);
HAL_UART_Transmit(&huart1, &telemetry_tx_buf[0], 20, 10);

// Altitude Hold PID Gain
altitude.out.kp = 2;
altitude.out.ki = 0;
altitude.out.kd = 0.01;
altitude.in.kp = 1000;
altitude.in.ki = 10;
altitude.in.kd = 0;

// GPS Hold PID Gain
lat.out.kp = 0.3;
lat.out.ki = 0;
lat.out.kd = 0.5;

lat.in.kp = 10;
lat.in.ki = 1;
lat.in.kd = 0;

lon.out.kp = 0.3;
lon.out.ki = 0;
lon.out.kd = 0.5;

lon.in.kp = 10;
lon.in.ki = 1;
lon.in.kd = 0;

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
		  LPS22HH.baroAlt = getAltitude2(LPS22HH.pressure_raw/4096.f, LPS22HH.temperature_raw/100.f);
		  baro_offset += LPS22HH.baroAlt;
		  HAL_Delay(100);

		  baro_cnt++;
	  }
  }
  baro_offset = baro_offset / baro_cnt;

  // Read Initial GPS
//  while(lat_gps_home == 0 && lon_gps_home == 0)
//  {
//	  HAL_Delay(1);
//	  Read_Gps();
//
//	  lat_gps_home = l_lat_gps;
//	  lat_gps_home = l_lon_gps;
//  }

  // Read Battery Voltage
  batVolt = adcVal * 0.00699563f;

  /********************* FC Ready to Fly ************************/

  LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH4); //Enable Timer Counting

  TIM3->PSC = 2000;
  HAL_Delay(100);
  TIM3->PSC = 1500;
  HAL_Delay(100);
  TIM3->PSC = 1000;
  HAL_Delay(100);

  LL_TIM_CC_DisableChannel(TIM3, LL_TIM_CHANNEL_CH4);


("Start\n");

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

		  Double_Roll_Pitch_PID_Calculation(&pitch, (iBus.RV - 1500)*0.07f, BNO080_Pitch, ICM20602.gyro_x);
		  Double_Roll_Pitch_PID_Calculation(&roll, (iBus.RH - 1500)*0.07f, BNO080_Roll, ICM20602.gyro_y);

		  // Choose Flight Mode
		  if(iBus.LV < 1550 && iBus.LV > 1400) is_throttle_middle = 1;
		  else is_throttle_middle = 0;

		  if(iBus.LH > 1485 && iBus.LH < 1515) is_yaw_middle = 1;
		  else is_yaw_middle = 0;

		  flight_mode = 1;
		  if(iBus.SwA == 2000 && iBus.SwB == 1000 && iBus.SwD == 2000 && is_throttle_middle == 1) flight_mode = 2;
		  else if(iBus.SwA == 2000 && iBus.SwB == 2000 && is_throttle_middle == 1) flight_mode = 3;


		  if(flight_mode == 2) //Altitude Holding Mode
		  {
			  if(iBus.VrB < 1100) iBus_VrB_flag = 0;
			  else if(iBus.VrB > 1900) iBus_VrB_flag = 2;
			  else iBus_VrB_flag = 1;

			  if(iBus_VrB_flag==0 && iBus_VrB_Prev_flag==1) altitude_setpoint -= 0.5f;
			  else if(iBus_VrB_flag==2 && iBus_VrB_Prev_flag==1) altitude_setpoint += 0.5f;

			  iBus_VrB_Prev_flag = iBus_VrB_flag;

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
				  ccr4 = (unsigned int)((float)ccr4 * MOTOR_FREQ_ADJUST);
			  }
			  else
			  {
				  Single_Yaw_Heading_PID_Calculation(&yaw_heading, yaw_heading_reference, BNO080_Yaw, ICM20602.gyro_z);
				  ccr1 = 84000 + takeoff_throttle - pitch.in.pid_result + roll.in.pid_result - yaw_heading.pid_result + altitude.in.pid_result;
				  ccr2 = 84000 + takeoff_throttle + pitch.in.pid_result + roll.in.pid_result + yaw_heading.pid_result + altitude.in.pid_result;
				  ccr3 = 84000 + takeoff_throttle + pitch.in.pid_result - roll.in.pid_result - yaw_heading.pid_result + altitude.in.pid_result;
				  ccr4 = 84000 + takeoff_throttle - pitch.in.pid_result - roll.in.pid_result + yaw_heading.pid_result + altitude.in.pid_result;
				  ccr2 = (unsigned int)((float)ccr2 * MOTOR_FREQ_ADJUST);
				  ccr4 = (unsigned int)((float)ccr4 * MOTOR_FREQ_ADJUST);
			  }
		  }
		  else if(flight_mode == 3 ) //GPS holding Mode
		  {
			  Double_Altitude_PID_Calculation(&altitude, altitude_setpoint, actual_pressure_fast);

			  Double_GPS_PID_Calculation(&lat, l_lat_waypoint, l_lat_gps);
			  Double_GPS_PID_Calculation(&lon, l_lon_waypoint, l_lon_gps);

			  //Because the correction is calculated as if the nose was facing north, we need to convert it for the current heading.
			  gps_roll_adjust = ((float)lon.in.pid_result * cos((360.f - BNO080_Yaw) * 0.017453)) + ((float)lat.in.pid_result * sin((360.f - BNO080_Yaw) * 0.017453));
			  gps_pitch_adjust = ((float)lat.in.pid_result * cos((360.f - BNO080_Yaw) * 0.017453)) - ((float)lon.in.pid_result * sin((360.f - BNO080_Yaw) * 0.017453));

			  //Limit the maximum correction to 300. This way we still have full controll with the pitch and roll stick on the transmitter.
			  if (gps_roll_adjust > GPS_PD_MAX) gps_roll_adjust = GPS_PD_MAX;
			  if (gps_roll_adjust < GPS_PD_MIN) gps_roll_adjust = GPS_PD_MIN;
			  if (gps_pitch_adjust > GPS_PD_MAX) gps_pitch_adjust = GPS_PD_MAX;
			  if (gps_pitch_adjust < GPS_PD_MIN) gps_pitch_adjust = GPS_PD_MIN;

			  if(is_yaw_middle == 0)
			  {
				  yaw_heading_reference = BNO080_Yaw;
				  Single_Yaw_Rate_PID_Calculation(&yaw_rate, (iBus.LH-1500), ICM20602.gyro_z);
				  ccr1 = 84000 + takeoff_throttle - pitch.in.pid_result + roll.in.pid_result - yaw_rate.pid_result + altitude.in.pid_result - gps_pitch_adjust + gps_roll_adjust;
				  ccr2 = 84000 + takeoff_throttle + pitch.in.pid_result + roll.in.pid_result + yaw_rate.pid_result + altitude.in.pid_result + gps_pitch_adjust + gps_roll_adjust;
				  ccr3 = 84000 + takeoff_throttle + pitch.in.pid_result - roll.in.pid_result - yaw_rate.pid_result + altitude.in.pid_result + gps_pitch_adjust - gps_roll_adjust;
				  ccr4 = 84000 + takeoff_throttle - pitch.in.pid_result - roll.in.pid_result + yaw_rate.pid_result + altitude.in.pid_result - gps_pitch_adjust - gps_roll_adjust;
				  ccr2 = (unsigned int)((float)ccr2 * MOTOR_FREQ_ADJUST);
				  ccr4 = (unsigned int)((float)ccr4 * MOTOR_FREQ_ADJUST);
			  }
			  else
			  {
				  Single_Yaw_Heading_PID_Calculation(&yaw_heading, yaw_heading_reference , BNO080_Yaw, ICM20602.gyro_z);
				  ccr1 = 84000 + takeoff_throttle - pitch.in.pid_result + roll.in.pid_result - yaw_heading.pid_result + altitude.in.pid_result - gps_pitch_adjust + gps_roll_adjust;
				  ccr2 = 84000 + takeoff_throttle + pitch.in.pid_result + roll.in.pid_result + yaw_heading.pid_result + altitude.in.pid_result + gps_pitch_adjust + gps_roll_adjust;
				  ccr3 = 84000 + takeoff_throttle + pitch.in.pid_result - roll.in.pid_result - yaw_heading.pid_result + altitude.in.pid_result + gps_pitch_adjust - gps_roll_adjust;
				  ccr4 = 84000 + takeoff_throttle - pitch.in.pid_result - roll.in.pid_result + yaw_heading.pid_result + altitude.in.pid_result - gps_pitch_adjust - gps_roll_adjust;
				  ccr2 = (unsigned int)((float)ccr2 * MOTOR_FREQ_ADJUST);
				  ccr4 = (unsigned int)((float)ccr4 * MOTOR_FREQ_ADJUST);
			  }
		  }
		  else if(flight_mode == 4 ) // Return to Home Mode
		  {
			  Double_Altitude_PID_Calculation(&altitude, altitude_setpoint, actual_pressure_fast);

			  if (gps_add_counter >= 0)gps_add_counter --;
			  Read_Gps();

			  if(l_lat_waypoint == 0) l_lat_waypoint = l_lat_gps;
			  else if(l_lon_waypoint == 0) l_lon_waypoint = l_lon_gps;

			  if (l_lat_gps_float_adjust > 1) {
				  l_lat_waypoint ++;
				  l_lat_gps_float_adjust --;
			  }
			  if (l_lat_gps_float_adjust < -1) {
				  l_lat_waypoint --;
				  l_lat_gps_float_adjust ++;
			  }

			  if (l_lon_gps_float_adjust > 1) {
				  l_lon_waypoint ++;
				  l_lon_gps_float_adjust --;
			  }
			  if (l_lon_gps_float_adjust < -1) {
				  l_lon_waypoint --;
				  l_lon_gps_float_adjust ++;
			  }

			  Double_GPS_PID_Calculation(&lat, l_lat_waypoint, l_lat_gps);
			  Double_GPS_PID_Calculation(&lon, l_lon_waypoint, l_lon_gps);

			  //Because the correction is calculated as if the nose was facing north, we need to convert it for the current heading.
			  gps_roll_adjust = ((float)lon.in.pid_result * cos(BNO080_Yaw * 0.017453)) + ((float)lat.in.pid_result * cos((BNO080_Yaw - 90) * 0.017453));
			  gps_pitch_adjust = ((float)lat.in.pid_result * cos(BNO080_Yaw * 0.017453)) + ((float)lon.in.pid_result * cos((BNO080_Yaw + 90) * 0.017453));

			  //Limit the maximum correction to 300. This way we still have full controll with the pitch and roll stick on the transmitter.
			  if (gps_roll_adjust > GPS_PD_MAX) gps_roll_adjust = GPS_PD_MAX;
			  if (gps_roll_adjust < GPS_PD_MIN) gps_roll_adjust = GPS_PD_MIN;
			  if (gps_pitch_adjust > GPS_PD_MAX) gps_pitch_adjust = GPS_PD_MAX;
			  if (gps_pitch_adjust < GPS_PD_MIN) gps_pitch_adjust = GPS_PD_MIN;

			  Single_Yaw_Heading_PID_Calculation(&yaw_heading, 0 , BNO080_Yaw, ICM20602.gyro_z);
			  ccr1 = 84000 + takeoff_throttle - pitch.in.pid_result + roll.in.pid_result - yaw_heading.pid_result + altitude.in.pid_result - gps_pitch_adjust + gps_roll_adjust;
			  ccr2 = 84000 + takeoff_throttle + pitch.in.pid_result + roll.in.pid_result + yaw_heading.pid_result + altitude.in.pid_result + gps_pitch_adjust + gps_roll_adjust;
			  ccr3 = 84000 + takeoff_throttle + pitch.in.pid_result - roll.in.pid_result - yaw_heading.pid_result + altitude.in.pid_result + gps_pitch_adjust - gps_roll_adjust;
			  ccr4 = 84000 + takeoff_throttle - pitch.in.pid_result - roll.in.pid_result + yaw_heading.pid_result + altitude.in.pid_result - gps_pitch_adjust - gps_roll_adjust;
			  ccr2 = (unsigned int)((float)ccr2 * MOTOR_FREQ_ADJUST);
			  ccr4 = (unsigned int)((float)ccr4 * MOTOR_FREQ_ADJUST);
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
				  ccr4 = (unsigned int)((float)ccr4 * MOTOR_FREQ_ADJUST);
			  }
			  else
			  {
				  Single_Yaw_Heading_PID_Calculation(&yaw_heading, yaw_heading_reference, BNO080_Yaw, ICM20602.gyro_z);
				  ccr1 = 84000 + (iBus.LV - 1000) * 83.9 - pitch.in.pid_result + roll.in.pid_result -yaw_heading.pid_result;
				  ccr2 = 84000 + (iBus.LV - 1000) * 83.9 + pitch.in.pid_result + roll.in.pid_result +yaw_heading.pid_result;
				  ccr3 = 84000 + (iBus.LV - 1000) * 83.9 + pitch.in.pid_result - roll.in.pid_result -yaw_heading.pid_result;
				  ccr4 = 84000 + (iBus.LV - 1000) * 83.9 - pitch.in.pid_result - roll.in.pid_result +yaw_heading.pid_result;
				  ccr2 = (unsigned int)((float)ccr2 * MOTOR_FREQ_ADJUST);
				  ccr4 = (unsigned int)((float)ccr4 * MOTOR_FREQ_ADJUST);
			  }

			  altitude_setpoint = actual_pressure_fast;
			  Reset_PID_Integrator(&altitude.out);
			  Reset_PID_Integrator(&altitude.in);

			  l_lat_waypoint = l_lat_gps;
			  l_lon_waypoint = l_lon_gps;
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
	  if(tim7_20ms_flag == 1 && tim7_100ms_flag == 0)
	  {
		  tim7_20ms_flag = 0;
//		  Encode_Msg_AHRS(&telemetry_tx_buf[0]);
//		  HAL_UART_Transmit_IT(&huart1, &telemetry_tx_buf[0], 20);
	  }

	  else if(tim7_20ms_flag == 1 && tim7_100ms_flag == 1)
	  {
		  tim7_20ms_flag = 0;
		  tim7_100ms_flag = 0;
//		  Encode_Msg_AHRS(&telemetry_tx_buf[0]);
//		  HAL_UART_Transmit_IT(&huart1, &telemetry_tx_buf[0], 40);
//		  Encode_Msg_Altitude(&telemetry_tx_buf[0]);
		  Encode_Msg_Gps(&telemetry_tx_buf[0]);
		  HAL_UART_Transmit_DMA(&huart1, &telemetry_tx_buf[0], 35); // altitude : 26, gps : 35
	  }

	  if (gps_add_counter >= 0)gps_add_counter --;
	  Read_Gps();

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

		  float theta = 360.f - BNO080_Yaw;
		  float theta_radian = theta * 0.01745329252f;
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

		  LPS22HH.baroAlt = getAltitude2(LPS22HH.pressure_raw/4096.f, LPS22HH.temperature_raw/100.f); //Default Unit = 1m
		  LPS22HH.baroAltGround = LPS22HH.baroAlt - baro_offset;

		  //moving average of altitude
		  pressure_total_average -= pressure_rotating_mem[pressure_rotating_mem_location];
		  pressure_rotating_mem[pressure_rotating_mem_location] = LPS22HH.baroAltGround;
		  pressure_total_average += pressure_rotating_mem[pressure_rotating_mem_location];
		  pressure_rotating_mem_location++;
		  if(pressure_rotating_mem_location == 5) pressure_rotating_mem_location = 0;
		  actual_pressure_fast = pressure_total_average / 5.0f;

		  //1st order IIR
/*		  actual_pressure_slow = actual_pressure_slow * 0.985f + actual_pressure_fast * 0.015f;
		  actual_pressure_diff = actual_pressure_slow - actual_pressure_fast;
		  if (actual_pressure_diff > 4)actual_pressure_diff = 4;
		  if (actual_pressure_diff < -4)actual_pressure_diff = -4;
		  if (actual_pressure_diff > 0.5 || actual_pressure_diff < -0.5) actual_pressure_slow -= actual_pressure_diff / 3.0;
		   actual_pressure = actual_pressure_slow; */
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

	  if(batVolt == 0) batVolt = adcVal * 0.00699563f;
	  batVolt = 0.98 * batVolt_prev + 0.02 * (adcVal * 0.00699563f);
	  batVolt_prev = batVolt;
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




			("%f,%f,%f,", x, y, z);
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

("\nCalibration data successfully stored\n");
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
					if(uart1_rx_data==0x47)
					{
						telemetry_rx_buf[cnt]=uart1_rx_data;
						cnt++;
					}
					break;
				case 1:
					if(uart1_rx_data==0x53)
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

	  telemetry_tx_buf[19] = 0xff;

	  for(int i=0; i<19; i++)
	  {
		  telemetry_tx_buf[19] = telemetry_tx_buf[19] - telemetry_tx_buf[i];
	  }
}

void Receive_Pid_Gain(void)
{
	  if(telemetry_rx_cplt_flag == 1) //Receive GCS Message
		  	  {
		  		  telemetry_rx_cplt_flag = 0;

		  		  if(iBus.SwA == 1000) //Check FS-i6 Switch A
		  		  {
		  			  unsigned char chksum = 0xff;
		  			  for(int i=0;i<19;i++) chksum = chksum - telemetry_rx_buf[i];

		  			  if(chksum == telemetry_rx_buf[19]) //Check checksum of GCS Message
		  			  {
		  				  LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH4);

		  				  TIM3->PSC = 1000;
		  				  HAL_Delay(10);

		  				  LL_TIM_CC_DisableChannel(TIM3, LL_TIM_CHANNEL_CH4);

		  				  switch(telemetry_rx_buf[2]) //Check ID of GCS Message
		  				  {
		  				  case 0:
		  					  roll.in.kp = *(float*)&telemetry_rx_buf[3];
		  					  roll.in.ki = *(float*)&telemetry_rx_buf[7];
		  					  roll.in.kd = *(float*)&telemetry_rx_buf[11];
		  					  EP_PIDGain_Write(telemetry_rx_buf[2], roll.in.kp, roll.in.ki, roll.in.kd);
		  					  EP_PIDGain_Read(telemetry_rx_buf[2], &roll.in.kp, &roll.in.ki, &roll.in.kd);
		  					  Encode_Msg_PID_Gain(&telemetry_tx_buf[0], telemetry_rx_buf[2], roll.in.kp, roll.in.ki, roll.in.kd);
		  					  HAL_UART_Transmit_IT(&huart1, &telemetry_tx_buf[0], 20);
		  					  break;
		  				  case 1:
		  					  roll.out.kp = *(float*)&telemetry_rx_buf[3];
		  					  roll.out.ki = *(float*)&telemetry_rx_buf[7];
		  					  roll.out.kd = *(float*)&telemetry_rx_buf[11];
		  					  EP_PIDGain_Write(telemetry_rx_buf[2], roll.out.kp, roll.out.ki, roll.out.kd);
		  					  EP_PIDGain_Read(telemetry_rx_buf[2], &roll.out.kp, &roll.out.ki, &roll.out.kd);
		  					  Encode_Msg_PID_Gain(&telemetry_tx_buf[0], telemetry_rx_buf[2], roll.out.kp, roll.out.ki, roll.out.kd);
		  					  HAL_UART_Transmit_IT(&huart1, &telemetry_tx_buf[0], 20);
		  					  break;
		  				  case 2:
		  					  pitch.in.kp = *(float*)&telemetry_rx_buf[3];
		  					  pitch.in.ki = *(float*)&telemetry_rx_buf[7];
		  					  pitch.in.kd = *(float*)&telemetry_rx_buf[11];
		  					  EP_PIDGain_Write(telemetry_rx_buf[2], pitch.in.kp, pitch.in.ki, pitch.in.kd);
		  					  EP_PIDGain_Read(telemetry_rx_buf[2], &pitch.in.kp, &pitch.in.ki, &pitch.in.kd);
		  					  Encode_Msg_PID_Gain(&telemetry_tx_buf[0], telemetry_rx_buf[2], pitch.in.kp, pitch.in.ki, pitch.in.kd);
		  					  HAL_UART_Transmit_IT(&huart1, &telemetry_tx_buf[0], 20);
		  					  break;
		  				  case 3:
		  					  pitch.out.kp = *(float*)&telemetry_rx_buf[3];
		  					  pitch.out.ki = *(float*)&telemetry_rx_buf[7];
		  					  pitch.out.kd = *(float*)&telemetry_rx_buf[11];
		  					  EP_PIDGain_Write(telemetry_rx_buf[2], pitch.out.kp, pitch.out.ki, pitch.out.kd);
		  					  EP_PIDGain_Read(telemetry_rx_buf[2], &pitch.out.kp, &pitch.out.ki, &pitch.out.kd);
		  					  Encode_Msg_PID_Gain(&telemetry_tx_buf[0], telemetry_rx_buf[2], pitch.out.kp, pitch.out.ki, pitch.out.kd);
		  					  HAL_UART_Transmit_IT(&huart1, &telemetry_tx_buf[0], 20);
		  					  break;
		  				  case 4:
		  					  yaw_heading.kp = *(float*)&telemetry_rx_buf[3];
		  					  yaw_heading.ki = *(float*)&telemetry_rx_buf[7];
		  					  yaw_heading.kd = *(float*)&telemetry_rx_buf[11];
		  					  EP_PIDGain_Write(telemetry_rx_buf[2], yaw_heading.kp, yaw_heading.ki, yaw_heading.kd);
		  					  EP_PIDGain_Read(telemetry_rx_buf[2], &yaw_heading.kp, &yaw_heading.ki, &yaw_heading.kd);
		  					  Encode_Msg_PID_Gain(&telemetry_tx_buf[0], telemetry_rx_buf[2], yaw_heading.kp, yaw_heading.ki, yaw_heading.kd);
		  					  HAL_UART_Transmit_IT(&huart1, &telemetry_tx_buf[0], 20);
		  					  break;
		  				  case 5:
		  					  yaw_rate.kp = *(float*)&telemetry_rx_buf[3];
		  					  yaw_rate.ki = *(float*)&telemetry_rx_buf[7];
		  					  yaw_rate.kd = *(float*)&telemetry_rx_buf[11];
		  					  EP_PIDGain_Write(telemetry_rx_buf[2], yaw_rate.kp, yaw_rate.ki, yaw_rate.kd);
		  					  EP_PIDGain_Read(telemetry_rx_buf[2], &yaw_rate.kp, &yaw_rate.ki, &yaw_rate.kd);
		  					  Encode_Msg_PID_Gain(&telemetry_tx_buf[0], telemetry_rx_buf[2], yaw_rate.kp, yaw_rate.ki, yaw_rate.kd);
		  					  HAL_UART_Transmit_IT(&huart1, &telemetry_tx_buf[0], 20);
		  					  break;
		  				  case 0x10:
		  					  switch(telemetry_rx_buf[3]) //Check PID Gain ID of GCS PID Gain Request Message
		  					  {
		  					  case 0:
		  						  Encode_Msg_PID_Gain(&telemetry_tx_buf[0], telemetry_rx_buf[3], roll.in.kp, roll.in.ki, roll.in.kd);
		  						  HAL_UART_Transmit(&huart1, &telemetry_tx_buf[0], 20, 10);
		  						  break;
		  					  case 1:
		  						  Encode_Msg_PID_Gain(&telemetry_tx_buf[0], telemetry_rx_buf[3], roll.out.kp, roll.out.ki, roll.out.kd);
		  						  HAL_UART_Transmit(&huart1, &telemetry_tx_buf[0], 20, 10);
		  						  break;
		  					  case 2:
		  						  Encode_Msg_PID_Gain(&telemetry_tx_buf[0], telemetry_rx_buf[3], pitch.in.kp, pitch.in.ki, pitch.in.kd);
		  						  HAL_UART_Transmit(&huart1, &telemetry_tx_buf[0], 20, 10);
		  						  break;
		  					  case 3:
		  						  Encode_Msg_PID_Gain(&telemetry_tx_buf[0], telemetry_rx_buf[3], pitch.out.kp, pitch.out.ki, pitch.out.kd);
		  						  HAL_UART_Transmit(&huart1, &telemetry_tx_buf[0], 20, 10);
		  						  break;
		  					  case 4:
		  						  Encode_Msg_PID_Gain(&telemetry_tx_buf[0], telemetry_rx_buf[3], yaw_heading.kp, yaw_heading.ki, yaw_heading.kd);
		  						  HAL_UART_Transmit(&huart1, &telemetry_tx_buf[0], 20, 10);
		  						  break;
		  					  case 5:
		  						  Encode_Msg_PID_Gain(&telemetry_tx_buf[0], telemetry_rx_buf[3], yaw_rate.kp, yaw_rate.ki, yaw_rate.kd);
		  						  HAL_UART_Transmit(&huart1, &telemetry_tx_buf[0], 20, 10);
		  						  break;
		  					  case 6:
		  						  Encode_Msg_PID_Gain(&telemetry_tx_buf[0], 0, roll.in.kp, roll.in.ki, roll.in.kd);
		  						  HAL_UART_Transmit(&huart1, &telemetry_tx_buf[0], 20, 10);
		  						  Encode_Msg_PID_Gain(&telemetry_tx_buf[0], 1, roll.out.kp, roll.out.ki, roll.out.kd);
		  						  HAL_UART_Transmit(&huart1, &telemetry_tx_buf[0], 20, 10);
		  						  Encode_Msg_PID_Gain(&telemetry_tx_buf[0], 2, pitch.in.kp, pitch.in.ki, pitch.in.kd);
		  						  HAL_UART_Transmit(&huart1, &telemetry_tx_buf[0], 20, 10);
		  						  Encode_Msg_PID_Gain(&telemetry_tx_buf[0], 3, pitch.out.kp, pitch.out.ki, pitch.out.kd);
		  						  HAL_UART_Transmit(&huart1, &telemetry_tx_buf[0], 20, 10);
		  						  Encode_Msg_PID_Gain(&telemetry_tx_buf[0], 4, yaw_heading.kp, yaw_heading.ki, yaw_heading.kd);
		  						  HAL_UART_Transmit(&huart1, &telemetry_tx_buf[0], 20, 10);
		  						  Encode_Msg_PID_Gain(&telemetry_tx_buf[0], 5, yaw_rate.kp, yaw_rate.ki, yaw_rate.kd);
		  						  HAL_UART_Transmit(&huart1, &telemetry_tx_buf[0], 20, 10);
		  						  break;
		  					  }
		  					  break;
		  				  }
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

	telemetry_tx_buf[6] = pvt.numSV;

	telemetry_tx_buf[7] = (int)BNO080_Yaw >> 24;
	telemetry_tx_buf[8] = (int)BNO080_Yaw >> 16;
	telemetry_tx_buf[9] = (int)BNO080_Yaw >> 8;
	telemetry_tx_buf[10] = (int)BNO080_Yaw;

	telemetry_tx_buf[11] = l_lat_gps >> 24;
	telemetry_tx_buf[12] = l_lat_gps >> 16;
	telemetry_tx_buf[13] = l_lat_gps >> 8;
	telemetry_tx_buf[14] = l_lat_gps;

	telemetry_tx_buf[15] = l_lon_gps >> 24;
	telemetry_tx_buf[16] = l_lon_gps >> 16;
	telemetry_tx_buf[17] = l_lon_gps >> 8;
	telemetry_tx_buf[18] = l_lon_gps;

	telemetry_tx_buf[19] = l_lat_waypoint >> 24;
	telemetry_tx_buf[20] = l_lat_waypoint >> 16;
	telemetry_tx_buf[21] = l_lat_waypoint >> 8;
	telemetry_tx_buf[22] = l_lat_waypoint;

	telemetry_tx_buf[23] = l_lon_waypoint >> 24;
	telemetry_tx_buf[24] = l_lon_waypoint >> 16;
	telemetry_tx_buf[25] = l_lon_waypoint >> 8;
	telemetry_tx_buf[26] = l_lon_waypoint;

	telemetry_tx_buf[27] = (int)gps_pitch_adjust >> 24;
	telemetry_tx_buf[28] = (int)gps_pitch_adjust >> 16;
	telemetry_tx_buf[29] = (int)gps_pitch_adjust >> 8;
	telemetry_tx_buf[30] = (int)gps_pitch_adjust;

	telemetry_tx_buf[31] = (int)gps_roll_adjust >> 24;
	telemetry_tx_buf[32] = (int)gps_roll_adjust >> 16;
	telemetry_tx_buf[33] = (int)gps_roll_adjust >> 8;
	telemetry_tx_buf[34] = (int)gps_roll_adjust;
}

void Read_Gps(void)
{
	  /********************* GPS Data Parsing ************************/
	  if(m8n_rx_cplt_flag == 1) // GPS receive checking
	  {
		  m8n_rx_cplt_flag = 0;

		  if(M8N_UBX_CHKSUM_Check(&m8n_rx_buf[0], 100) == 1)
		  {
			  LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_2);
			  M8N_UBX_NAV_PVT_Parsing(&m8n_rx_buf[0], &pvt);
//	 		  M8N_UBX_NAV_POSLLH_Parsing(&m8n_rx_buf[0], &posllh);

//			  if((pvt.lat - pvt.lat_prev > 500) || (pvt.lat - pvt.lat_prev < -500)) pvt.lat = lat_prev;
//			  if((pvt.lon - pvt.lon_prev > 500) || (pvt.lon - pvt.lon_prev < -500)) pvt.lon = lon_prev;
//			  lat_prev = pvt.lat;
//			  lon_prev = pvt.lon;

			  lat_gps_actual = pvt.lat;
			  lon_gps_actual = pvt.lon;
		  }
		  else
		  {
			  l_lat_gps = 0;
			  l_lon_gps = 0;
			  lat_gps_previous = 0;
			  lon_gps_previous = 0;
		  }

		  if (lat_gps_previous == 0 && lon_gps_previous == 0) {                                              //If this is the first time the GPS code is used.
			  lat_gps_previous = lat_gps_actual;                                                               //Set the lat_gps_previous variable to the lat_gps_actual variable.
			  lon_gps_previous = lon_gps_actual;                                                               //Set the lon_gps_previous variable to the lon_gps_actual variable.
		  }

		  lat_gps_loop_add = (float)(lat_gps_actual - lat_gps_previous) / 20.0;                              //Divide the difference between the new and previous latitude by ten.
		  lon_gps_loop_add = (float)(lon_gps_actual - lon_gps_previous) / 20.0;                              //Divide the difference between the new and previous longitude by ten.

		  l_lat_gps = lat_gps_previous;                                                                      //Set the l_lat_gps variable to the previous latitude value.
		  l_lon_gps = lon_gps_previous;                                                                      //Set the l_lon_gps variable to the previous longitude value.

		  lat_gps_previous = lat_gps_actual;                                                                 //Remember the new latitude value in the lat_gps_previous variable for the next loop.
		  lon_gps_previous = lon_gps_actual;                                                                 //Remember the new longitude value in the lat_gps_previous variable for the next loop.

		  //The GPS is set to a 5Hz refresh rate. Between every 2 GPS measurments, 19 GPS values are simulated.
		  gps_add_counter = 10;                                                                               //Set the gps_add_counter variable to 10 as a count down loop timer
		  new_gps_data_counter = 19;                                                                          //Set the new_gps_data_counter to 19. This is the number of simulated values between 2 GPS measurements.
		  lat_gps_add = 0;                                                                                   //Reset the lat_gps_add variable.
		  lon_gps_add = 0;                                                                                   //Reset the lon_gps_add variable.
		  new_gps_data_available = 1;
	  }

	  //After 10 program loops 10 x 1ms = 10ms the gps_add_counter is 0.
	  if (gps_add_counter == 0 && new_gps_data_counter > 0) {                                                 //If gps_add_counter is 0 and there are new GPS simulations needed.
	    new_gps_data_available = 1;                                                                           //Set the new_gps_data_available to indicate that there is new data available.
	    new_gps_data_counter --;                                                                              //Decrement the new_gps_data_counter so there will only be 9 simulations
	    gps_add_counter = 10;                                                                                  //Set the gps_add_counter variable to 5 as a count down loop timer

	    lat_gps_add += lat_gps_loop_add;                                                                      //Add the simulated part to a buffer float variable because the l_lat_gps can only hold integers.
	    if ((lat_gps_add >= 10) || (lat_gps_add <= -10)) {                                                                          //If the absolute value of lat_gps_add is larger then 1.
	      l_lat_gps += (int)lat_gps_add;                                                                      //Increment the lat_gps_add value with the lat_gps_add value as an integer. So no decimal part.
	      lat_gps_add -= (int)lat_gps_add;                                                                    //Subtract the lat_gps_add value as an integer so the decimal value remains.
	    }

	    lon_gps_add += lon_gps_loop_add;                                                                      //Add the simulated part to a buffer float variable because the l_lat_gps can only hold integers.
	    if ((lon_gps_add >= 10) || (lon_gps_add <= -10)) {                                                                          //If the absolute value of lat_gps_add is larger then 1.
	      l_lon_gps += (int)lon_gps_add;                                                                      //Increment the lat_gps_add value with the lat_gps_add value as an integer. So no decimal part.
	      lon_gps_add -= (int)lon_gps_add;                                                                    //Subtract the lat_gps_add value as an integer so the decimal value remains.
	    }
	  }
}

void return_to_home(void) {

	if (flight_mode == 4) {
		/////////////////////////////////////////////////////////////////////////////////////////////////////////
		//Step 0 - make some basic calculations
		/////////////////////////////////////////////////////////////////////////////////////////////////////////
		if (return_to_home_step == 0) {
			//Is the quadcopter nearby? Then land without returning to home.
			if( l_lat_waypoint - lat_gps_home < 100 && l_lat_waypoint - lat_gps_home > -100 ) is_lat_nearby = 1;
			if( l_lon_waypoint - lon_gps_home < 100 && l_lon_waypoint - lon_gps_home > -100 ) is_lon_nearby = 1;

			if (is_lat_nearby == 1 && is_lon_nearby == 1)return_to_home_step = 3;
			else {
				return_to_home_move_factor = 0.0;
				if (return_to_home_lat_factor == 1 || return_to_home_lon_factor == 1)return_to_home_step = 1;
				//cos(((float)l_lat_gps / 1000000.0)
				if (abs_int(lat_gps_home, l_lat_waypoint) >= abs_int(lon_gps_home, l_lon_waypoint)) {
					return_to_home_lon_factor = (float)abs_int(lon_gps_home, l_lon_waypoint) / (float)abs_int(lat_gps_home, l_lat_waypoint);
					return_to_home_lat_factor = 1;
				}
				else {
					return_to_home_lon_factor = 1;
					return_to_home_lat_factor = (float)abs_int(lat_gps_home, l_lat_waypoint) / (float)abs_int(lon_gps_home, l_lon_waypoint);
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
				altitude_setpoint -= 0.05;
				return_to_home_decrease -= 0.05;
			}
		}
		/////////////////////////////////////////////////////////////////////////////////////////////////////////
		//Step 2 - Return to the home position
		/////////////////////////////////////////////////////////////////////////////////////////////////////////
		if (return_to_home_step == 2) {
			if (lat_gps_home == l_lat_waypoint && lon_gps_home == l_lon_waypoint)return_to_home_step = 3;
			if (abs_int(lat_gps_home, l_lat_waypoint) < 160 && abs_int(lon_gps_home, l_lon_waypoint) < 160 && return_to_home_move_factor > 0.05)return_to_home_move_factor -= 0.00015;
			else if (return_to_home_move_factor < 0.20)return_to_home_move_factor += 0.0001;

			if (lat_gps_home != l_lat_waypoint) {
				if (lat_gps_home > l_lat_waypoint) l_lat_gps_float_adjust += return_to_home_move_factor * return_to_home_lat_factor;
				if (lat_gps_home < l_lat_waypoint) l_lat_gps_float_adjust -= return_to_home_move_factor * return_to_home_lat_factor;
			}
			if (lon_gps_home != l_lon_waypoint) {
				if (lon_gps_home > l_lon_waypoint) l_lon_gps_float_adjust += return_to_home_move_factor * return_to_home_lon_factor;
				if (lon_gps_home < l_lon_waypoint) l_lon_gps_float_adjust -= return_to_home_move_factor * return_to_home_lon_factor;
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
	takeoff_throttle = 84 * ( batVolt * (-21.765) + 1874.829 - 1000);
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
