//References:
// https://www.servotecnica.com/en/resources/white-papers-en-mobile/dual-loop-advanced-control-techniques-for-real-world-drivetrains/
// https://controlguru.com/the-cascade-control-architecture/

/**
 * PID control.c
 * @author ChrisP @ M-HIVE

 * This library source code is for cascade double loop pid control for STM32 Drone Development online course.
 *
 * Created by ChrisP(Wonyeob Park) @ M-HIVE Embedded Academy, July, 2020
 * Rev. 1.0
 *
 * Where to take the online course.
 * https://www.inflearn.com/course/STM32CubelDE-STM32F4%EB%93%9C%EB%A1%A0-%EA%B0%9C%EB%B0%9C (Korean language supported only)
 *
 * Where to buy MH-FC V2.2 STM32F4 Drone Flight Controller.
 * https://smartstore.naver.com/mhivestore/products/4961922335
 *
 * https://github.com/ChrisWonyeobPark
 * https://blog.naver.com/lbiith
 * https://cafe.naver.com/mhiveacademy
 * https://www.udemy.com/course/stm32-drone-programming/?referralCode=E24CB7B1CD9993855D45
 * https://www.inflearn.com/course/stm32cubelde-stm32f4%EB%93%9C%EB%A1%A0-%EA%B0%9C%EB%B0%9C
*/

#include "PID control.h"

PIDDouble roll;
PIDDouble pitch;

PIDSingle yaw_heading;
PIDSingle yaw_rate;

PIDDouble altitude;

PIDDouble lat;
PIDDouble lon;

#define DT 0.001f
#define OUTER_DERIV_FILT_ENABLE 1
#define INNER_DERIV_FILT_ENABLE 1

void Double_Roll_Pitch_PID_Calculation(PIDDouble* axis, float set_point_angle, float angle/*BNO080 Rotation Angle*/, float rate/*ICM-20602 Angular Rate*/)
{
	/*********** Double PID Outer Begin (Roll and Pitch Angular Position Control) *************/
	axis->out.reference = set_point_angle;	//Set point of outer PID control
	axis->out.meas_value = angle;			//BNO080 rotation angle

	axis->out.error = axis->out.reference - axis->out.meas_value;	//Define error of outer loop
	axis->out.p_result = axis->out.error * axis->out.kp;			//Calculate P result of outer loop

#define OUT_ERR_SUM_MAX 2000
#define OUT_I_ERR_MIN -OUT_ERR_SUM_MAX
	if(axis->out.error_sum > OUT_ERR_SUM_MAX) axis->out.error_sum = OUT_ERR_SUM_MAX;
	else if(axis->out.error_sum < OUT_I_ERR_MIN) axis->out.error_sum = OUT_I_ERR_MIN;
	axis->out.i_result = axis->out.error_sum * axis->out.ki;			//Calculate I result of outer loop

	axis->out.error_deriv = -rate;										//Define derivative of outer loop (rate = ICM-20602 Angular Rate)

#if !OUTER_DERIV_FILT_ENABLE
	axis->out.d_result = axis->out.error_deriv * axis->out.kd;			//Calculate D result of outer loop
#else
	axis->out.error_deriv_filt = axis->out.error_deriv_filt * 0.4f + axis->out.error_deriv * 0.6f;	//filter for derivative
	axis->out.d_result = axis->out.error_deriv_filt * axis->out.kd;									//Calculate D result of inner loop
#endif

	axis->out.pid_result = axis->out.p_result + axis->out.i_result + axis->out.d_result;  //Calculate PID result of outer loop
	/****************************************************************************************/
	
	/************ Double PID Inner Begin (Roll and Pitch Angular Rate Control) **************/
	axis->in.reference = axis->out.pid_result;	//Set point of inner PID control is the PID result of outer loop (for double PID control)
	axis->in.meas_value = rate;					//ICM-20602 angular rate

	axis->in.error = axis->in.reference - axis->in.meas_value;	//Define error of inner loop
	axis->in.p_result = axis->in.error * axis->in.kp;			//Calculate P result of inner loop

	axis->in.error_sum = axis->in.error_sum + axis->in.error * DT;	//Define summation of inner loop
#define IN_ERR_SUM_MAX 500
#define IN_I_ERR_MIN -IN_ERR_SUM_MAX
	if(axis->in.error_sum > IN_ERR_SUM_MAX) axis->in.error_sum = IN_ERR_SUM_MAX;
	else if(axis->in.error_sum < IN_I_ERR_MIN) axis->in.error_sum = IN_I_ERR_MIN;
	axis->in.i_result = axis->in.error_sum * axis->in.ki;							//Calculate I result of inner loop

	axis->in.error_deriv = -(axis->in.meas_value - axis->in.meas_value_prev) / DT;	//Define derivative of inner loop
	axis->in.meas_value_prev = axis->in.meas_value;									//Refresh value_prev to the latest value

#if !INNER_DERIV_FILT_ENABLE
	axis->in.d_result = axis->in.error_deriv * axis->in.kd;				//Calculate D result of inner loop
#else
	axis->in.error_deriv_filt = axis->in.error_deriv_filt * 0.5f + axis->in.error_deriv * 0.5f;	//filter for derivative
	axis->in.d_result = axis->in.error_deriv_filt * axis->in.kd;								//Calculate D result of inner loop
#endif
	
	axis->in.pid_result = axis->in.p_result + axis->in.i_result + axis->in.d_result; //Calculate PID result of inner loop
	/****************************************************************************************/
}

void Single_Yaw_Heading_PID_Calculation(PIDSingle* axis, float set_point_angle, float angle/*BNO080 Rotation Angle*/, float rate/*ICM-20602 Angular Rate*/)
{
	/*********** Single PID Begin (Yaw Angular Position) *************/
	axis->reference = set_point_angle;	//Set point of yaw heading @ yaw stick is center.
	axis->meas_value = angle;			//Current BNO080_Yaw angle @ yaw stick is center.

	axis->error = axis->reference - axis->meas_value;	//Define error of yaw angle control

	if(axis->error > 180.f) axis->error -= 360.f;
	else if(axis->error < -180.f) axis->error += 360.f;
	
	axis->p_result = axis->error * axis->kp;			//Calculate P result of yaw angle control

	axis->error_sum = axis->error_sum + axis->error * DT;	//Define summation of yaw angle control
	axis->i_result = axis->error_sum * axis->ki;			//Calculate I result of yaw angle control

	axis->error_deriv = -rate;						//Define differentiation of yaw angle control
	axis->d_result = axis->error_deriv * axis->kd;	//Calculate D result of yaw angle control
	
	axis->pid_result = axis->p_result + axis->i_result + axis->d_result; //Calculate PID result of yaw angle control
	/***************************************************************/
}

void Single_Yaw_Rate_PID_Calculation(PIDSingle* axis, float set_point_rate, float rate/*ICM-20602 Angular Rate*/)
{
	/*********** Single PID Begin (Yaw Angular Rate Control) *************/
	axis->reference = set_point_rate;	//Set point of yaw heading @ yaw stick is not center.
	axis->meas_value = rate;			//Current ICM20602.gyro_z @ yaw stick is not center.

	axis->error = axis->reference - axis->meas_value;	//Define error of yaw rate control
	axis->p_result = axis->error * axis->kp;			//Calculate P result of yaw rate control

	axis->error_sum = axis->error_sum + axis->error * DT;	//Define summation of yaw rate control
	axis->i_result = axis->error_sum * axis->ki;			//Calculate I result of yaw rate control

	axis->error_deriv = -(axis->meas_value - axis->meas_value_prev) / DT;	//Define differentiation of yaw rate control
	axis->meas_value_prev = axis->meas_value;								//Refresh value_prev to the latest value
	axis->d_result = axis->error_deriv * axis->kd;							//Calculate D result of yaw rate control

	axis->pid_result = axis->p_result + axis->i_result + axis->d_result; //Calculate PID result of yaw control
	/*******************************************************************/
}

void Reset_PID_Integrator(PIDSingle* axis)
{
	axis->error_sum = 0;
}

void Reset_GPS_Integrator(PDSingle_GPS* axis)
{
	axis->total_average = 0;
	axis->rotating_mem_location = 0;
	for(int i; i < sizeof(axis->rotating_mem); i++)
	{
		axis->rotating_mem[i] = 0;
	}
}

void Reset_All_PID_Integrator(void)
{
	Reset_PID_Integrator(&roll.in);
	Reset_PID_Integrator(&roll.out);
	Reset_PID_Integrator(&pitch.in);
	Reset_PID_Integrator(&pitch.out);
	Reset_PID_Integrator(&yaw_heading);
	Reset_PID_Integrator(&yaw_rate);

	Reset_PID_Integrator(&altitude.in);
	Reset_PID_Integrator(&altitude.out);

	Reset_GPS_Integrator(&lat);
	Reset_GPS_Integrator(&lon);
}

void Single_Altitude_PID_Calculation(PIDSingle* axis, float set_point_altitude, float current_altitude)
{
#define I_RESULT_MAX 4000
#define I_RESULT_MIN -I_RESULT_MAX

	axis->reference = set_point_altitude;
	axis->meas_value = current_altitude;

	axis->error = axis->reference - axis->meas_value;
	axis->error_sum = axis->error_sum + axis->error * DT;
	axis->error_deriv = -(axis->meas_value - axis->meas_value_prev) / DT;
	axis->meas_value_prev  = axis->meas_value;
	/*
	axis->kp = 0;
	if(axis->error > 0.05 || axis->error < -0.05)
	{
		if(axis->error > 0) axis->kp = (axis->error - 0.05) * 3000;
		else axis->kp = (-(axis->error) - 0.05) * 3000;

		if(axis->kp > 5000) axis->kp = 5000;
	}*/
	axis->p_result = axis->kp * axis->error;
	axis->i_result = axis->ki * axis->error_sum;
	axis->d_result = axis->kd * axis->error_deriv;

	if(axis->i_result > I_RESULT_MAX) axis->i_result = I_RESULT_MAX;
	else if(axis->i_result < I_RESULT_MIN ) axis->i_result = I_RESULT_MIN;

	axis->pid_result = axis->p_result + axis->i_result + axis->d_result;
	if(axis->pid_result < -3000) axis->pid_result = -3000;
	else if(axis->pid_result > 10000) axis->pid_result = 10000;

}

void Single_Altitude_Rate_PID_Calculation(PIDSingle* axis, float set_point_rate, float current_altitude)
{
	axis->reference = set_point_rate;
	axis->meas_value = current_altitude;

	axis->meas_rate = -(axis->meas_value - axis->meas_value_prev) / DT;
	axis->meas_value_prev = axis->meas_value;

	axis->error = axis->reference - axis->meas_rate;
	axis->p_result = axis->kp * axis->error;

	axis->error_sum = axis->error_sum + axis->error * DT;
	axis->i_result = axis->ki * axis->error_sum;

	axis->error_deriv = -(axis->meas_rate - axis->meas_rate_prev) / DT;
	axis->meas_rate_prev = axis->meas_rate_prev;
	axis->d_result = axis->kd * axis->error_deriv;

	axis->pid_result = axis->p_result + axis->i_result + axis->d_result;
}

void Double_Altitude_PID_Calculation(PIDDouble* axis, float set_point_altitude, float current_altitude)
{
#define ALT_ERR_SUM_MAX 2000
#define ALT_ERR_SUM_MIN -ALT_ERR_SUM_MAX
	/*********** Double PID Outer Begin (Roll and Pitch Angular Position Control) *************/
	axis->out.reference = set_point_altitude;	//Set point of outer PID control
	axis->out.meas_value = current_altitude;		//Actual Altitude from Fusion

	axis->out.error = axis->out.reference - axis->out.meas_value;	//Define error of outer loop
	axis->out.p_result = axis->out.error * axis->out.kp;			//Calculate P result of outer loop

	axis->out.error_deriv = -(axis->out.meas_value - axis->out.meas_value_prev) / DT;
	axis->out.meas_value_prev = axis->out.meas_value;

#if !INNER_DERIV_FILT_ENABLE
	axis->out.d_result = axis->out.error_deriv * axis->out.kd;				//Calculate D result of outer loop
#else
	axis->out.error_deriv_filt = axis->out.error_deriv_filt * 0.8f + axis->out.error_deriv * 0.2f;	//filter for derivative
	axis->out.d_result = axis->out.error_deriv_filt * axis->out.kd;				//Calculate D result of outer loop
#endif

	axis->out.pid_result = axis->out.p_result + axis->out.d_result;
	/****************************************************************************************/

	/************ Double PID Inner Begin (Roll and Pitch Angular Rate Control) **************/
	axis->in.reference = axis->out.pid_result;	//Set point of inner PID control is the PID result of outer loop (for double PID control)
	axis->in.meas_value = -(axis->out.error_deriv_filt);					//ICM-20602 angular rate

	axis->in.error = axis->in.reference - axis->in.meas_value;	//Define error of inner loop
	axis->in.p_result = axis->in.error * axis->in.kp;			//Calculate P result of inner loop

	axis->in.error_sum = axis->in.error_sum + axis->in.error * DT;
	if(axis->in.error_sum > ALT_ERR_SUM_MAX) axis->in.error_sum = ALT_ERR_SUM_MAX;
	else if(axis->in.error_sum < ALT_ERR_SUM_MIN) axis->in.error_sum = ALT_ERR_SUM_MIN;
	axis->in.i_result = axis->in.ki * axis->in.error_sum;

	axis->in.error_deriv = -(axis->in.meas_value - axis->in.meas_value_prev) / DT;	//Define derivative of inner loop
	axis->in.meas_value_prev = axis->in.meas_value;									//Refresh value_prev to the latest value

#if !INNER_DERIV_FILT_ENABLE
	axis->in.d_result = axis->in.error_deriv * axis->in.kd;				//Calculate D result of inner loop
#else
	axis->in.error_deriv_filt = axis->in.error_deriv_filt * 0.4f + axis->in.error_deriv * 0.6f;	//filter for derivative
	axis->in.d_result = axis->in.error_deriv_filt * axis->in.kd;								//Calculate D result of inner loop
#endif

	axis->in.pid_result = axis->in.p_result + axis->in.i_result + axis->in.d_result; //Calculate PID result of inner loop
	/****************************************************************************************/
if (axis->in.pid_result < -4000) axis->in.pid_result = -4000;
if (axis->in.pid_result > 8000) axis->in.pid_result = 8000;

}

void Single_GPS_PD_Calculation(PDSingle_GPS* axis, signed int set_point_gps, signed int gps)
{
	axis->reference = set_point_gps;
	axis->meas_value = gps;

	axis->error = axis->reference - axis->meas_value;                                                         //Calculate the latitude error between waypoint and actual position.

	axis->total_average -=  axis->rotating_mem[axis->rotating_mem_location];                         //Subtract the current memory position to make room for the new value.
	axis->rotating_mem[ axis->rotating_mem_location] = axis->error - axis->error_prev;          //Calculate the new change between the actual pressure and the previous measurement.
	axis->total_average +=  axis->rotating_mem[ axis->rotating_mem_location];                         //Add the new value to the long term avarage value.

	axis->rotating_mem_location++;                                                                        //Increase the rotating memory location.
	if ( axis->rotating_mem_location == 35) axis->rotating_mem_location = 0;                                //Start at 0 when the memory location 35 is reached.

	axis->error_prev = axis->error;                                                             //Remember the error for the next loop.

	//Calculate the GPS pitch and roll correction as if the nose of the multicopter is facing north.
	//The Proportional part = (float)gps_lat_error * gps_p_gain.
	//The Derivative part = (float)gps_lat_total_avarage * gps_d_gain.
	axis->p_result = axis->kp * axis->error;

	axis->d_result = axis->kd * axis->total_average;

	axis->pd_result = axis->p_result + axis->d_result;
}

void Double_GPS_PID_Calculation(PIDDouble* axis, double set_point_gps,  gps)
{
   /*********** Double PID Outer Begin (Roll and Pitch Angular Position Control) *************/
#define GPS_OUTER_DERIV_FILT_ENABLE 1
#define GPS_INNER_DERIV_FILT_ENABLE 1
   axis->out.reference = set_point_gps;   //Set point of outer PID control
   axis->out.meas_value = gps;

   axis->out.error = axis->out.reference - axis->out.meas_value;   //Define error of outer loop
   axis->out.p_result = axis->out.error * axis->out.kp;         //Calculate P result of outer loop

   axis->out.error_sum = axis->out.error_sum + axis->out.error * DT;   //Define summation of outer loop
#define OUT_ERR_SUM_MAX 500
#define OUT_ERR_SUM_MIN -OUT_ERR_SUM_MAX
   if(axis->out.error_sum > OUT_ERR_SUM_MAX) axis->out.error_sum = OUT_ERR_SUM_MAX;
   else if(axis->out.error_sum < OUT_ERR_SUM_MIN) axis->out.error_sum = OUT_ERR_SUM_MIN;
   axis->out.i_result = axis->out.error_sum * axis->out.ki;         //Calculate I result of outer loop

   axis->out.error_deriv = -(axis->out.meas_value - axis->out.meas_value_prev)/DT;//Define derivative of outer loop
   axis->out.meas_value_prev = axis->out.meas_value;

#if !GPS_OUTER_DERIV_FILT_ENABLE
   axis->out.d_result = axis->out.error_deriv * axis->out.kd;         //Calculate D result of outer loop
#else
   axis->out.error_deriv_filt = axis->out.error_deriv_filt * 0.1f + axis->out.error_deriv * 0.9f;   //filter for derivative
   axis->out.d_result = axis->out.error_deriv_filt * axis->out.kd;                           //Calculate D result of inner loop
#endif

   axis->out.pid_result = axis->out.p_result + axis->out.i_result + axis->out.d_result;  //Calculate PID result of outer loop
   /****************************************************************************************/

   /************ Double PID Inner Begin (Roll and Pitch Angular Rate Control) **************/
   axis->in.reference = axis->out.pid_result;   //Set point of inner PID control is the PID result of outer loop (for double PID control)
   axis->in.meas_value = -(axis->out.error_deriv);

   axis->in.error = axis->in.reference - axis->in.meas_value;   //Define error of inner loop
   axis->in.p_result = axis->in.error * axis->in.kp;         //Calculate P result of inner loop

   axis->in.error_sum = axis->in.error_sum + axis->in.error * DT;   //Define summation of inner loop
#define IN_ERR_SUM_MAX 500
#define IN_ERR_SUM_MIN -IN_ERR_SUM_MAX
   if(axis->in.error_sum > IN_ERR_SUM_MAX) axis->in.error_sum = IN_ERR_SUM_MAX;
   else if(axis->in.error_sum < IN_ERR_SUM_MIN) axis->in.error_sum = IN_ERR_SUM_MIN;
   axis->in.i_result = axis->in.error_sum * axis->in.ki;                     //Calculate I result of inner loop

   axis->in.error_deriv = -(axis->in.meas_value - axis->in.meas_value_prev) / DT;   //Define derivative of inner loop
   axis->in.meas_value_prev = axis->in.meas_value;                           //Refresh value_prev to the latest value

#if !GPS_INNER_DERIV_FILT_ENABLE
   axis->in.d_result = axis->in.error_deriv * axis->in.kd;            //Calculate D result of inner loop
#else
   axis->in.error_deriv_filt = axis->in.error_deriv_filt * 0.1f + axis->in.error_deriv * 0.9f;   //filter for derivative
   axis->in.d_result = axis->in.error_deriv_filt * axis->in.kd;                        //Calculate D result of inner loop
#endif

   axis->in.pid_result = axis->in.p_result + axis->in.i_result + axis->in.d_result; //Calculate PID result of inner loop
   /****************************************************************************************/
}
