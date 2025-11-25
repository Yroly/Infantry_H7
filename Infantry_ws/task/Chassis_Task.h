#ifndef __CHASSIS_TASK_H__
#define __CHASSIS_TASK_H__

#include "main.h"
#include "dm_motor_drv.h"
#include "INS_Task.h"
#include "cmsis_os2.h"
#include "bsp_dwt.h"
#include "VMC.h"
#include "Leg.h"
#include "pid.h"
#include "app_preference.h"

#define TP_PID_KP 10.0f
#define TP_PID_KI 0.0f 
#define TP_PID_KD 0.1f
#define TP_PID_MAX_OUT  2.0f
#define TP_PID_MAX_IOUT 0.0f

#define TURN_PID_KP 2.0f
#define TURN_PID_KI 0.0f 
#define TURN_PID_KD 0.2f
#define TURN_PID_MAX_OUT  1.0f//轮毂电机的额定扭矩
#define TURN_PID_MAX_IOUT 0.0f

#define ROLL_PID_KP 100.0f
#define ROLL_PID_KI 0.0f 
#define ROLL_PID_KD 0.0f
#define ROLL_PID_MAX_OUT  100.0f//轮毂电机的额定扭矩
#define ROLL_PID_MAX_IOUT 0.0f

class Chassis_Class{
public:
	float Chassis_DWT_dt;
	uint32_t Chassis_DWT_Count;
	Joint_Motor_t Joint_Motor[4];
	Wheel_Motor_t Wheel_Motor[2];
	
	fp32 Chassis_Q;
	fp32 Chassis_R1;
	fp32 Chassis_R0;

	float v_set;
	float v_target;
	float x_set;
	float turn_set;
	float target_turn;
	float leg_set;
	float leg_lx_set;
	float target_leg_lx_set;
	float leg_left_set;
	float leg_right_set;
	float last_leg_set;
	float last_leg_left_set;
	float last_leg_right_set;
	float roll_set;
	float roll_target;
	float now_roll_set;

	float v_filter;
	float x_filter;
	
	float myPithR;
	float myPithGyroR;
	float myPithL;
	float myPithGyroL;
	float roll;
	float total_yaw;
	float theta_err;
		
	float turn_T;
	float leg_tp;
	
	uint8_t start_flag;
	
	uint8_t recover_flag;
	
	uint32_t count_key;
	uint8_t jump_flag;
	float jump_leg;
	uint32_t jump_time_r;
	uint32_t jump_time_l;
	uint8_t jump_status_r;
	uint8_t jump_status_l;
	
	void Chassis_Init(void);
	void Leg_Init();
	void Feedback_Update(void);
	void Chassis_Control(void);
	void Chassis_Control_Loop(void);
	void mySaturate(float *in,float min,float max);
	void slope_following(float *target,float *set,float acc);
};

extern Chassis_Class Chassis;

#endif  /*__CHASSIS_TASK_H__*/
