#ifndef __CHASSIS_TASK_H__
#define __CHASSIS_TASK_H__

#include "main.h"
#include "dm_motor_drv.h"
#include "drv_dji_motor.h"
#include "INS_Task.h"
#include "cmsis_os2.h"
#include "bsp_dwt.h"
#include "VMC.h"
#include "pid.h"
#include "app_preference.h"
#include "Legs.h"
#include "Leg.h"
#include "user_lib.h"
#include "remote_task.h"
#include "bsp_can.h"

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
typedef enum{
	CHASSIS_STOP = 0,
	CHASSIS_RUN,
	CHASSIS_GYROSCOPE,
}chassis_mode_t;
typedef struct{
	fp32 vx;
	fp32 vy;
	fp32 wz;
	fp32 vx_set;
	fp32 vy_set;
	fp32 wz_set;
	
	fp32 MIN_Torque;
	fp32 MAX_Torque;
	
	uint8_t Gear;
	fp32 Speed;
	fp32 Speed_Set;
	fp32 Vx_Set_Last;
	fp32 Speed_Set_Last;
}Chassis_Velocity_t;
typedef struct
{
	bool Spin_Flag;
	bool RC_Flag;
	bool Visual_Flag;
	bool Energy_Flag;
	bool Looding_Flag;
	bool Fric_Flag;
	bool Shoot_Flag;
	bool Shoot_Reversal_Flag;
	bool Speed_Up_Flag;
	bool Velocity_Clac_Flag;
	
	bool Collapse_Flag;
	bool Start_Jump_Flag;
	bool Jump_Flag;
	bool Jump_MAX_Flag;
	bool Jump_shrink_Flag;
	bool Recovery_Leg_Flag;
	bool Model_Flag;
	bool Wheel_Leg_Model_Flag;
	bool Liftoff_Flag[2];//离地标志位
	bool jump_Flag;

} Chassis_Ctrl_Flags_t;
class Chassis_Class{
public:
	/*PID*/
	PidTypeDef Leg_Roll_Pid;
	PidTypeDef Gyro_X_Pid;
	PidTypeDef Leg_Angle0_err_Pid;
	PidTypeDef Leg_L0_Pid[2];
	PidTypeDef Leg_L0_Speed_Pid[2];
	PidTypeDef Stand_Position_Pid[2];
	PidTypeDef Stand_Speed_Pid[2];
	PidTypeDef L0_Speed_Pid[2];

	float x_fdb;
	chassis_mode_t Mode;
	chassis_mode_t Last_Mode;
	Chassis_Ctrl_Flags_t Flags;

	uint32_t Chassis_Task_DWT_Count;
	float Chassis_Task_DWT_dt;
	uint32_t Chassis_DWT_Count;
	float Chassis_DWT_dt;

	Joint_Motor_t Joint_Motor[4];
	RM3508_TypeDef Wheel_Motor[2];
	
	fp32 Chassis_Q;
	fp32 Chassis_R1;
	fp32 Chassis_R0;
	fp32 ROLL_set;
	fp32 Blance_Turn_Kp;
	fp32 ROLL_MAX_set;

	fp32 L0_Leg_KP, L1_Leg_KP;
	fp32 L0_Leg_Speed_KP;
	fp32 Power_Set_KP;
	fp32 Power_Set_err;
	fp32 ramp_period;

	fp32 Init_Lout_M;
	fp32 Wheel_Init_Lout;
	fp32 Init_Lout;

	fp32 LITTLE_TOP_V;

	fp32 MPC_errms, MPC_MS, Last_MPC_MS;

	fp32 V_COLLAPSE;
	fp32 FN_max;

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
	
	Chassis_Velocity_t Velocity;
	
	void Chassis_Init(void);
	void Leg_Init();
	void Behaviour_Mode(void);
	void Feedback_Update(void);
	void Control(void);
	void Control_Loop(void);
	void slope_following(float *target,float *set,float acc);
private:
	void Behaviour_Control(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set);
	void RC_to_Control(fp32 *vx_set, fp32 *vy_set);
	void Key_to_Control(fp32 *vx_set, fp32 *vy_set);
	void Flag_Control();
};

extern Chassis_Class Chassis;
extern fp32 ramp_float( fp32  final, fp32  now, fp32  ramp );

#endif  /*__CHASSIS_TASK_H__*/
