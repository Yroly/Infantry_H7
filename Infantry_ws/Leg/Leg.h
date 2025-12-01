#pragma once

#include "arm_math.h"
#include "bsp_dwt.h"
#include "algorithm_kalman.h"

/*
目标值：L0_set,v_set，x_set
反馈值：pitch，angle1,angle4,L0和angle0(这两者由前三者正解得到)，roll,yaw，x,v,gyro
K增益
*/
#define LEG_GRAVITY        17.1/2
#define LEG_GRAVITY_WHELL  -2.054

#define PITCH_OFET 0
#define ROLL_ZERO  0.00

#define LEG_ZERO_1 -0
#define LEG_ZERO_2 -0
#define LEG_ZERO_3  0
#define LEG_ZERO_4  0
#define L0_MAX      0.39
#define L0_MIN      0.12

#define Q_DATE       0.1
#define R0_DATE      0.001
#define R1_DATE      0.08

#define MPC_KW      0.0
#define MPC_KJ      0.4

#define G_DATE      0.006
#define H_DATE      0.05
#define STAN_TP_KI  0.1

typedef struct DATA
{
    /* data */
    float now;
    float last;	
    float dot_now;   // 微分
	  float dot_last;  // 微分
    float ddot;      // 二阶微分
	
	  float dot_now2;  // 微分
	  float dot_last2; // 微分
    float ddot2;     // 二阶微分
} DataStructure;


typedef struct  VMC
{
	float TP_1_fdb,TP_2_fdb;
	float F_fdb,Tp_fdb;    // 
	float FN_fdb; // 
	
	DataStructure angle0;              //正解算出来的摆角 
	DataStructure Tilt_angle_0;        //正解算出来的摆角转化为轮腿状态下的摆角  状态变量5,6,7,8
	DataStructure L0_fdb;              //正解算出来的摆长L0
	
}VmcStructure;

typedef struct
{
	float Yaw_fdb,Yaw_Gyro_fdb,Last_Yaw_Gyro_fdb;      //状态变量3以及4
	float Pitch_fdb,Pitch_Gyro_fdb,Last_Pitch_Gyro_fdb;//状态变量9以及10
	float Accel_Z;
	float F_inertial;//侧向惯性力矩补偿前馈

} Blance_fdb;

typedef struct
{
    //期望值
    float L0_set;                //L0期望值
	  float L0_MAX_set;                //L0期望值
	  float Tilt_angle_0_set;      //坐标转换下的L0期望值
		float Distance_set,Wheel_Speed_set,Wheel_Collapse_V_set;//,速度，位移期望值
	  float Carload_Distance_set;
		float yaw_set;						    //偏航角期望值
		float yaw_Gyro_set;					  //偏航角期望值
} Blance_set;
typedef struct
{
	mat xhat, xhatminus, z, A,B, H, AT, HT, Q, R, P, Pminus, K,W_aaccel;
	
	float raw_value;
	float filtered_value[2];
	float W_AAccel_date;
	float xhat_data[2], xhatminus_data[2], z_data[2], Pminus_data[4], K_data[4];
	float P_data[4];
	float AT_data[4], HT_data[4];
	float A_data[4];
	float B_data[4];
	float H_data[4];
	float Q_data[4];
	float R_data[4];
	float Q_melody;
	float R0_melody;
	float R1_melody;
}Blance_KF;


typedef struct
{
    //输出值，其中F和Tp通过VMC虚拟为关节电机输出
    float F_Init_out;  // 初值为上层机构重力
    float F_out;       // 根据腿长PD控制得到， 初值为上层机构重力
	  float Tp_out;      // 根据状态反馈矩阵得到五连杆转矩

	  float Wheel_torque_out; // 根据状态反馈矩阵得到轮子转矩
	  float TP_1_out;
    float TP_2_out;

} Blance_out;

class VmcClass
{
public:
		
	uint32_t Leg_DWT_Count;
	float Leg_dt;
	uint32_t KF_DWT_Count;
  float KF_dt;

	VmcStructure   WBR;
  Blance_out 	   Leg_out;
  Blance_KF  	   Leg_KF;

	float Distance_Observe_fdb,Wheel_Speed_fdb,Last_Wheel_Speed_fdb;//状态变量1以及2
	float Carload_Distance_fdb,Carload_Distance_fdb_halt;
	float Wheel_Speed_Forecast,Last_Wheel_Speed_Forecast;
	float Wheel_Accel_Forecast,Last_Wheel_Accel_Forecast;
	float Wheel_Distance_Estimate,Wheel_Distance_Estimate_halt,Wheel_Speed_Estimate,Wheel_Accel_Estimate;
	float Leg_Accl_Y,Accl_Wheel;

	float l1,l2,l3,l4,l5;                //五连杆长,单位是m
	float angle1, angle2, angle3, angle4;//弧度制,14为关节反馈角度,23为中间计算过度角度
	float angle1_dot,angle4_dot;         //五连杆末端坐标
	//坐标(五连杆坐标系下的，原点在五连杆的中垂线上)
	float xa, ya;
	float xb, yb;
	float xc, yc;       //五连杆末端坐标
	float xc_dot,yc_dot;//五连杆末端坐标
	float xd, yd;
	
	void K_Matching();
	void Zjie();
	void Njie(const float xc, const float yc);
	void VMC_Positive();
	void VMC_Negative();
	void VMC_Positive_Kinematics();
	void Support_Force_resolving();
	void Absolute_Speed_KF_Forecast();
	void Absolute_Speed_KF_Init();
	void KF_Feedback_Update();
};
/*---------------------------------------------------------------*/
class LegClass
{
private:
   
public:
    float Mw_Wheel,Ml_leg;


    float Kn; //腿部质心位置系数
		
    VmcClass       Vmc_Ctrl[2];
    Blance_set 	   Leg_set;
    Blance_fdb	   Leg_fdb;


		float Rl,FN[2];            //双驱动轮轮距
    float K_filter_gyro; //Pitch_gyro_低通滤波
	
		void MPC_Forecast_Init(uint8_t i);
		void MPC_U_i_Feedback(uint8_t i);
		void MPC_Forecast_resolving(uint8_t i);
		
    LegClass();

};

extern LegClass Leg_Ctrl;
