#ifndef __VMC_H
#define __VMC_H

#include "main.h"
#include "arm_math.h"
#include "bsp_dwt.h"

#define pi 3.1415926f
#define LEG_PID_KP  350.0f
#define LEG_PID_KI  0.0f
#define LEG_PID_KD  3000.0f
#define LEG_PID_MAX_OUT  90.0f //90ţ
#define LEG_PID_MAX_IOUT 0.0f
#define LEG_PID_MAX_I 0.0f

typedef struct
{	
	float	L1;
	float L2;
	float L3;
	float L4;
	float L5;
}connection_rod_t;


/* 
	A-B-C-D-E-A
	L1 L2 L3 L4 L5
										
		E____A					
		/		 \					| L0
  D/			\B        | 
	 \			/         | phi0
    \ 	 /          |
     \	/           |
		  \/C           |
 */
typedef struct
{	
	float xB,yB;
	float xD,yD;
	
	float xC,yC;
	float L0,phi0;
	
	float d_phi0;
	float d_L0;
	float dd_L0;
	float last_L0;
	float last_d_L0;

	float alpha;
	float d_alpha;
	
	float last_phi0;

	float phi4,phi1;
	float phi3,phi2;
	
	float d_L0_now;

}coordinate_point_t;

typedef struct 
{
	float j11,j12,j21,j22;
	
}jacobian_t;


typedef struct  
{
	float Tp_1_fdb,Tp_2_fdb;
	float F0_fdb,Tp_fdb;    
	float FN_fdb;  
}F0_Tp_feedback_t;

class VMC_Class
{
public:

	connection_rod_t five_link;
	
	coordinate_point_t point;
	
	jacobian_t jacobi;

	float torque_set[2];

	float F0;
	float Tp;

  F0_Tp_feedback_t F_fdb;
	
	float theta;
	float d_theta;
	float last_d_theta;
	float dd_theta;
	
	float FN;

	void kinematics_forward();
	void kinematics_reverse();
	
	void vmc_calc(float pitch ,float pitch_Gyro ,float dt ,float Joint_motor_front_vel /*Ç°µç»úËÙ¶È*/ ,float Joint_motor_rear_vel /*ºóµç»úËÙ¶È*/);
  void vmc_forward();
  void vmc_reverse();
	float LQR_K_Calc(float *coe,float len);
	
	uint8_t leg_flag;
};

typedef struct{
	float Stand_Speed;//起立旋转速度
	float L0Speed;		//起立收腿速度
	float Stand_Angle;//起身关节角度
	/*
	起立旋转方向，面向车头，从左顺为-1正为1
	*/
	int8_t Stand_Sign[2];
}Stand_t;
typedef struct{
	float L0_set_max;
	float L0_set_middle;
	float L0_set_min;
	float L0_set;
	float L0_set_final;
}leg_set_t;
typedef struct
{
	bool Blance_flag;
	bool Revolve_flag_L;
	bool Revolve_flag_R;
	uint8_t Stand_flag;
}leg_flag_t;

class leg_class
{
public:
	Stand_t stand;
	float stand_delay_time;
	float wheel_T[2];
	leg_flag_t leg_flag;
	VMC_Class vmc[2];
	leg_set_t leg_set;
	void leg_data_clear(uint8_t count);
	leg_class();
};
extern leg_class leg;

void Limit_min_max(float *in,float min,float max);

extern VMC_Class VMC;
#endif
