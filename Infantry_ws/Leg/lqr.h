#ifndef __LQR_H
#define __LQR_H

#include "arm_math.h"

#include "Leg.h"

#ifdef __cplusplus
extern "C" {
#endif

#include "bsp_dwt.h"
	
#ifdef __cplusplus
}
#endif

typedef struct
{	
	float Distance;
	float Distance_dot;
	float phi;
	float phi_dot;
	float theta_ll;
	float theta_ll_dot;	
	float theta_lr;
	float theta_lr_dot;
	float theta;
	float theta_dot;	
	float erfa_dot[2];
} Blance_state;

typedef struct
{	
	float X_KP;
	float V_KP;
	float Yaw_KP;
	float Yaw_Gyro_KP;
} Bench_model;

typedef struct
{
    float F_Init_out;
    float F_out;
	  float Tp_out;
	  float TP_1_out,TP_2_out;
	  float Wheel_torque_out;
	  float Wheel_torque_MPC_out,Tp_MPC_out;
	  float Wheel_out[10]    , Joint_out[10];
	  float Wheel_MPC_out[10], Joint_MPC_out[10];
    float Adate_Wheel_out;
} LQR_Out;

class LQRClass
{
	private:
		
	public:
	
	float LQR_Joint[2][10],LQR_Wheel[2][10];
	float Wheel_Kgain1[2][10],Joint_Kgain1[2][10];
	float K_adapt[2];
	float Collapse_J_K[2][4];
	float Fall_KP,Fall_Speed_KP;
	float Jump_KP,Jump_Speed_KP;
  float Jump_shou_KP,Jump_Speed_shou_KP;
	float Jump_err,Jump_min;
  float Inti_Lset,Middle_Lset,MAX_Lset;
  float L0_set_max,L0_set_min;
	
	Blance_state   Leg_state;
	Bench_model    Bench_KP;
	LQR_Out        LQR_out[2];
	
	void  LQR_Update(float L_leg,float R_leg);
	void  Kgain_Init(uint8_t i);
  void  LQR_Calc(uint8_t i,Blance_out* Leg_out);
  float LQR_polyfit(float L_leg,float R_leg);

  LQRClass();
};

extern LQRClass LQR_Ctrl;

#endif
