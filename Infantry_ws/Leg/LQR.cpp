#include "LQR.h"
#include "Leg.h"
#include "Mpc.h"
#include "app_preference.h"

LQRClass LQR_Ctrl;

LQRClass::LQRClass(){
	/*设置腿的倾角*/
	Leg_Ctrl.Leg_set.Tilt_angle_0_set=0;
	Leg_Ctrl.Leg_set.Tilt_angle_0_set=0;
	/*板凳模型的参数*/
	Bench_KP.X_KP=0;//2 3 -1.2 -1.2
	Bench_KP.V_KP=0;
	Bench_KP.Yaw_KP=0;
	Bench_KP.Yaw_Gyro_KP=0;
	/*异常倒地腿部倾角增益*/
  Collapse_J_K[0][0]=20; Collapse_J_K[0][1]=6.5;  Collapse_J_K[0][2]=0;  Collapse_J_K[0][3]=0;
	Collapse_J_K[1][0]=0;  Collapse_J_K[1][1]=0;    Collapse_J_K[1][2]=20; Collapse_J_K[1][3]=6.5;

	Fall_KP=720,Fall_Speed_KP=LEG_L0_SPEED_PID_KP;//
	
	Jump_KP=720,Jump_Speed_KP=0;//
  Jump_shou_KP=640,Jump_Speed_shou_KP=0;//
  Jump_err=0.04,Jump_min=0.02;//
  Init_Lset=0.065,Middle_Lset=0.10,MAX_Lset=0.20;//
  L0_set_max=0.20,L0_set_min=0.065;//
}
float K_W[2][10]={
-0.6634,-2.1199,-1.3371,-0.9411,-9.7245,-2.1620,-4.0539,-0.5256,-18.3809,-1.3821,
-0.6634,-2.1199,+1.3371,+0.9411,-4.0539,-0.5256,-9.7245,-2.1620,-18.3809,-1.3821,
};
float K_J[2][10]={
+1.4126,+4.8533,-4.8723,-3.5265,+64.5621,+15.0901,-8.0275,-0.9139,-89.2034,-5.4780,
+1.4126,+4.8533,+4.8723,+3.5265,-8.0275,-0.9139,+64.5621,+15.0901,-89.2034,-5.4780,
};

void LQRClass::LQR_Update(float L_leg,float R_leg){
	for(uint8_t i=0;i<=9;i++){
		 LQR_Wheel[0][ i]= K_W[0][i];
		 LQR_Wheel[1][ i]= K_W[1][i];
		 LQR_Joint[0][ i]= K_J[0][i];
	   LQR_Joint[1][ i]= K_J[1][i];
	}
}

void LQRClass::Kgain_Init(uint8_t i){
	LQR_out[i].Wheel_out[0]=  +Wheel_Kgain1[i][0]*(                                     0 - Leg_state.Distance    );   
	LQR_out[i].Wheel_out[1]=	+Wheel_Kgain1[i][1]*( Leg_Ctrl.Leg_set.Wheel_Speed_set[0]   - Leg_state.Distance_dot);
	LQR_out[i].Wheel_out[2]=	+Wheel_Kgain1[i][2]*( Leg_Ctrl.Leg_set.yaw_set              - Leg_state.phi         );
	LQR_out[i].Wheel_out[3]=	+Wheel_Kgain1[i][3]*( Leg_Ctrl.Leg_set.yaw_Gyro_set         - Leg_state.phi_dot     );
	LQR_out[i].Wheel_out[4]=	+Wheel_Kgain1[i][4]*( Leg_Ctrl.Leg_set.Tilt_angle_0_set     - Leg_state.theta_ll    );
	LQR_out[i].Wheel_out[5]=	+Wheel_Kgain1[i][5]*( 0                            					- Leg_state.theta_ll_dot);
	LQR_out[i].Wheel_out[6]=	+Wheel_Kgain1[i][6]*( Leg_Ctrl.Leg_set.Tilt_angle_0_set     - Leg_state.theta_lr    );
	LQR_out[i].Wheel_out[7]=	+Wheel_Kgain1[i][7]*( 0                            				  - Leg_state.theta_lr_dot);
	LQR_out[i].Wheel_out[8]=	+Wheel_Kgain1[i][8]*( 0                            				  - Leg_state.theta       );
	LQR_out[i].Wheel_out[9]=	+Wheel_Kgain1[i][9]*( 0                            				  - Leg_state.theta_dot   );

	LQR_out[i].Joint_out[0]=  +Joint_Kgain1[i][0]*(                                     0 - Leg_state.Distance     );
	LQR_out[i].Joint_out[1]=  +Joint_Kgain1[i][1]*( Leg_Ctrl.Leg_set.Wheel_Speed_set[1]   - Leg_state.Distance_dot );
	LQR_out[i].Joint_out[2]=  +Joint_Kgain1[i][2]*( Leg_Ctrl.Leg_set.yaw_set              - Leg_state.phi          );
	LQR_out[i].Joint_out[3]=  +Joint_Kgain1[i][3]*( Leg_Ctrl.Leg_set.yaw_Gyro_set         - Leg_state.phi_dot      );
	LQR_out[i].Joint_out[4]=  +Joint_Kgain1[i][4]*( Leg_Ctrl.Leg_set.Tilt_angle_0_set     - Leg_state.theta_ll     );
	LQR_out[i].Joint_out[5]=  +Joint_Kgain1[i][5]*( 0                             				- Leg_state.theta_ll_dot );
	LQR_out[i].Joint_out[6]=  +Joint_Kgain1[i][6]*( Leg_Ctrl.Leg_set.Tilt_angle_0_set     - Leg_state.theta_lr     );
	LQR_out[i].Joint_out[7]=  +Joint_Kgain1[i][7]*( 0                             				- Leg_state.theta_lr_dot );
	LQR_out[i].Joint_out[8]=  +Joint_Kgain1[i][8]*( 0                            					- Leg_state.theta        );
	LQR_out[i].Joint_out[9]=  +Joint_Kgain1[i][9]*( 0                             				- Leg_state.theta_dot    );

	LQR_out[i].Adate_Wheel_out   =K_adapt[i] * (MPC_Ctrl.MPC_forecast_erfa_dot[i]-Leg_state.erfa_dot[i]);
}

void LQRClass::LQR_Calc(uint8_t  i,Blance_out* Leg_out){
	float Wheel_torque_out=0,
	Tp_out=0;
	LQR_Ctrl.Kgain_Init(i);
	for(uint8_t j=0;j<=9;j++){
		 Wheel_torque_out    +=LQR_out[i].Wheel_out[j];	
		 Tp_out              +=LQR_out[i].Joint_out[j];
	}
	LQR_out[i].Wheel_torque_out    =Wheel_torque_out;
	LQR_out[i].Tp_out              =Tp_out;
	//	LQR_out[i].Wheel_torque_out    =Wheel_torque_out+LQR_out[i].Adate_Wheel_out ;
	//  LQR_out[i].Tp_out              =Tp_out; 
	Leg_out->Tp_out          =LQR_out[i].Tp_out; 
	Leg_out->Wheel_torque_out=LQR_out[i].Wheel_torque_out;
}
