#include "LQR.h"
#include "Leg.h"
#include "Mpc.h"
#include "app_preference.h"

LQRClass LQR_Ctrl;

LQRClass::LQRClass()
{
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
  Inti_Lset=0.13,Middle_Lset=0.17,MAX_Lset=0.38;//
  L0_set_max=0.38,L0_set_min=0.12;//
}
float K_W[2][10]={
-0.6634,-2.1199,-1.3371,-0.9411,-9.7245,-2.1620,-4.0539,-0.5256,-18.3809,-1.3821,
-0.6634,-2.1199,+1.3371,+0.9411,-4.0539,-0.5256,-9.7245,-2.1620,-18.3809,-1.3821,
};
float K_J[2][10]={
+1.4126,+4.8533,-4.8723,-3.5265,+64.5621,+15.0901,-8.0275,-0.9139,-89.2034,-5.4780,
+1.4126,+4.8533,+4.8723,+3.5265,-8.0275,-0.9139,+64.5621,+15.0901,-89.2034,-5.4780,
};



void LQRClass::LQR_Update(float L_leg,float R_leg)
{
	for(uint8_t i=0;i<=9;i++){
		 LQR_Wheel[0][ i]= K_W[0][i];
		 LQR_Wheel[1][ i]= K_W[1][i];
		 LQR_Joint[0][ i]= K_J[0][i];
	   LQR_Joint[1][ i]= K_J[1][i];
	}
	
//	LQR_Wheel[0][0]=4.2488*L_leg*L_leg + -1.3543*R_leg*R_leg + -2.5890*L_leg*R_leg + -2.6407*L_leg + 2.4269*R_leg + -1.0837;
//	LQR_Wheel[0][1]=9.2192*L_leg*L_leg + -6.5005*R_leg*R_leg + -7.7972*L_leg*R_leg + -4.0937*L_leg + 7.6592*R_leg + -3.1841;
//	LQR_Wheel[0][2]=-9.2831*L_leg*L_leg + 0.5202*R_leg*R_leg + 1.2855*L_leg*R_leg + 4.3627*L_leg + -1.3110*R_leg + -1.4538;
//	LQR_Wheel[0][3]=-41.6315*L_leg*L_leg + 2.3069*R_leg*R_leg + 5.7797*L_leg*R_leg + 19.6054*L_leg + -5.8877*R_leg + -6.5187;
//	LQR_Wheel[0][4]=54.9270*L_leg*L_leg + 5.4173*R_leg*R_leg + -11.4935*L_leg*R_leg + -53.5677*L_leg + 9.3053*R_leg + -4.6276;
//	LQR_Wheel[0][5]=12.5095*L_leg*L_leg + 1.5867*R_leg*R_leg + -1.6043*L_leg*R_leg + -9.1631*L_leg + 1.1419*R_leg + -2.0388;
//	LQR_Wheel[0][6]=-20.8693*L_leg*L_leg + 16.2984*R_leg*R_leg + 12.5382*L_leg*R_leg + 7.6936*L_leg + -17.7049*R_leg + -2.4059;
//	LQR_Wheel[0][7]=-7.1180*L_leg*L_leg + 1.8065*R_leg*R_leg + 1.4102*L_leg*R_leg + 1.9381*L_leg + -3.1408*R_leg + -0.3066;
//	LQR_Wheel[0][8]=-23.5882*L_leg*L_leg + -17.4953*R_leg*R_leg + -11.9912*L_leg*R_leg + 46.8239*L_leg + 25.5644*R_leg + -26.0635;
//	LQR_Wheel[0][9]=-1.2680*L_leg*L_leg + -2.2812*R_leg*R_leg + -1.7221*L_leg*R_leg + 3.9608*L_leg + 2.9678*R_leg + -2.6281;
//	
//	LQR_Wheel[0][0]=-3.1981*L_leg*L_leg + -0.2821*R_leg*R_leg + 3.8244*L_leg*R_leg + 2.4714*L_leg + -2.6862*R_leg + -1.0854;
//	LQR_Wheel[1][1]=-10.6681*L_leg*L_leg + -3.4563*R_leg*R_leg + 9.0115*L_leg*R_leg + 8.7901*L_leg + -5.1457*R_leg + -3.1995;
//	LQR_Wheel[1][2]=-1.5853*L_leg*L_leg + 4.1037*R_leg*R_leg + 4.6227*L_leg*R_leg + 1.6542*L_leg + -4.5044*R_leg + 1.4284;
//	LQR_Wheel[1][3]=-7.0865*L_leg*L_leg + 18.3669*R_leg*R_leg + 20.7564*L_leg*R_leg + 7.4364*L_leg + -20.2519*R_leg + 6.4053;
//	LQR_Wheel[1][4]=2.5952*L_leg*L_leg + 11.0609*R_leg*R_leg + -3.5803*L_leg*R_leg + -18.3066*L_leg + 7.4061*R_leg + -2.3230;
//	LQR_Wheel[1][5]=-0.2222*L_leg*L_leg + -2.5026*R_leg*R_leg + -0.8781*L_leg*R_leg + -2.8444*L_leg + 1.4986*R_leg + -0.2916;
//	LQR_Wheel[1][6]=-5.5195*L_leg*L_leg + 11.0238*R_leg*R_leg + 41.7210*L_leg*R_leg + 7.7608*L_leg + -51.4247*R_leg + -4.6716;
//	LQR_Wheel[1][7]=-0.9252*L_leg*L_leg + 0.9935*R_leg*R_leg + 11.4185*L_leg*R_leg + 2.5747*L_leg + -10.1317*R_leg + -2.0869;
//	LQR_Wheel[1][8]=6.5464*L_leg*L_leg + -52.6366*R_leg*R_leg + -14.7028*L_leg*R_leg + 15.5077*L_leg + 60.6349*R_leg + -26.4833;
//	LQR_Wheel[1][9]=0.2034*L_leg*L_leg + -5.1718*R_leg*R_leg + -0.9441*L_leg*R_leg + 1.8811*L_leg + 5.3600*R_leg + -2.6630;
//	
//	LQR_Joint[0][0]=1.3986*L_leg*L_leg + -4.4357*R_leg*R_leg + -0.4128*L_leg*R_leg + -0.1005*L_leg + -0.1782*R_leg + 0.3633;
//	LQR_Joint[0][1]=4.8135*L_leg*L_leg + -8.3707*R_leg*R_leg + 0.7630*L_leg*R_leg + -1.9446*L_leg + -1.9643*R_leg + 1.4654;
//	LQR_Joint[0][2]=15.6886*L_leg*L_leg + -1.2369*R_leg*R_leg + 2.6740*L_leg*R_leg + -4.8652*L_leg + -2.2072*R_leg + -1.4544;
//	LQR_Joint[0][3]=70.2315*L_leg*L_leg + -5.8138*R_leg*R_leg + 11.9775*L_leg*R_leg + -21.7415*L_leg + -9.9052*R_leg + -6.5545;
//	LQR_Joint[0][4]=-1.0866*L_leg*L_leg + 71.4121*R_leg*R_leg + -7.6316*L_leg*R_leg + 25.0594*L_leg + -0.4371*R_leg + 14.6364;
//	LQR_Joint[0][5]=27.1122*L_leg*L_leg + 4.8204*R_leg*R_leg + -2.0406*L_leg*R_leg + -18.9494*L_leg + 0.8900*R_leg + 6.6214;
//	LQR_Joint[0][6]=47.8920*L_leg*L_leg + -152.3551*R_leg*R_leg + 9.8690*L_leg*R_leg + -16.5204*L_leg + -13.3628*R_leg + 0.0930;
//	LQR_Joint[0][7]=15.3108*L_leg*L_leg + -33.5169*R_leg*R_leg + 0.6307*L_leg*R_leg + -2.9799*L_leg + 0.9734*R_leg + -0.1413;
//	LQR_Joint[0][8]=101.7985*L_leg*L_leg + 57.2176*R_leg*R_leg + -18.9407*L_leg*R_leg + -142.6241*L_leg + 36.7700*R_leg + -34.6809;
//	LQR_Joint[0][9]=8.5365*L_leg*L_leg + 5.7733*R_leg*R_leg + -2.2261*L_leg*R_leg + -13.4093*L_leg + 3.8320*R_leg + -3.3055;
//	
//	LQR_Joint[1][0]=-7.1368*L_leg*L_leg + 8.0181*R_leg*R_leg + -4.4519*L_leg*R_leg + 0.9208*L_leg + -1.0591*R_leg + 0.3400;
//	LQR_Joint[1][1]=-15.5816*L_leg*L_leg + 21.9749*R_leg*R_leg + -9.7292*L_leg*R_leg + 0.5963*L_leg + -3.9958*R_leg + 1.3844;
//	LQR_Joint[1][2]=-5.5119*L_leg*L_leg + -1.3462*R_leg*R_leg + -8.1880*L_leg*R_leg + 1.3062*L_leg + 4.6651*R_leg + 1.5856;
//	LQR_Joint[1][3]=-24.4837*L_leg*L_leg + -5.7233*R_leg*R_leg + -36.8690*L_leg*R_leg + 5.7009*L_leg + 21.0121*R_leg + 7.1422;
//	LQR_Joint[1][4]=-69.5315*L_leg*L_leg + -66.0779*R_leg*R_leg + 31.5193*L_leg*R_leg + -3.2189*L_leg + -21.4849*R_leg + -0.5279;
//	LQR_Joint[1][5]=-24.8890*L_leg*L_leg + -0.5778*R_leg*R_leg + 3.7111*L_leg*R_leg + 3.8642*L_leg + -3.5331*R_leg + -0.4306;
//	LQR_Joint[1][6]=3.1352*L_leg*L_leg  + 117.2397*R_leg*R_leg + -56.0946*L_leg*R_leg + 3.3771*L_leg + 20.8076*R_leg + 14.6518;
//	LQR_Joint[1][7]=-2.8188*L_leg*L_leg +  22.7219*R_leg*R_leg + 9.5498*L_leg*R_leg + 1.7319*L_leg + -19.5370*R_leg + 6.5911;
//	LQR_Joint[1][8]=24.3178*L_leg*L_leg + -74.3964*R_leg*R_leg + 207.0409*L_leg*R_leg + 56.5325*L_leg + -172.2328*R_leg + -33.4201;
//	LQR_Joint[1][9]=0.9531*L_leg*L_leg  + -5.4023*R_leg*R_leg + 17.8244*L_leg*R_leg + 5.8479*L_leg + -16.1829*R_leg + -3.2081;


}

void LQRClass::Kgain_Init(uint8_t i)
{
		LQR_out[i].Wheel_out[0]=  +Wheel_Kgain1[i][0]*(                                     0 - Leg_state.Distance    );   
		LQR_out[i].Wheel_out[1]=	+Wheel_Kgain1[i][1]*( Leg_Ctrl.Leg_set.Wheel_Speed_set      - Leg_state.Distance_dot);
		LQR_out[i].Wheel_out[2]=	+Wheel_Kgain1[i][2]*( Leg_Ctrl.Leg_set.yaw_set              - Leg_state.phi         );
		LQR_out[i].Wheel_out[3]=	+Wheel_Kgain1[i][3]*( Leg_Ctrl.Leg_set.yaw_Gyro_set         - Leg_state.phi_dot     );
		LQR_out[i].Wheel_out[4]=	+Wheel_Kgain1[i][4]*( Leg_Ctrl.Leg_set.Tilt_angle_0_set     - Leg_state.theta_ll    );
		LQR_out[i].Wheel_out[5]=	+Wheel_Kgain1[i][5]*( 0                            					- Leg_state.theta_ll_dot);
		LQR_out[i].Wheel_out[6]=	+Wheel_Kgain1[i][6]*( Leg_Ctrl.Leg_set.Tilt_angle_0_set     - Leg_state.theta_lr    );
		LQR_out[i].Wheel_out[7]=	+Wheel_Kgain1[i][7]*( 0                            				  - Leg_state.theta_lr_dot);
		LQR_out[i].Wheel_out[8]=	+Wheel_Kgain1[i][8]*( 0                            				  - Leg_state.theta       );
		LQR_out[i].Wheel_out[9]=	+Wheel_Kgain1[i][9]*( 0                            				  - Leg_state.theta_dot   );

		LQR_out[i].Joint_out[0]=  +Joint_Kgain1[i][0]*(                                     0 - Leg_state.Distance     );
		LQR_out[i].Joint_out[1]=  +Joint_Kgain1[i][1]*( Leg_Ctrl.Leg_set.Wheel_Speed_set      - Leg_state.Distance_dot );
		LQR_out[i].Joint_out[2]=  +Joint_Kgain1[i][2]*( Leg_Ctrl.Leg_set.yaw_set              - Leg_state.phi          );
		LQR_out[i].Joint_out[3]=  +Joint_Kgain1[i][3]*( Leg_Ctrl.Leg_set.yaw_Gyro_set         - Leg_state.phi_dot      );
		LQR_out[i].Joint_out[4]=  +Joint_Kgain1[i][4]*( Leg_Ctrl.Leg_set.Tilt_angle_0_set     - Leg_state.theta_ll     );
		LQR_out[i].Joint_out[5]=  +Joint_Kgain1[i][5]*( 0                             				- Leg_state.theta_ll_dot );
		LQR_out[i].Joint_out[6]=  +Joint_Kgain1[i][6]*( Leg_Ctrl.Leg_set.Tilt_angle_0_set     - Leg_state.theta_lr     );
		LQR_out[i].Joint_out[7]=  +Joint_Kgain1[i][7]*( 0                             				- Leg_state.theta_lr_dot );
		LQR_out[i].Joint_out[8]=  +Joint_Kgain1[i][8]*( 0                            					- Leg_state.theta        );
		LQR_out[i].Joint_out[9]=  +Joint_Kgain1[i][9]*( 0                             				- Leg_state.theta_dot    );
	
//	LQR_out[i].Wheel_out[0]=  +Wheel_Kgain1[i][0]*( Leg_Ctrl.Leg_set.Carload_Distance_set - MPC_Ctrl.MPC_forecast_X[0] );   
//	LQR_out[i].Wheel_out[1]=	+Wheel_Kgain1[i][1]*( Leg_Ctrl.Leg_set.Wheel_Speed_set      - MPC_Ctrl.MPC_forecast_X[1] );
//	LQR_out[i].Wheel_out[2]=	+Wheel_Kgain1[i][2]*( Leg_Ctrl.Leg_set.yaw_set              - MPC_Ctrl.MPC_forecast_X[2] );
//	LQR_out[i].Wheel_out[3]=	+Wheel_Kgain1[i][3]*( Leg_Ctrl.Leg_set.yaw_Gyro_set         - MPC_Ctrl.MPC_forecast_X[3] );
//	LQR_out[i].Wheel_out[4]=	+Wheel_Kgain1[i][4]*( Leg_Ctrl.Leg_set.Tilt_angle_0_set     - MPC_Ctrl.MPC_forecast_X[4] );
//	LQR_out[i].Wheel_out[5]=	+Wheel_Kgain1[i][5]*( 0                            					- MPC_Ctrl.MPC_forecast_X[5] );
//	LQR_out[i].Wheel_out[6]=	+Wheel_Kgain1[i][6]*( Leg_Ctrl.Leg_set.Tilt_angle_0_set     - MPC_Ctrl.MPC_forecast_X[6] );
//  LQR_out[i].Wheel_out[7]=	+Wheel_Kgain1[i][7]*( 0                            				  - MPC_Ctrl.MPC_forecast_X[7] );
//	LQR_out[i].Wheel_out[8]=	+Wheel_Kgain1[i][8]*( 0                            				  - MPC_Ctrl.MPC_forecast_X[8] );
//	LQR_out[i].Wheel_out[9]=	+Wheel_Kgain1[i][9]*( 0                            				  - MPC_Ctrl.MPC_forecast_X[9] );

//	LQR_out[i].Joint_out[0]=  +Joint_Kgain1[i][0]*( Leg_Ctrl.Leg_set.Carload_Distance_set - MPC_Ctrl.MPC_forecast_X[0] );
//	LQR_out[i].Joint_out[1]=  +Joint_Kgain1[i][1]*( Leg_Ctrl.Leg_set.Wheel_Speed_set      - MPC_Ctrl.MPC_forecast_X[1] );
//	LQR_out[i].Joint_out[2]=  +Joint_Kgain1[i][2]*( Leg_Ctrl.Leg_set.yaw_set              - MPC_Ctrl.MPC_forecast_X[2] );
//	LQR_out[i].Joint_out[3]=  +Joint_Kgain1[i][3]*( Leg_Ctrl.Leg_set.yaw_Gyro_set         - MPC_Ctrl.MPC_forecast_X[3] );
//	LQR_out[i].Joint_out[4]=  +Joint_Kgain1[i][4]*( Leg_Ctrl.Leg_set.Tilt_angle_0_set     - MPC_Ctrl.MPC_forecast_X[4] );
//	LQR_out[i].Joint_out[5]=  +Joint_Kgain1[i][5]*( 0                             				- MPC_Ctrl.MPC_forecast_X[5] );
//	LQR_out[i].Joint_out[6]=  +Joint_Kgain1[i][6]*( Leg_Ctrl.Leg_set.Tilt_angle_0_set     - MPC_Ctrl.MPC_forecast_X[6] );
//	LQR_out[i].Joint_out[7]=  +Joint_Kgain1[i][7]*( 0                             				- MPC_Ctrl.MPC_forecast_X[7] );
//	LQR_out[i].Joint_out[8]=  +Joint_Kgain1[i][8]*( 0                            					- MPC_Ctrl.MPC_forecast_X[8] );
//	LQR_out[i].Joint_out[9]=  +Joint_Kgain1[i][9]*( 0                             				- MPC_Ctrl.MPC_forecast_X[9] );
	
	  LQR_out[i].Adate_Wheel_out   =K_adapt[i] * (MPC_Ctrl.MPC_forecast_erfa_dot[i]-Leg_state.erfa_dot[i]);
}

void LQRClass::LQR_Calc(uint8_t  i,Blance_out* Leg_out)
{
	float Wheel_torque_out=0,
	      Tp_out=0;

     LQR_Ctrl.Kgain_Init(i);
		
   for(uint8_t j=0;j<=9;j++)
	 {
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
