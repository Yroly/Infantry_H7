#include "Chassis_Task.h"
/*
关节电机4310
左腿1，3（前后）右腿2，4（前后）
3 4
1 2
 头
*/
Chassis_Class Chassis;
DM_Motor_Class DM_Class;
VMC_Class VMC;

bool need_stand = true;
bool need_enable_loop_flag = true;

PidTypeDef Leg[2];
uint32_t CHASSIS_TIME = 1;
float Poly_Coefficient[12][4]={	{-88.3079710751263,	68.9068310796955,	-30.0003802287502,	-0.197774178106864},
																{1.52414598059982	,-1.09343038036609,	-2.82688593867512,	0.0281973842051861},
																{-21.8700750609220	,12.7421672466682,	-2.58779676995074	,-0.750848242540331},
																{-29.3271263750692,	17.6067629457167,	-4.23484645974363	,-1.08976980288501},
																{-147.771748892911,	94.0665615939814,	-22.5139626085997	,2.53224765312440},
																{-6.72857056332562,	4.46216499907277,	-1.14328671767927	,0.176775242328476},
																{-43.1495035855057,	35.1427890165576,	-12.7617044245710	,3.36940801739176},
																{4.14428184617563,	-2.56933858132474,	0.479050092243477	,0.248175261724735},
																{-229.898177881547	,144.949258291255	,-33.9196587052128,	3.44291788865558},
																{-329.509693153293,	207.219295206736,	-48.3799707459102	,4.952560575479143},
																{380.589246401548,	-223.660017597103	,46.1696952431268	,9.82308882692083},
																{26.1010681824798	,-15.7241310513153	,3.39175554658673	,0.278568898146322}};
float LQR_K[12]={
   -2.1954,   -0.2044  , -0.8826,   -1.3245,    1.2784  ,  0.1112,
    2.5538,   0.2718  ,  1.5728  ,  2.2893  , 12.1973 ,   0.4578};

extern "C" void Chassis_Task(){
	while(INS.ins_flag == 0){
		osDelay(1);
	}
	Chassis.Chassis_Init();
	for(;;){
		
		Chassis.Feedback_Update();
		Chassis.Chassis_Control_Loop();
		
		if(Chassis.start_flag == 1){
			DM_Class.mit_ctrl(&hfdcan2,Chassis.Joint_Motor[3].para.id,0.0f,0.0f,0.0f,0.0f,VMC.torque_set[1]);
			osDelay(CHASSIS_TIME);
			DM_Class.mit_ctrl(&hfdcan2,Chassis.Joint_Motor[2].para.id,0.0f,0.0f,0.0f,0.0f,VMC.torque_set[0]);
			osDelay(CHASSIS_TIME);
			DM_Class.mit_ctrl2(&hfdcan2,Chassis.Wheel_Motor[1].para.id,0.0f,0.0f,0.0f,0.0f,leg.wheel_T[1]);
			osDelay(CHASSIS_TIME);
			DM_Class.mit_ctrl(&hfdcan1,Chassis.Joint_Motor[1].para.id,0.0f,0.0f,0.0f,0.0f,VMC.torque_set[1]);
			osDelay(CHASSIS_TIME);
			DM_Class.mit_ctrl(&hfdcan1,Chassis.Joint_Motor[0].para.id,0.0f,0.0f,0.0f,0.0f,VMC.torque_set[0]);
			osDelay(CHASSIS_TIME);
			DM_Class.mit_ctrl2(&hfdcan1,Chassis.Wheel_Motor[0].para.id,0.0f,0.0f,0.0f,0.0f,leg.wheel_T[0]);
			osDelay(CHASSIS_TIME);
		}else if(Chassis.start_flag == 0){
			DM_Class.mit_ctrl(&hfdcan2,Chassis.Joint_Motor[3].para.id,0.0f,0.0f,0.0f,0.0f,0.0f);
			osDelay(CHASSIS_TIME);
			DM_Class.mit_ctrl(&hfdcan2,Chassis.Joint_Motor[2].para.id,0.0f,0.0f,0.0f,0.0f,0.0f);
			osDelay(CHASSIS_TIME);
			DM_Class.mit_ctrl2(&hfdcan2,Chassis.Wheel_Motor[1].para.id,0.0f,0.0f,0.0f,0.0f,0.0f);
			osDelay(CHASSIS_TIME);
			DM_Class.mit_ctrl(&hfdcan1,Chassis.Joint_Motor[1].para.id,0.0f,0.0f,0.0f,0.0f,0.0f);
			osDelay(CHASSIS_TIME);
			DM_Class.mit_ctrl(&hfdcan1,Chassis.Joint_Motor[0].para.id,0.0f,0.0f,0.0f,0.0f,0.0f);
			osDelay(CHASSIS_TIME);
			DM_Class.mit_ctrl2(&hfdcan1,Chassis.Wheel_Motor[0].para.id,0.0f,0.0f,0.0f,0.0f,0.0f);
			osDelay(CHASSIS_TIME);
		}
	}
}

void Chassis_Class::Chassis_Init(void){
	const static float Leg_Pid[3] = {LEG_PID_KP,LEG_PID_KI,LEG_PID_KD};

	DM_Class.Joint_Motor_Init(&Chassis.Joint_Motor[0],6,MIT_MODE);
	DM_Class.Joint_Motor_Init(&Chassis.Joint_Motor[1],8,MIT_MODE);
	DM_Class.Wheel_Motor_Init(&Chassis.Wheel_Motor[0],1,MIT_MODE);
	
	DM_Class.Joint_Motor_Init(&Chassis.Joint_Motor[2],6,MIT_MODE);
	DM_Class.Joint_Motor_Init(&Chassis.Joint_Motor[3],8,MIT_MODE);
	DM_Class.Wheel_Motor_Init(&Chassis.Wheel_Motor[1],1,MIT_MODE);
	for(int j=0;j<10;j++){
	  DM_Class.enable_motor_mode(&hfdcan1,Chassis.Joint_Motor[1].para.id,Chassis.Joint_Motor[1].mode);
	  osDelay(3);
	  DM_Class.enable_motor_mode(&hfdcan1,Chassis.Joint_Motor[0].para.id,Chassis.Joint_Motor[0].mode);
	  osDelay(3);	
    DM_Class.enable_motor_mode(&hfdcan1,Chassis.Wheel_Motor[0].para.id,Chassis.Wheel_Motor[0].mode);// 右边轮毂电机
	  osDelay(3);
	  DM_Class.enable_motor_mode(&hfdcan2,Chassis.Joint_Motor[3].para.id,Chassis.Joint_Motor[3].mode);
	  osDelay(3);
		DM_Class.enable_motor_mode(&hfdcan2,Chassis.Joint_Motor[2].para.id,Chassis.Joint_Motor[2].mode);
	  osDelay(3);
    DM_Class.enable_motor_mode(&hfdcan2,Chassis.Wheel_Motor[1].para.id,Chassis.Wheel_Motor[1].mode);//左边轮毂电机
	  osDelay(3);	
	}

	
	Leg_Init();

	{		PID.Init(&Leg[0],
		POSITION,
		Leg_Pid,
		LEG_PID_MAX_OUT,
		LEG_PID_MAX_IOUT,
		LEG_PID_MAX_I);
			PID.Init(&Leg[1],
		POSITION,
		Leg_Pid,
		LEG_PID_MAX_OUT,
		LEG_PID_MAX_IOUT,
		LEG_PID_MAX_I);
	}
}

void Chassis_Class::Feedback_Update(void){
	if(need_enable_loop_flag){
		uint8_t i = 0;
		Chassis_DWT_dt = DWT_GetDeltaT(&Chassis_DWT_Count);
	}	

	for (uint8_t i = 0; i < 2; i++)
	{
		leg.vmc[i].point.phi1 = PI / 2.0f + pow(-1.0, i) * Joint_Motor[0 + i].para.POS.fdata;
		leg.vmc[i].point.phi4 = PI / 2.0f + pow(-1.0, i) * Joint_Motor[2 + i].para.POS.fdata;

		leg.vmc[i].F_fdb.Tp_1_fdb = Joint_Motor[0 + i].para.Torque.fdata;
		leg.vmc[i].F_fdb.Tp_2_fdb = Joint_Motor[2 + i].para.Torque.fdata;

		leg.vmc[i].vmc_calc(-INS.Pitch, 
							INS.Gyro[0], 
							3.0f / 1000.0f, 
							pow(-1.0, i) * Joint_Motor[0 + i].para.VEL.fdata, 
							pow(-1.0, i) * Joint_Motor[2 + i].para.VEL.fdata);

		Wheel_Motor[i].speed = (Wheel_Motor[i].para.VEL.fdata * WHEEL_RADIUS);
	}

};

void Chassis_Class::Leg_Init(void){
	
}

void Chassis_Class::Chassis_Control(void){
	
}

void Chassis_Class::Chassis_Control_Loop(void){
	uint8_t i = 0,j = 0;
	
	for(int i=0;i<12;i++)
	{
		LQR_K[i]=VMC.LQR_K_Calc(&Poly_Coefficient[i][0],VMC.point.L0 );	
	}
	for(i = 0; i < 2; i++){
	leg.wheel_T[i] = (LQR_K[0]*(VMC.theta-0.0f)
									+ LQR_K[1]*(VMC.d_theta-0.0f)
									+ LQR_K[2]*(x_set-Chassis.x_filter)
									+ LQR_K[3]*(Chassis.v_set-Chassis.v_filter)
									+ LQR_K[4]*(-INS.Pitch-0.0f)
									+ LQR_K[5]*(-INS.Gyro[0]-0.0f));
			
	leg.vmc[i].Tp = (LQR_K[6]*(VMC.theta-0.0f)
								 + LQR_K[7]*(VMC.d_theta-0.0f)
								 + LQR_K[8]*(Chassis.x_set-Chassis.x_filter)
								 + LQR_K[9]*(Chassis.v_set-Chassis.v_filter)
								 + LQR_K[10]*(-INS.Pitch-0.0f)
								 + LQR_K[11]*(-INS.Gyro[0]-0.0f));
	mySaturate(&leg.wheel_T[i],-3.0f,3.0f);
	}

}

void Chassis_Class::mySaturate(float *in,float min,float max){
  if(*in < min)
  {
    *in = min;
  }
  else if(*in > max)
  {
    *in = max;
  }
}
void Chassis_Class::slope_following(float *target,float *set,float acc){
	if(*target > *set)
	{
		*set = *set + acc;
		if(*set >= *target)
		*set = *target;
	}
	else if(*target < *set)
	{
		*set = *set - acc;
		if(*set <= *target)
		*set = *target;
	}
}
