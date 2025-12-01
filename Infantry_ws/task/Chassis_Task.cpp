#include "Chassis_Task.h"
/*
关节电机4310
左can2右can1
左腿2，3（前后）右腿1，0（前后）
0 3
1 2zuo
 头
*/
#define Left_Leg 0
#define Right_Leg 1
#define All 2
float k1 = 80, k2 = 60;

bool need_Stand = true;
bool need_enabled_loop_flag = true;

Chassis_Class Chassis;


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
		Chassis.Decide_Mode();
		Chassis.Feedback_Update();
		Chassis.Chassis_Control();
		Chassis.Chassis_Control_Loop();
		
		if(Chassis.start_flag == 1){
			DM_Class.mit_ctrl(&hfdcan2,Chassis.Joint_Motor[3].para.id,0.0f,0.0f,0.0f,0.0f,leg.vmc[0].torque_set[1]);
			osDelay(CHASSIS_TIME);
			DM_Class.mit_ctrl(&hfdcan2,Chassis.Joint_Motor[2].para.id,0.0f,0.0f,0.0f,0.0f,leg.vmc[0].torque_set[0]);
			osDelay(CHASSIS_TIME);
			DM_Class.mit_ctrl(&hfdcan1,Chassis.Joint_Motor[1].para.id,0.0f,0.0f,0.0f,0.0f,-leg.vmc[1].torque_set[1]);
			osDelay(CHASSIS_TIME);
			DM_Class.mit_ctrl(&hfdcan1,Chassis.Joint_Motor[0].para.id,0.0f,0.0f,0.0f,0.0f,-leg.vmc[1].torque_set[0]);
			osDelay(CHASSIS_TIME);
		}else if(Chassis.start_flag == 0){
			DM_Class.mit_ctrl(&hfdcan2,Chassis.Joint_Motor[3].para.id,0.0f,0.0f,0.0f,0.0f,0.0f);
			osDelay(CHASSIS_TIME);
			DM_Class.mit_ctrl(&hfdcan2,Chassis.Joint_Motor[2].para.id,0.0f,0.0f,0.0f,0.0f,0.0f);
			osDelay(CHASSIS_TIME);
			DM_Class.mit_ctrl(&hfdcan1,Chassis.Joint_Motor[1].para.id,0.0f,0.0f,0.0f,0.0f,0.0f);
			osDelay(CHASSIS_TIME);
			DM_Class.mit_ctrl(&hfdcan1,Chassis.Joint_Motor[0].para.id,0.0f,0.0f,0.0f,0.0f,0.0f);
			osDelay(CHASSIS_TIME);
		}
	}
}

void Chassis_Class::Chassis_Init(void){
	const static float Leg_Pid[3] = {LEG_PID_KP,LEG_PID_KI,LEG_PID_KD};

	DM_Class.Joint_Motor_Init(&Chassis.Joint_Motor[0],6,MIT_MODE);
	DM_Class.Joint_Motor_Init(&Chassis.Joint_Motor[1],8,MIT_MODE);
	
	DM_Class.Joint_Motor_Init(&Chassis.Joint_Motor[2],6,MIT_MODE);
	DM_Class.Joint_Motor_Init(&Chassis.Joint_Motor[3],8,MIT_MODE);
	for(int j=0;j<10;j++){
	  DM_Class.enable_motor_mode(&hfdcan1,Chassis.Joint_Motor[1].para.id,Chassis.Joint_Motor[1].mode);
	  osDelay(3);
	  DM_Class.enable_motor_mode(&hfdcan1,Chassis.Joint_Motor[0].para.id,Chassis.Joint_Motor[0].mode);
	  osDelay(3);	

		DM_Class.enable_motor_mode(&hfdcan2,Chassis.Joint_Motor[3].para.id,Chassis.Joint_Motor[3].mode);
		osDelay(3);
		DM_Class.enable_motor_mode(&hfdcan2,Chassis.Joint_Motor[2].para.id,Chassis.Joint_Motor[2].mode);
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
	Velocity.MAX_Torque = MAX_WHEEL_TORQUE;
	Velocity.MIN_Torque = -MAX_WHEEL_TORQUE;
	Feedback_Update();
}

void Chassis_Class::Feedback_Update(void){
	if(need_enable_loop_flag){
		uint8_t i = 0;
		Chassis_DWT_dt = DWT_GetDeltaT(&Chassis_DWT_Count);
			if(true){

			}
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
	Velocity.vx = (-Wheel_Motor[0].speed + Wheel_Motor[1].speed)/2.0f;
	
};
void Chassis_Class::Decide_Mode(void){
	
}
void Chassis_Class::Leg_Init(void){
	/*kalman filter Q&R */
	Chassis_Q = Q_DATE;
	Chassis_R0 = R0_DATE;
	Chassis_R1 = R1_DATE;
	
	Init_Lout_M  = LEG_GRAVITY;
	Wheel_Init_Lout = 0;
	/*leg zero point*/
	Leg_Ctrl.Leg_set.Tilt_angle_0_set = 0;
	
	L0_Leg_KP = LEG_L0_PID_KP;
	L1_Leg_KP = LEG_L1_PID_KP;
	L0_Leg_Speed_KP = LEG_L0_SPEED_PID_KP;

	ramp_period = 0.01;
	LITTLE_TOP_V = 1.2;
	V_COLLAPSE = 0.5;
	FN_max = 35;
	Power_Set_KP = 0.23;
	
//	Power_Ctrl.power_buffer_set = 30;
//	Flags.Jump_Flag = false;
//	Flags.jump_Flag = false;

	/* 腿的位置 旋转速度 收腿速度*/
	leg.stand.Stand_Speed = 2.0f;//
	leg.stand.L0Speed = -0.4f;
	leg.stand.Stand_Angle = 0.255f;
}

void Chassis_Class::Chassis_Control(void){
	
}

void Chassis_Class::Chassis_Control_Loop(void){
	uint8_t i = 0,j = 0;
	switch(leg.leg_flag.Stand_flag){
		case 1 : 
			if(!(leg.leg_flag.Revolve_flag_L || leg.leg_flag.Revolve_flag_R)){
				leg.leg_flag.Stand_flag = 3;
				osDelay(10);
			}
			if(leg.leg_flag.Revolve_flag_L){
				PID.Calc(&Chassis.Stand_Position_Pid[0], leg.vmc[0].point.phi0, leg.stand.Stand_Angle);
				Limit_min_max(&Chassis.Stand_Position_Pid[0].out,-3.0f,3.0f);
				PID.Calc(&Chassis.Stand_Speed_Pid[0], leg.vmc[0].point.d_phi0, Chassis.Stand_Position_Pid[0].out);
				//PID.Calc(&Chassis.Stand_Speed_Pid[0], leg.vmc[0].point.d_phi0, leg.stand.Stand_Sign[0] * leg.stand.Stand_Speed);
				leg.vmc[0].Tp = Chassis.Stand_Speed_Pid[0].out;
				leg.vmc[0].vmc_forward();
				Limit_min_max(&leg.vmc[0].torque_set[0],-3.0f,3.0f);
				Limit_min_max(&leg.vmc[0].torque_set[1],-3.0f,3.0f);
			}
			if(leg.leg_flag.Revolve_flag_R){
//				PID.Calc(&Chassis.Stand_Position_Pid[1],leg.vmc[1].point.phi0,leg.stand.Stand_Angle);
//				Limit_min_max(&Chassis.Stand_Position_Pid[1].out, -3.0f, 3.0f);
				
				PID.Calc(&Chassis.Stand_Speed_Pid[1], leg.vmc[1].point.d_phi0, leg.stand.Stand_Sign[1] * leg.stand.Stand_Speed);
				leg.vmc[1].Tp = Chassis.Stand_Speed_Pid[1].out;
				leg.vmc[1].vmc_forward();
				Limit_min_max(&leg.vmc[1].torque_set[0], -3.0f, 3.0f);
				Limit_min_max(&leg.vmc[1].torque_set[1], -3.0f, 3.0f);
			}
			/*判断是否完成站立 完成后对左右腿进行数据清零*/
			if(fabs(leg.stand.Stand_Angle - leg.vmc[0].point.phi0) <0.24386f){
				leg.leg_data_clear(Left_Leg);
				PID.Clear(&Stand_Speed_Pid[0]);
				leg.leg_flag.Revolve_flag_L = false;		
			}
			if(fabs(leg.stand.Stand_Angle - leg.vmc[1].point.phi0) <0.24386f){
				leg.leg_data_clear(Right_Leg);
				PID.Clear(&Stand_Speed_Pid[1]);
				leg.leg_flag.Revolve_flag_R = false;				
			}
			break;
		case 3 :
			PID.Calc(&Chassis.L0_Speed_Pid[0], leg.vmc[0].point.d_L0, leg.stand.L0Speed);
			leg.vmc[0].F0 = Chassis.L0_Speed_Pid[0].out;
	
			PID.Calc(&Chassis.L0_Speed_Pid[1], leg.vmc[1].point.d_L0, leg.stand.L0Speed);
			leg.vmc[1].F0 = Chassis.L0_Speed_Pid[1].out;
			
			leg.vmc[0].vmc_forward();
			leg.vmc[1].vmc_forward();
			
			Limit_min_max(&leg.vmc[0].torque_set[0], -40.0f, 40.0f);
			Limit_min_max(&leg.vmc[0].torque_set[1], -40.0f, 40.0f);

			Limit_min_max(&leg.vmc[1].torque_set[0], -40.0f, 40.0f);
			Limit_min_max(&leg.vmc[1].torque_set[1], -40.0f, 40.0f);
			
			if(leg.vmc[0].point.L0 <0.3 && leg.vmc[1].point.L0 <0.3){
				osDelay(50);
				leg.leg_flag.Stand_flag ++;;
			}
			break;
		case 4 :
			leg.leg_set.L0_set = 0.25;
			for (i = 0; i < 2; i++){
				leg.wheel_T[i] = LQR_K[0] * (leg.vmc[i].theta -0.0f) 
											 + LQR_K[1] * (leg.vmc[i].d_theta - 0.0f);

				leg.vmc[i].Tp = LQR_K[6] * (leg.vmc[i].theta -0.0f) 
											+ LQR_K[7] * (leg.vmc[i].d_theta - 0.0f);
				
				PID.Calc(&Chassis.Leg_L0_Pid[i], leg.vmc[i].point.L0, leg.leg_set.L0_set);
				PID.Calc(&Chassis.Leg_L0_Speed_Pid[i], leg.vmc[i].point.d_L0, 0.0f);
				{
					leg.vmc[i].F0 = Leg_L0_Pid[i].out + Leg_L0_Speed_Pid[i].out ;//+ k1 *cos(leg.vmc[i].theta);
					leg.vmc[i].vmc_forward();                                     
				}
			}
			if(fabs(leg.vmc[0].theta) < 0.2 && fabs(leg.vmc[1].theta) < 0.2){
				leg.leg_flag.Stand_flag = 0;
				leg.leg_flag.Blance_flag = true;
			}
			break;
		default : break;
	}
	if(leg.leg_flag.Blance_flag){
		for (i = 0; i < 2; i++){
			leg.wheel_T[i] = LQR_K[0] * (leg.vmc[i].theta - 0.0f) 
										 + LQR_K[1] * (leg.vmc[i].d_theta - 0.0f) 
										 + LQR_K[2] * (x_fdb - 0.0f) 
										 + LQR_K[3] * (Velocity.vx - Leg_Ctrl.Leg_set.Wheel_Speed_set) 
										 + LQR_K[4] * (-INS.Pitch - (-0.0f)) 
										 + LQR_K[5] * (-INS.Gyro[0]- 0.0f);

			leg.vmc[i].Tp = LQR_K[6] * (leg.vmc[i].theta - 0.0f) 
										+ LQR_K[7] * (leg.vmc[i].d_theta - 0.0f) 
										+ LQR_K[8] * (x_fdb - 0.0f) 
										+ LQR_K[9] * (Velocity.vx - Leg_Ctrl.Leg_set.Wheel_Speed_set) 
										+ LQR_K[10] * (-INS.Pitch - -0.0f) 
										+ LQR_K[11] * (-INS.Gyro[0] - 0.0f);

			PID.Calc(&Chassis.Leg_L0_Pid[i], leg.vmc[i].point.L0, leg.leg_set.L0_set);
			PID.Calc(&Chassis.Leg_L0_Speed_Pid[i], leg.vmc[i].point.d_L0, 0.0f);
		}
	PID.Calc(&Chassis.Leg_Angle0_err_Pid, leg.vmc[0].point.phi0 - leg.vmc[1].point.phi0, 0.0f);
	leg.vmc[0].Tp += Leg_Angle0_err_Pid.out;
	leg.vmc[1].Tp -= Leg_Angle0_err_Pid.out;
	
	leg.vmc[0].F0 = Leg_L0_Pid[0].out + Leg_L0_Speed_Pid[0].out + k1 *cos(leg.vmc[0].theta);
	leg.vmc[0].vmc_forward();

	leg.vmc[1].F0 = Leg_L0_Pid[1].out + Leg_L0_Speed_Pid[1].out + k2 *cos(leg.vmc[1].theta);
	leg.vmc[1].vmc_forward();
	}
	Limit_min_max(&leg.vmc[0].torque_set[0], -7.0f, 7.0f);
	Limit_min_max(&leg.vmc[0].torque_set[1], -7.0f, 7.0f);

	Limit_min_max(&leg.vmc[1].torque_set[0], -7.0f, 7.0f);
	Limit_min_max(&leg.vmc[1].torque_set[1], -7.0f, 7.0f);

	Limit_min_max(&leg.wheel_T[0], -3.0f, 3.0f);
	Limit_min_max(&leg.wheel_T[1], -3.0f, 3.0f);
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
