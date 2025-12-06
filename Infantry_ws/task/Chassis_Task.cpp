#include "Chassis_Task.h"
/*
关节电机4310
左can2右can1
左腿2，3（前后）右腿1，0（前后）
2	tou 1
0			1
2			3
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
float LQR_K[12]={
-4.4237,  -0.77482,  -0.98832,  -2.103,  5.2872,  0.74637,  
5.4468,  2.2148,  0.76191,  1.9448,  13.3541,  1.7109};

extern "C" void Chassis_Task(){
	while(INS.ins_flag == 0){
		osDelay(1);
	}
	Chassis.Chassis_Init();
	for(;;){
		Chassis.Behaviour_Mode();
		Chassis.Feedback_Update();
		Chassis.Control();
		Chassis.Control_Loop();
		
		if(Chassis.Mode == CHASSIS_STOP){
			DM_Class.mit_ctrl(&hfdcan2,Chassis.Joint_Motor[0].para.id,0.0f,0.0f,0.0f,0.0f,0.0f);
			osDelay(CHASSIS_TIME);
			DM_Class.mit_ctrl(&hfdcan2,Chassis.Joint_Motor[2].para.id,0.0f,0.0f,0.0f,0.0f,0.0f);
			osDelay(CHASSIS_TIME);
			RM_Class.rm3508_ctrl(&hfdcan2,Chassis.Wheel_Motor[0].tx_id,0);
			osDelay(CHASSIS_TIME);
			DM_Class.mit_ctrl(&hfdcan1,Chassis.Joint_Motor[1].para.id,0.0f,0.0f,0.0f,0.0f,0.0f);
			osDelay(CHASSIS_TIME);
			DM_Class.mit_ctrl(&hfdcan1,Chassis.Joint_Motor[3].para.id,0.0f,0.0f,0.0f,0.0f,0.0f);
			osDelay(CHASSIS_TIME);
			RM_Class.rm3508_ctrl(&hfdcan1,Chassis.Wheel_Motor[1].tx_id,0);
			osDelay(CHASSIS_TIME);
		}else{
			DM_Class.mit_ctrl(&hfdcan2,Chassis.Joint_Motor[0].para.id,0.0f,0.0f,0.0f,0.0f,leg.vmc[0].torque_set[0]);
			osDelay(CHASSIS_TIME);
			DM_Class.mit_ctrl(&hfdcan2,Chassis.Joint_Motor[2].para.id,0.0f,0.0f,0.0f,0.0f,leg.vmc[0].torque_set[1]);
			osDelay(CHASSIS_TIME);
			RM_Class.rm3508_ctrl(&hfdcan2,Chassis.Wheel_Motor[0].tx_id,&leg.wheel_T[0]);
			osDelay(CHASSIS_TIME);
			DM_Class.mit_ctrl(&hfdcan1,Chassis.Joint_Motor[1].para.id,0.0f,0.0f,0.0f,0.0f,-leg.vmc[1].torque_set[0]);
			osDelay(CHASSIS_TIME);
			DM_Class.mit_ctrl(&hfdcan1,Chassis.Joint_Motor[3].para.id,0.0f,0.0f,0.0f,0.0f,-leg.vmc[1].torque_set[1]);
			osDelay(CHASSIS_TIME);
			RM_Class.rm3508_ctrl(&hfdcan2,Chassis.Wheel_Motor[1].tx_id,&leg.wheel_T[1]);
			osDelay(CHASSIS_TIME);
		}
	}
}

void Chassis_Class::Chassis_Init(void){
	Mode = CHASSIS_STOP;
	
	Chassis_Task_DWT_dt = 0;
	Chassis_Task_DWT_Count = 0;
	
	Leg_Init();

{PID.Init(&Leg_Angle0_err_Pid, POSITION,
			 LEG_ANGLE0_ERR_PID_KP,
			 LEG_ANGLE0_ERR_PID_KI,
			 LEG_ANGLE0_ERR_PID_KD,
			 LEG_ANGLE0_ERR_PID_MAX_OUT,
			 LEG_ANGLE0_ERR_PID_MAX_IOUT,
			 LEG_ANGLE0_ERR_PID_BAND_I);
	PID.Init(&Stand_Position_Pid[0],
			 POSITION,
			 LEG_Position_PID_KP,
			 LEG_Position_PID_KI,
			 LEG_Position_PID_KD,
			 LEG_Position_PID_MAX_OUT,
			 LEG_Position_PID_MAX_IOUT,
			 LEG_Position_PID_BAND_I);
	PID.Init(&Stand_Position_Pid[1],
			 POSITION,
			 LEG_Position_PID_KP,
			 LEG_Position_PID_KI,
			 LEG_Position_PID_KD,
			 LEG_Position_PID_MAX_OUT,
			 LEG_Position_PID_MAX_IOUT,
			 LEG_Position_PID_BAND_I);
	PID.Init(&Stand_Speed_Pid[0],
			 POSITION,
			 LEG_Speed_PID_KP,
			 LEG_Speed_PID_KI,
			 LEG_Speed_PID_KD,
			 LEG_Speed_PID_MAX_OUT,
			 LEG_Speed_PID_MAX_IOUT,
			 LEG_Speed_PID_BAND_I);
	PID.Init(&Stand_Speed_Pid[1],
			 POSITION,
			 LEG_Speed_PID_KP,
			 LEG_Speed_PID_KI,
			 LEG_Speed_PID_KD,
			 LEG_Speed_PID_MAX_OUT,
			 LEG_Speed_PID_MAX_IOUT,
			 LEG_Speed_PID_BAND_I);

	PID.Init(&L0_Speed_Pid[0],
			 POSITION,
			 LEG_L0Speed_PID_KP,
			 LEG_L0Speed_PID_KI,
			 LEG_L0Speed_PID_KD,
			 LEG_L0Speed_PID_MAX_OUT,
			 LEG_L0Speed_PID_MAX_IOUT,
			 LEG_L0Speed_PID_BAND_I);
	PID.Init(&L0_Speed_Pid[1],
			 POSITION,
			 LEG_L0Speed_PID_KP,
			 LEG_L0Speed_PID_KI,
			 LEG_L0Speed_PID_KD,
			 LEG_L0Speed_PID_MAX_OUT,
			 LEG_L0Speed_PID_MAX_IOUT,
			 LEG_L0Speed_PID_BAND_I);		
	}
	Velocity.MAX_Torque = MAX_WHEEL_TORQUE;
	Velocity.MIN_Torque = -MAX_WHEEL_TORQUE;
	Feedback_Update();
	DM_Class.Joint_Motor_Init(&Chassis.Joint_Motor[0],6,MIT_MODE);
	DM_Class.Joint_Motor_Init(&Chassis.Joint_Motor[2],8,MIT_MODE);
	
	DM_Class.Joint_Motor_Init(&Chassis.Joint_Motor[1],6,MIT_MODE);
	DM_Class.Joint_Motor_Init(&Chassis.Joint_Motor[3],8,MIT_MODE);
	for(int j=0;j<10;j++){
		/*left leg 0 2*/
		DM_Class.enable_motor_mode(&hfdcan2,Chassis.Joint_Motor[0].para.id,Chassis.Joint_Motor[0].mode);
		osDelay(3);
		DM_Class.enable_motor_mode(&hfdcan2,Chassis.Joint_Motor[2].para.id,Chassis.Joint_Motor[2].mode);
		osDelay(3);
		
	  DM_Class.enable_motor_mode(&hfdcan1,Chassis.Joint_Motor[1].para.id,Chassis.Joint_Motor[1].mode);
	  osDelay(3);
	  DM_Class.enable_motor_mode(&hfdcan1,Chassis.Joint_Motor[3].para.id,Chassis.Joint_Motor[3].mode);
	  osDelay(3);	
	}
}
void Chassis_Class::Feedback_Update(void){
	if(need_enable_loop_flag){
		uint8_t i = 0;
		Chassis_DWT_dt = DWT_GetDeltaT(&Chassis_DWT_Count);
			if(dog.Remote_Dog.State == Device_Online){
				
			}
	}
	for (uint8_t i = 0; i < 2; i++){
		leg.vmc[i].point.phi1 = PI / 2.0f + pow(-1.0, i) * Joint_Motor[0 + i].para.POS;
		leg.vmc[i].point.phi4 = PI / 2.0f + pow(-1.0, i) * Joint_Motor[2 + i].para.POS;

		leg.vmc[i].F_fdb.Tp_1_fdb = Joint_Motor[0 + i].para.Torque;
		leg.vmc[i].F_fdb.Tp_2_fdb = Joint_Motor[2 + i].para.Torque;

		leg.vmc[i].vmc_calc(-INS.Pitch, 
							INS.Gyro[0], 
							1.0f / 1000.0f, 
							pow(-1.0, i) * Joint_Motor[0 + i].para.VEL, 
							pow(-1.0, i) * Joint_Motor[2 + i].para.VEL);

		Wheel_Motor[i].speed = (Wheel_Motor[i].speed * WHEEL_RADIUS);
	}
	Velocity.vx = (-Wheel_Motor[0].speed + Wheel_Motor[1].speed)/2.0f;
	x_fdb = x_fdb + Velocity.vx * (1.0 / 1000.0);	
};
void Chassis_Class::RC_to_Control(fp32 *vx_set,fp32 *vy_set){
	static int16_t vx_channel, vy_channel;
	static fp32 vx_set_channel[2],vy_set_channel,temp_set_channel;
	if(dog.Remote_Dog.State == Device_Online){
		vx_set_channel[0] = ramp_float(vx_channel * CHASSIS_VX_RC_SEN,vx_set_channel[0],Vx_Set_Frame_Period);
		vx_set_channel[1] = ramp_float(vx_channel * CHASSIS_VX_RC_SEN,vx_set_channel[1],Vx_Set_Frame_Period);
		for(uint8_t u = 0;u < 2;u++){
			if(vx_channel < CHASSIS_RC_DEADLINE && vx_channel > -CHASSIS_RC_DEADLINE){
				vx_set_channel[u] = 0.0f;
			}
			if(vy_channel < CHASSIS_RC_DEADLINE && vy_channel > -CHASSIS_RC_DEADLINE){
				vy_set_channel = 0.0f;
			}
			vx_set[u] = float_constrain(vx_set_channel[u], -2.5, 2.5);
		}
		*vy_set = vy_set_channel;
	}
}
void Chassis_Class::Key_to_Control(fp32 *vx_set,fp32 *vy_set){
}
void Chassis_Class::Behaviour_Mode(void){
	if(dog.Remote_Dog.State == Device_Online){
		if(remote.RemoteMode == REMOTE_INPUT){
			switch(remote.rc.s[0]){
				case 1 : 
						Mode = CHASSIS_RUN;
					break;
				case 3:
					break;
				case 2:
						Mode = CHASSIS_STOP;
					break;
			}
		}
	Flag_Control();
	}
}
void Chassis_Class::Behaviour_Control(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set){
	static fp32 vw_set, Last_angle_set = 0;
	if (Mode == CHASSIS_STOP){
		vx_set[0] = 0.0f;
		vx_set[1] = 0.0f;
		*vy_set = 0.0f;
		*angle_set = 0.0f;
	}
	else if (Mode == CHASSIS_RUN){
		RC_to_Control(vx_set, vy_set);
		*angle_set = *vy_set;
	}
}

void Chassis_Class::Leg_Init(void){
	/*kalman filter Q&R */
	Chassis_Q = Q_DATE;
	Chassis_R0 = R0_DATE;
	Chassis_R1 = R1_DATE;
	
	Init_Lout_M  = LEG_GRAVITY;
	Wheel_Init_Lout = 0.189;
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
	leg.stand.Stand_Speed = 1.0f;//
	leg.stand.L0Speed = -0.4f;
	leg.stand.Stand_Angle = 1.95f;
}
void Chassis_Class::Flag_Control(){
	if (Mode == CHASSIS_STOP){
		leg.leg_data_clear(All);
		Chassis.Velocity.vx = 0;
		Chassis.x_fdb = 0;

		leg.leg_flag.Blance_flag = false;
		leg.leg_flag.Stand_flag = false;

		Flags.Liftoff_Flag[0] = false;
		Flags.Liftoff_Flag[1] = false;
		Flags.Recovery_Leg_Flag = false;

		PID.Clear(&Chassis.Leg_Angle0_err_Pid);
		PID.Clear(&Gyro_X_Pid);
		PID.Clear(&Leg_Roll_Pid);
		for (uint8_t i = 0; i < 2; i++){
			PID.Clear(&Leg_L0_Pid[i]);
			PID.Clear(&Stand_Position_Pid[i]);
			PID.Clear(&Stand_Speed_Pid[i]);
			PID.Clear(&L0_Speed_Pid[i]);		
		}
	}
	if(!leg.leg_flag.Blance_flag){
		leg.wheel_T[0] = 0;
		leg.wheel_T[1] = 0;
		x_fdb = 0;
	}
	if (need_Stand){
		if (Last_Mode == CHASSIS_STOP && Mode != Last_Mode){
			leg.leg_flag.Stand_flag = 1;
			leg.leg_flag.Revolve_flag_L = true;
			leg.leg_flag.Revolve_flag_R = true;
			leg.leg_flag.Blance_flag = false;
			leg.stand.Stand_Sign[0] = 1;//sign((Stand_Angle - leg.vmc[0].point.phi0));
			leg.stand.Stand_Sign[1] = 1;//sign((Stand_Angle - leg.vmc[1].point.phi0));
		}
	}
	if (Mode == Last_Mode){
		return;
	}else{
		Last_Mode = Mode;
	}
}
void Chassis_Class::Control(void){
	fp32 vx_set[2] = {0}, vy_set = 0.0f, angle_set = 0.0f;
	fp32 sin_yaw = 0.0f, cos_yaw = 0.0f;
	Behaviour_Control(vx_set, &vy_set, &angle_set);
	if(Mode == CHASSIS_RUN){
		leg.leg_set.L0_set_final = leg.leg_set.L0_set_middle + remote.Key_ch[3] * L0_SET_RC_SEN;
		leg.leg_set.L0_set = ramp_float(leg.leg_set.L0_set_final, leg.leg_set.L0_set, 0.00065);
		Limit_min_max(&leg.leg_set.L0_set, leg.leg_set.L0_set_min, leg.leg_set.L0_set_max);

		Leg_Ctrl.Leg_set.Wheel_Speed_set = vx_set[0];
		Leg_Ctrl.Leg_set.Wheel_Speed_set = vx_set[1];
		Leg_Ctrl.Leg_set.Wheel_Collapse_V_set = Leg_Ctrl.Leg_set.Wheel_Collapse_V_set = 0;
		Leg_Ctrl.Leg_set.yaw_Gyro_set = angle_set;
		Leg_Ctrl.Leg_set.yaw_Gyro_set = angle_set;
	}
	if (Mode == CHASSIS_STOP){
		Leg_Ctrl.Leg_set.Wheel_Speed_set = Leg_Ctrl.Leg_set.Wheel_Speed_set = 0;
		Leg_Ctrl.Leg_set.Wheel_Collapse_V_set = Leg_Ctrl.Leg_set.Wheel_Collapse_V_set = 0;
		Leg_Ctrl.Leg_set.yaw_Gyro_set = Leg_Ctrl.Leg_set.yaw_Gyro_set = 0;
	}
}
/*控制计算*/
void Chassis_Class::Control_Loop(void){
	uint8_t i = 0,j = 0;
	switch(leg.leg_flag.Stand_flag){
		case 1 : 
			if(!(leg.leg_flag.Revolve_flag_L || leg.leg_flag.Revolve_flag_R)){
				leg.leg_flag.Stand_flag = 3;
				osDelay(10);
			}
			if(leg.leg_flag.Revolve_flag_L){
				PID.Calc(&Chassis.Stand_Position_Pid[0], leg.vmc[0].point.phi0, leg.stand.Stand_Angle);
				Limit_min_max(&Chassis.Stand_Position_Pid[0].out,-2.0f,2.0f);
				PID.Calc(&Chassis.Stand_Speed_Pid[0], leg.vmc[0].point.d_phi0, Chassis.Stand_Position_Pid[0].out);
//				PID.Calc(&Chassis.Stand_Speed_Pid[0], leg.vmc[0].point.d_phi0, leg.stand.Stand_Sign[0] * leg.stand.Stand_Speed);
				leg.vmc[0].Tp = Chassis.Stand_Speed_Pid[0].out;
				leg.vmc[0].vmc_forward();
				Limit_min_max(&leg.vmc[0].torque_set[0],-3.0f,3.0f);
				Limit_min_max(&leg.vmc[0].torque_set[1],-3.0f,3.0f);
			}
			if(leg.leg_flag.Revolve_flag_R){
				PID.Calc(&Chassis.Stand_Position_Pid[1],leg.vmc[1].point.phi0,-leg.stand.Stand_Angle);
				Limit_min_max(&Chassis.Stand_Position_Pid[1].out, -2.0f, 2.0f);
				PID.Calc(&Chassis.Stand_Speed_Pid[1], leg.vmc[1].point.d_phi0, Chassis.Stand_Position_Pid[1].out);				
//				PID.Calc(&Chassis.Stand_Speed_Pid[1], leg.vmc[1].point.d_phi0, leg.stand.Stand_Sign[1] * leg.stand.Stand_Speed);
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
			
			Limit_min_max(&leg.vmc[0].torque_set[0], -5.0f, 5.0f);
			Limit_min_max(&leg.vmc[0].torque_set[1], -5.0f, 5.0f);

			Limit_min_max(&leg.vmc[1].torque_set[0], -5.0f, 5.0f);
			Limit_min_max(&leg.vmc[1].torque_set[1], -5.0f, 5.0f);
			
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
fp32 ramp_float(fp32 final, fp32 now, fp32 ramp){
	fp32 buffer = 0;
	buffer = final - now;

	if (buffer > 0){
		if (buffer > ramp){
			now += ramp;
		}else{
			now += buffer;
		}
	}else{
		if (buffer < -ramp){
			now += -ramp;
		}else{
			now += buffer;
		}
	}
	return now;
}
