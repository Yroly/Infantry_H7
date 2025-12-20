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
float k1 = 8, k2 = 6;//80 60 

bool need_Stand = true;
Chassis_Class Chassis;
bool need_enable_loop_flag = true;

PidTypeDef Leg[2];
uint32_t CHASSIS_TIME = 1;
float LQR_K[12]={
-6.3252,  -0.966,  -0.89306,  -1.2718,  2.2247,  0.35337,  
6.0998,  0.6507,  -0.15627,  -0.23126,  15.6904,  2.1794};
//float LQR_K[12] = {
//-6.8647,  -1.4669,  -0.89374,  -1.5038,  2.669,  0.52559,  
//6.1888,  1.4096,  -0.11053,  -0.16184,  13.6357,  2.9717};
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

			DM_Class.mit_ctrl(&hfdcan1,Chassis.Joint_Motor[1].para.id,0.0f,0.0f,0.0f,0.0f,0.0f);
			osDelay(CHASSIS_TIME);
			DM_Class.mit_ctrl(&hfdcan1,Chassis.Joint_Motor[3].para.id,0.0f,0.0f,0.0f,0.0f,0.0f);
			osDelay(CHASSIS_TIME);
			
			DM_Class.mit_ctrl(&hfdcan3,Chassis.Wheel_Motor[0].para.id,0.0f,0.0f,0.0f,0.0f,0.0f);
			osDelay(CHASSIS_TIME);
			DM_Class.mit_ctrl(&hfdcan3,Chassis.Wheel_Motor[1].para.id,0.0f,0.0f,0.0f,0.0f,0.0f);
			osDelay(CHASSIS_TIME);
		}else{
			DM_Class.mit_ctrl(&hfdcan2,Chassis.Joint_Motor[0].para.id,0.0f,0.0f,0.0f,0.0f,leg.vmc[0].torque_set[0]);
			osDelay(CHASSIS_TIME);
			DM_Class.mit_ctrl(&hfdcan2,Chassis.Joint_Motor[2].para.id,0.0f,0.0f,0.0f,0.0f,leg.vmc[0].torque_set[1]);
			osDelay(CHASSIS_TIME);

			DM_Class.mit_ctrl(&hfdcan1,Chassis.Joint_Motor[1].para.id,0.0f,0.0f,0.0f,0.0f,-leg.vmc[1].torque_set[0]);
			osDelay(CHASSIS_TIME);
			DM_Class.mit_ctrl(&hfdcan1,Chassis.Joint_Motor[3].para.id,0.0f,0.0f,0.0f,0.0f,-leg.vmc[1].torque_set[1]);
			osDelay(CHASSIS_TIME);
			DM_Class.mit_ctrl(&hfdcan3,Chassis.Wheel_Motor[0].para.id,0.0f,0.0f,0.0f,0.0f,leg.wheel_T[0]);
			osDelay(CHASSIS_TIME);
			DM_Class.mit_ctrl(&hfdcan3,Chassis.Wheel_Motor[1].para.id,0.0f,0.0f,0.0f,0.0f,leg.wheel_T[1]);
			osDelay(CHASSIS_TIME);
		}
	}
}

void Chassis_Class::Chassis_Init(void){
	Mode = CHASSIS_STOP;
	offset[0] = 0.0f;//0.0-0.4
	offset[1] = 0.0f;
	k[0] = 0;
	k[1] = 0;
	Chassis_Task_DWT_dt = 0;
	Chassis_Task_DWT_Count = 0;
	wheel_pid[0] = 5;
	Leg_Init();
{	PID.Init(&Leg_Roll_Pid, POSITION,
		LEG_ROLL_PID_KP,
		LEG_ROLL_PID_KI,
		LEG_ROLL_PID_KD,
		LEG_ROLL_PID_MAX_OUT,
		LEG_ROLL_PID_MAX_IOUT,
		LEG_ROLL_PID_BAND_I);
	PID.Init(&Leg_Angle0_err_Pid, POSITION,
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
	PID.Init(&Leg_L0_Pid[0],
			POSITION,
			LEG_L0_PID_KP,
			LEG_L0_PID_KI,
			LEG_L0_PID_KD,
			LEG_L0_PID_MAX_OUT,
			LEG_L0_PID_MAX_IOUT,
			LEG_L0_PID_BAND_I);
	PID.Init(&Leg_L0_Pid[1],
			POSITION,
			LEG_L0_PID_KP,
			LEG_L0_PID_KI,
			LEG_L0_PID_KD,
			LEG_L0_PID_MAX_OUT,
			LEG_L0_PID_MAX_IOUT,
			LEG_L0_PID_BAND_I);
	PID.Init(&Leg_L0_Speed_Pid[0],
			 POSITION,
			 LEG_L0_SPEED_PID_KP,
			 LEG_L0_SPEED_PID_KI,
			 LEG_L0_SPEED_PID_KD,
			 LEG_L0_SPEED_PID_MAX_OUT,
			 LEG_L0_SPEED_PID_MAX_IOUT,
			 LEG_L0_SPEED_PID_BAND_I);
	PID.Init(&Leg_L0_Speed_Pid[1],
			 POSITION,
			 LEG_L0_SPEED_PID_KP,
			 LEG_L0_SPEED_PID_KI,
			 LEG_L0_SPEED_PID_KD,
			 LEG_L0_SPEED_PID_MAX_OUT,
			 LEG_L0_SPEED_PID_MAX_IOUT,
			 LEG_L0_SPEED_PID_BAND_I);
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
	Feedback_Update();
	DM_Class.Joint_Motor_Init(&Chassis.Joint_Motor[0],6,MIT_MODE);
	DM_Class.Joint_Motor_Init(&Chassis.Joint_Motor[2],8,MIT_MODE);
	
	DM_Class.Joint_Motor_Init(&Chassis.Joint_Motor[1],6,MIT_MODE);
	DM_Class.Joint_Motor_Init(&Chassis.Joint_Motor[3],8,MIT_MODE);
	DM_Class.Wheel_Motor_Init(&Chassis.Wheel_Motor[0],0,MIT_MODE);
	DM_Class.Wheel_Motor_Init(&Chassis.Wheel_Motor[1],1,MIT_MODE);

	DM_Class.enable_motor(&hfdcan2,Chassis.Joint_Motor[0].para.id,Chassis.Joint_Motor[0].mode);
	osDelay(5);
	DM_Class.enable_motor(&hfdcan2,Chassis.Joint_Motor[2].para.id,Chassis.Joint_Motor[2].mode);
	osDelay(5);
	DM_Class.enable_motor(&hfdcan1,Chassis.Joint_Motor[1].para.id,Chassis.Joint_Motor[1].mode);
	osDelay(5);
	DM_Class.enable_motor(&hfdcan1,Chassis.Joint_Motor[3].para.id,Chassis.Joint_Motor[3].mode);
	osDelay(5);
	DM_Class.enable_motor(&hfdcan3,Chassis.Wheel_Motor[0].para.id,Chassis.Wheel_Motor[0].mode);
	osDelay(5);
	DM_Class.enable_motor(&hfdcan3,Chassis.Wheel_Motor[1].para.id,Chassis.Wheel_Motor[1].mode);
	osDelay(5);
}
void Chassis_Class::Feedback_Update(void){
	K_filter_gyro[0] = 0.05f;

	if(need_enable_loop_flag){
		uint8_t i = 0;
		Chassis_DWT_dt = DWT_GetDeltaT(&Chassis_DWT_Count);
			if(dog.Remote_Dog.State == Device_Online){
				
			}
	}
	last_Gyro[0] = Gyro[0];
	Gyro[0] = K_filter_gyro[0] * BMI088.Gyro[0] + (1 - K_filter_gyro[0]) * last_Gyro[0];
	for (uint8_t i = 0; i < 2; i++){
		leg.vmc[i].point.phi1 = PI / 2.0f + pow(-1.0, i) * Joint_Motor[0 + i].para.POS;
		leg.vmc[i].point.phi4 = PI / 2.0f + pow(-1.0, i) * Joint_Motor[2 + i].para.POS;

		leg.vmc[i].F_fdb.Tp_1_fdb = Joint_Motor[0 + i].para.Torque;
		leg.vmc[i].F_fdb.Tp_2_fdb = Joint_Motor[2 + i].para.Torque;

		leg.vmc[i].vmc_calc(-INS.Pitch, 
							Gyro[0],
							1.0f / 1000.0f,
							pow(-1.0, i) * Joint_Motor[0 + i].para.VEL, 
							pow(-1.0, i) * Joint_Motor[2 + i].para.VEL);

		Wheel_Motor[i].speed = Wheel_Motor[i].para.VEL;
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
						Mode = CHASSIS_STOP;
					break;
				case 2:
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
	Init_Lout_M  = LEG_GRAVITY;//the weight of all car
	
	L0_Leg_KP = LEG_L0_PID_KP;
	L1_Leg_KP = LEG_L1_PID_KP;
	L0_Leg_Speed_KP = LEG_L0_SPEED_PID_KP;

	ramp_period = 0.01;
	LITTLE_TOP_V = 1.2;
	V_COLLAPSE = 0.5;
	FN_max = 5;
	Power_Set_KP = 0.23;
	
//	Power_Ctrl.power_buffer_set = 30;
//	Flags.Jump_Flag = false;
//	Flags.jump_Flag = false;

	/* 腿的位置 旋转速度 收腿速度*/
	leg.stand.Stand_Speed = 0.5f;//
	leg.stand.L0Speed = -0.3f;
	leg.stand.Stand_Angle[0] = 0.25f;
	leg.stand.Stand_Angle[1] = 0.25f;
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
			leg.leg_flag.Stand_flag = 0;//1
			leg.leg_flag.Revolve_flag_L = false;//true
			leg.leg_flag.Revolve_flag_R = false;
			leg.leg_flag.Blance_flag = true;
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
		leg.leg_set.L0_set = ramp_float(leg.leg_set.L0_set_final, leg.leg_set.L0_set, 0.00065);//0.00065
		Roll_set = ramp_float(INS.Roll + remote.Key_ch[0] * ROLL_SET_TC_SEN,Roll_set,0.0075);
		Limit_min_max(&leg.leg_set.L0_set, leg.leg_set.L0_set_min, leg.leg_set.L0_set_max);
		
		Leg_Ctrl.Leg_set.Wheel_Speed_set[0] = vx_set[0];
		Leg_Ctrl.Leg_set.Wheel_Speed_set[1] = vx_set[1];
		Leg_Ctrl.Leg_set.Wheel_Collapse_V_set = Leg_Ctrl.Leg_set.Wheel_Collapse_V_set = 0;
		Leg_Ctrl.Leg_set.yaw_Gyro_set = angle_set;
		Leg_Ctrl.Leg_set.yaw_Gyro_set = angle_set;
	}
	if (Mode == CHASSIS_STOP){
		Leg_Ctrl.Leg_set.Wheel_Speed_set[0] = Leg_Ctrl.Leg_set.Wheel_Speed_set[1] = 0;
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
				PID.Calc(&Chassis.Stand_Position_Pid[0], leg.vmc[0].point.phi0, leg.stand.Stand_Angle[1]);
				Limit_min_max(&Chassis.Stand_Position_Pid[0].out,-2.0f,2.0f);
				PID.Calc(&Chassis.Stand_Speed_Pid[0], leg.vmc[0].point.d_phi0, Chassis.Stand_Position_Pid[0].out);
				leg.vmc[0].Tp = Chassis.Stand_Speed_Pid[0].out;
				leg.vmc[0].vmc_forward();
				Limit_min_max(&leg.vmc[0].torque_set[0],-3.0f,3.0f);
				Limit_min_max(&leg.vmc[0].torque_set[1],-3.0f,3.0f);
			}
			if(leg.leg_flag.Revolve_flag_R){
				PID.Calc(&Chassis.Stand_Position_Pid[1],leg.vmc[1].point.phi0,leg.stand.Stand_Angle[1]);
				Limit_min_max(&Chassis.Stand_Position_Pid[1].out, -2.0f, 2.0f);
				PID.Calc(&Chassis.Stand_Speed_Pid[1], leg.vmc[1].point.d_phi0, Chassis.Stand_Position_Pid[1].out);				
				leg.vmc[1].Tp = Chassis.Stand_Speed_Pid[1].out;
				leg.vmc[1].vmc_forward();
				Limit_min_max(&leg.vmc[1].torque_set[0], -3.0f, 3.0f);
				Limit_min_max(&leg.vmc[1].torque_set[1], -3.0f, 3.0f);
			}
			/*判断是否完成站立 完成后对左右腿进行数据清零*/
			if(fabs(leg.stand.Stand_Angle[0] - leg.vmc[0].point.phi0) <0.2f){
				leg.leg_data_clear(Left_Leg);
				PID.Clear(&Stand_Speed_Pid[0]);
				leg.leg_flag.Revolve_flag_L = false;		
			}
			if(fabs(leg.stand.Stand_Angle[1] - leg.vmc[1].point.phi0) <0.2f){
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
			
			Limit_min_max(&leg.vmc[0].torque_set[0], -2.0f, 2.0f);
			Limit_min_max(&leg.vmc[0].torque_set[1], -2.0f, 2.0f);

			Limit_min_max(&leg.vmc[1].torque_set[0], -2.0f, 2.0f);
			Limit_min_max(&leg.vmc[1].torque_set[1], -2.0f, 2.0f);
			
			if(leg.vmc[0].point.L0 <0.025 && leg.vmc[1].point.L0 <0.025){
				osDelay(50);
				leg.leg_flag.Stand_flag ++;;
			}
			break;
		case 4 :
			leg.leg_set.L0_set = 0.020f;
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
			leg.wheel_T[i] = LQR_K[0] * (leg.vmc[i].theta - offset[0]) 
										 + LQR_K[1] * (leg.vmc[i].d_theta - 0.0f) 
										 + LQR_K[2] * (x_fdb - 0.0f) 
										 + LQR_K[3] * (Velocity.vx - Leg_Ctrl.Leg_set.Wheel_Speed_set[0]) 
										 + LQR_K[4] * (-INS.Pitch - (-0.0f)) 
										 + LQR_K[5] * (-Gyro[0]- 0.0f);
			leg.vmc[i].Tp = LQR_K[6] * (leg.vmc[i].theta - offset[1]) 
										+ LQR_K[7] * (leg.vmc[i].d_theta - 0.0f) 
										+ LQR_K[8] * (x_fdb - 0.0f) 
										+ LQR_K[9] * (Velocity.vx - Leg_Ctrl.Leg_set.Wheel_Speed_set[1]) 
										+ LQR_K[10] * (-INS.Pitch - -0.0f) 
										+ LQR_K[11] * (-Gyro[0] - 0.0f);
			PID.Calc(&Chassis.Leg_L0_Pid[i], leg.vmc[i].point.L0, leg.leg_set.L0_set);
			PID.Calc(&Chassis.Leg_L0_Speed_Pid[i], leg.vmc[i].point.d_L0, 0.0f);
			PID.Calc(&Chassis.Leg_Roll_Pid,INS.Roll,Chassis.Roll_set);
		}
	PID.Calc(&Chassis.Leg_Angle0_err_Pid, leg.vmc[0].point.phi0 - leg.vmc[1].point.phi0, 0.0f);
	leg.vmc[0].Tp += Leg_Angle0_err_Pid.out;
	leg.vmc[1].Tp -= Leg_Angle0_err_Pid.out;
	
	leg.vmc[0].F0 = Leg_L0_Pid[0].out + Leg_L0_Speed_Pid[0].out + Leg_Roll_Pid.out + k[0] *cos(leg.vmc[0].theta);
	leg.vmc[0].vmc_forward();

	leg.vmc[1].F0 = Leg_L0_Pid[1].out + Leg_L0_Speed_Pid[1].out - Leg_Roll_Pid.out + k[1] *cos(leg.vmc[1].theta);
	leg.vmc[1].vmc_forward();
	}
	Limit_min_max(&leg.vmc[0].torque_set[0], -4.0f, 4.0f);
	Limit_min_max(&leg.vmc[0].torque_set[1], -4.0f, 4.0f);

	Limit_min_max(&leg.vmc[1].torque_set[0], -4.0f, 4.0f);
	Limit_min_max(&leg.vmc[1].torque_set[1], -4.0f, 4.0f);

	Limit_min_max(&leg.wheel_T[0], -0.45f, 0.45f);
	Limit_min_max(&leg.wheel_T[1], -0.45f, 0.45f);
}
void Chassis_Class::Ground_detection(){

}
void Chassis_Class::Turn_Dispose(){
	if(Mode == CHASSIS_RUN && Leg_Ctrl.Leg_set.Wheel_Speed_set[0] > 0.0f && Leg_Ctrl.Leg_set.Wheel_Speed_set[1] > 0.0f){
		if (Leg_Ctrl.Vmc_Ctrl[1].Wheel_Speed_Estimate > Leg_Ctrl.Vmc_Ctrl[0].Wheel_Speed_Estimate && Leg_Ctrl.Leg_set.yaw_Gyro_set > 0){
			Leg_Ctrl.FN[0] = LEG_GRAVITY * (2 * (Leg_Ctrl.Vmc_Ctrl[0].WBR.L0_fdb.now + Leg_Ctrl.Vmc_Ctrl[1].WBR.L0_fdb.now) + Leg_Ctrl.Rl) *
							 (Leg_Ctrl.Vmc_Ctrl[1].Wheel_Speed_Estimate * Leg_Ctrl.Vmc_Ctrl[1].Wheel_Speed_Estimate - Leg_Ctrl.Vmc_Ctrl[0].Wheel_Speed_Estimate * Leg_Ctrl.Vmc_Ctrl[0].Wheel_Speed_Estimate) / 8.0f / Leg_Ctrl.Rl / Leg_Ctrl.Rl;
			Leg_Ctrl.FN[1] = 0;
		}
		if (Leg_Ctrl.Vmc_Ctrl[1].Wheel_Speed_Estimate < Leg_Ctrl.Vmc_Ctrl[0].Wheel_Speed_Estimate && Leg_Ctrl.Leg_set.yaw_Gyro_set < 0){
			Leg_Ctrl.FN[1] = LEG_GRAVITY * (2 * (Leg_Ctrl.Vmc_Ctrl[0].WBR.L0_fdb.now + Leg_Ctrl.Vmc_Ctrl[1].WBR.L0_fdb.now) + Leg_Ctrl.Rl) *
							 (Leg_Ctrl.Vmc_Ctrl[0].Wheel_Speed_Estimate * Leg_Ctrl.Vmc_Ctrl[0].Wheel_Speed_Estimate - Leg_Ctrl.Vmc_Ctrl[1].Wheel_Speed_Estimate * Leg_Ctrl.Vmc_Ctrl[1].Wheel_Speed_Estimate) / 8.0f / Leg_Ctrl.Rl / Leg_Ctrl.Rl;
			Leg_Ctrl.FN[0] = 0;
		}
		Limit_min_max(&Leg_Ctrl.FN[0], -100.0f, 100.0f);
		Limit_min_max(&Leg_Ctrl.FN[1], -100.0f, 100.0f);		
	}else{
			Leg_Ctrl.FN[0] = Leg_Ctrl.FN[1] = 0;		
	}
}
void Chassis_Class::Jump_Dispose(){
	if(dog.Remote_Dog.State != Device_Online){
		Flags.Jump_Flag = false;
		Flags.Jump_MAX_Flag = false;
		Flags.Start_Jump_Flag = false;
		Flags.Jump_shrink_Flag = false;
		Flags.jump_Flag = false;	
	}
	/*起跳开始先缩腿*/
	if(dog.Remote_Dog.State == Device_Online && Mode == CHASSIS_RUN && remote.Key_ch[4] == 1 && Flags.Jump_Flag == false){
		Flags.jump_Flag = true;
		Flags.Start_Jump_Flag = true;
		/*先缩腿*/
		Leg_Ctrl.Leg_set.L0_set = LQR_Ctrl.Init_Lset;
		/*增大对腿长的收敛速度*/
		L0_Leg_KP = L1_Leg_KP = LQR_Ctrl.Jump_KP;
		L0_Leg_Speed_KP = LQR_Ctrl.Jump_Speed_KP;
		/*若不开调，运行此行*/
		// Flags.Start_Jump_Flag=false;		
	}
	/*伸腿*/
	if(Flags.Start_Jump_Flag == true && ((Leg_Ctrl.Vmc_Ctrl[0].WBR.L0_fdb.now) < LQR_Ctrl.Init_Lset + LQR_Ctrl.Jump_min) && ((Leg_Ctrl.Vmc_Ctrl[1].WBR.L0_fdb.now) < LQR_Ctrl.Init_Lset + LQR_Ctrl.Jump_min) && (Flags.Jump_Flag == false)){
		/*起跳标志位*/
		Flags.Jump_Flag = true;
		/*伸腿*/
		Leg_Ctrl.Leg_set.L0_set = LQR_Ctrl.MAX_Lset;
	/*支持力为机体重量*/
		Init_Lout = Init_Lout_M;		
	}
	/*伸腿完成后缩腿，惯性起跳，处于悬空状态，只保留对theta的约束*/
	else if (Flags.Jump_Flag == true && ((Leg_Ctrl.Vmc_Ctrl[0].WBR.L0_fdb.now) > (LQR_Ctrl.MAX_Lset - LQR_Ctrl.Jump_err)) && ((Leg_Ctrl.Vmc_Ctrl[1].WBR.L0_fdb.now) > (LQR_Ctrl.MAX_Lset - LQR_Ctrl.Jump_err)) && Flags.Jump_MAX_Flag == false){
		/*达到最大腿长*/
		Flags.Jump_MAX_Flag = true;
		/*清零增益K*/
		Clear_K(LQR_Ctrl.Wheel_Kgain1, LQR_Ctrl.Joint_Kgain1);
		/*增大对腿长的收敛速度*/
		L0_Leg_KP = L1_Leg_KP = LQR_Ctrl.Jump_shou_KP;
		L0_Leg_Speed_KP = LQR_Ctrl.Jump_Speed_shou_KP;
		/*对腿倾角的约束K赋值*/
		LQR_Ctrl.Joint_Kgain1[0][4] = LQR_Ctrl.LQR_Joint[0][4];
		LQR_Ctrl.Joint_Kgain1[0][5] = LQR_Ctrl.LQR_Joint[0][5];
		LQR_Ctrl.Joint_Kgain1[0][6] = LQR_Ctrl.LQR_Joint[0][6];
		LQR_Ctrl.Joint_Kgain1[0][7] = LQR_Ctrl.LQR_Joint[0][7];
		LQR_Ctrl.Joint_Kgain1[1][4] = LQR_Ctrl.LQR_Joint[1][4];
		LQR_Ctrl.Joint_Kgain1[1][5] = LQR_Ctrl.LQR_Joint[1][5];
		LQR_Ctrl.Joint_Kgain1[1][6] = LQR_Ctrl.LQR_Joint[1][6];
		LQR_Ctrl.Joint_Kgain1[1][7] = LQR_Ctrl.LQR_Joint[1][7];
	/*支持力为腿的重量向下*/
		Init_Lout = Wheel_Init_Lout;
		/*回到最短腿长*/
		Leg_Ctrl.Leg_set.L0_set = LQR_Ctrl.Init_Lset;
	}
	/*到达最短腿长后复原*/
	else if ((Flags.Jump_MAX_Flag == true) && (Flags.Jump_Flag == true) && ((Leg_Ctrl.Vmc_Ctrl[0].WBR.L0_fdb.now) < LQR_Ctrl.Init_Lset + LQR_Ctrl.Jump_min) && ((Leg_Ctrl.Vmc_Ctrl[1].WBR.L0_fdb.now) < LQR_Ctrl.Init_Lset + LQR_Ctrl.Jump_min)){
		/*缩腿完成*/
		Flags.Jump_shrink_Flag = true;
		/*跳跃结束*/
		Flags.jump_Flag = false;
		/*下落腿长PID*/
		L0_Leg_KP = L1_Leg_KP = LQR_Ctrl.Fall_KP;
		L0_Leg_Speed_KP = LQR_Ctrl.Fall_Speed_KP;
	/*支持力为腿的重量向上*/
		Init_Lout = Init_Lout_M;
		/*回到正常腿长*/
		Leg_Ctrl.Leg_set.L0_set = LQR_Ctrl.Middle_Lset;
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
void Clear_K(float Wheel[2][10], float Joint[2][10]){
	uint8_t i = 0, j = 0;
	for (i = 0; i < 2; i++){
		for (j = 0; j <= 9; j++){
			Wheel[i][j] = 0;
			Joint[i][j] = 0;
		}
	}
}