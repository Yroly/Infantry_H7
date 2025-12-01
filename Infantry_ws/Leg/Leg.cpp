#include "Leg.h"
#include "app_preference.h"
#include "LQR.h"
#include "Leg.h"
#include "Mpc.h"
#include "Chassis_Task.h"

LegClass  Leg_Ctrl;

LegClass::LegClass()
{
	  Vmc_Ctrl[0].Leg_DWT_Count=Vmc_Ctrl[1].Leg_DWT_Count=0;
	  Vmc_Ctrl[0].KF_DWT_Count =Vmc_Ctrl[0].KF_DWT_Count=0;
	
	  Mw_Wheel=0.648;    //1kg
		Ml_leg  =2.033f;   //单位：kg
	  Rl      =0.44/2.0f;//驱动轮间距
		Kn      =0;
	
    Vmc_Ctrl[0].l1 =Vmc_Ctrl[1].l1 = 0.13597f;  
    Vmc_Ctrl[0].l2 =Vmc_Ctrl[1].l2 = 0.27739f;
    Vmc_Ctrl[0].l3 =Vmc_Ctrl[1].l3 = 0.27739f;
    Vmc_Ctrl[0].l4 =Vmc_Ctrl[1].l4 = 0.13597f;
    Vmc_Ctrl[0].l5 =Vmc_Ctrl[1].l5 = 0.200f;
	
	  Vmc_Ctrl[0].xc = Vmc_Ctrl[1].xc=0.0f,
	  Vmc_Ctrl[0].yc = Vmc_Ctrl[1].yc=0.0f;
	  Vmc_Ctrl[0].angle1=Vmc_Ctrl[1].angle1=PI/3*2;
	  Vmc_Ctrl[0].angle4=Vmc_Ctrl[1].angle4=PI/3;
	  Vmc_Ctrl[0].WBR.angle0.now=Vmc_Ctrl[1].WBR.angle0.now=0.0f;
		 
	  Vmc_Ctrl[0].Leg_out.F_out =0;
	  Vmc_Ctrl[0].Leg_out.F_Init_out= LEG_GRAVITY*g_HENGYANG ;
		
	  Leg_fdb.Pitch_fdb=0;
		Vmc_Ctrl[0].Distance_Observe_fdb=Vmc_Ctrl[1].Distance_Observe_fdb=0;
		Vmc_Ctrl[0].Wheel_Speed_fdb=Vmc_Ctrl[1].Wheel_Speed_fdb=0;
		Leg_fdb.Pitch_Gyro_fdb=0;
		Leg_fdb.Yaw_fdb=0;
		Leg_fdb.Yaw_Gyro_fdb=0;
		
    Vmc_Ctrl[0].Zjie();
		Vmc_Ctrl[1].Zjie();
	
    Leg_set.L0_set = 0.23;//初值
		K_filter_gyro=0.6;

}



void VmcClass::K_Matching()//，更新K增益
{

	Zjie();//，先正解更新出L0，再更新K增益
	Support_Force_resolving();//,支持力解算，放在正解后面，因为需要用到L0,angle0，angle1，angle2，angle3，angle4

	
}

/**
 * @brief: 运动学逆解
 * @author: Dandelion
 * @param {float} xc，五连杆末端坐标
 * @param {float} yc，五连杆末端坐标，可以作为目标值输入，然后解算出所需的关节角度
 * @return {*}
 */
void VmcClass::Njie(const float xc, const float yc)
{
    this->xc = xc;
    this->yc = yc;

    float m, n, b, x1, y1;
    float A, B, C;

    A = 2 * l1 * yc;
    B = 2 * l1 * (xc + l5 / 2);
    C = l2 * l2 - l1 * l1 - xc * xc - yc * yc - l5 * l5 / 4 + xc * l5;
    angle1 = 2 * atan((A + sqrt(A * A + B * B - C * C)) / (B - C));
    if (angle1 < 0)
        angle1 += 2 * PI;

    // nije_5(&angle1, (void *)0, x, y, l1, l6, l3, l4, l5); //利用L1,L6计算c1;
    m = l1 * cos(angle1);
    n = l1 * sin(angle1);
    b = 0;
    // x1 = l2 / l6 * ((x - m) * cos(b) - (y - n) * sin(b)) + m;
    // y1 = l2 / l6 * ((x - m) * sin(b) + (y - n) * cos(b)) + n; //得到闭链五杆端点的坐标
    x1 = ((xc - m) * cos(b) - (yc - n) * sin(b)) + m;
    y1 = ((xc - m) * sin(b) + (yc - n) * cos(b)) + n; // 得到闭链五杆端点的坐标

    A = 2 * y1 * l4;
    B = 2 * l4 * (x1 - l5 / 2);
    // c = l3 * l3 + 2 * l5 * x1 - l4 * l4 - l5 * l5 - x1 * x1 - y1 * y1;
    C = l3 * l3 + l5 * x1 - l4 * l4 - l5 * l5 / 4 - x1 * x1 - y1 * y1;
    angle4 = 2 * atan((A - sqrt(A * A + B * B - C * C)) / (B - C));
    // nije_5((void *)0, &angle2, x1, y1, l1, l2, l3, l4, l5);        //计算c4 ,
}
/**
 * @brief: 运动学正解
 * @author: Dandelion
 * @param {float} angle1，前关节电机角度
 * @param {float} angle4，后关节电机角度
 * @param {float} pitch %rad，上层机构偏移
 * @return {*}
 */
void VmcClass::Zjie()
{
	  xb = l1 * cos(angle1);
    yb = l1 * sin(angle1);
	  xd = l5 + l4 * cos(angle4);
    yd = l4 * sin(angle4);
    float lbd = sqrt(pow((xd - xb), 2) + pow((yd - yb), 2));
    float A0 = 2 * l2 * (xd - xb);
    float B0 = 2 * l2 * (yd - yb);
    float C0 = pow(l2, 2) + pow(lbd, 2) - pow(l3, 2);
    angle2 = 2 * atan2((B0 + sqrt(pow(A0, 2) + pow(B0, 2) - pow(C0, 2))) , (A0 + C0));
    xc = xb + l2 * cos(angle2);
    yc = yb + l2 * sin(angle2);
	  angle3 = atan2((yc-yd),(xc-xd));

	  WBR.L0_fdb.last=  WBR.L0_fdb.now;
    WBR.L0_fdb.now= sqrt(pow((xc-l5/2), 2) + pow(yc, 2));
	
    WBR.angle0.last = WBR.angle0.now;
    WBR.angle0.now = atan2(yc,xc-l5/2);
	
    WBR.Tilt_angle_0.now=WBR.angle0.now-PI/2+Leg_Ctrl.Leg_fdb.Pitch_fdb;//轮腿模型下转换坐标系
}
/**
 * @brief: VMC_Positive（虚拟力算法,正解）
 * @author: Dandelion
 * @param {float} TP_H_Q 关节电机力矩,，按照玺佬图，这两个输出都是向左
 * @param {float} F  沿杆方向的受力，，五连杆末端向上的推力
 * @param {float} Tp 杆所受的力矩，，输出向左
 * @return {*}
 */
float TP_limit=0.5;
void VmcClass::VMC_Positive()
{

	 Leg_out.TP_1_out=
										 l1 * sin(WBR.angle0.now - angle3) * sin(angle1 - angle2) 
										 /sin(angle3 - angle2)*Leg_out.F_out
										 +l1 * cos(WBR.angle0.now - angle3) * sin(angle1 - angle2) 
										 /(WBR.L0_fdb.now * sin(angle3 - angle2))*Leg_out.Tp_out;
	
	 Leg_out.TP_2_out=
										 l4 * sin(WBR.angle0.now - angle2) * sin(angle3 - angle4) 
										 /sin(angle3 - angle2)*Leg_out.F_out
										 +l4 * cos(WBR.angle0.now - angle2) * sin(angle3 - angle4) 
										 /(WBR.L0_fdb.now * sin(angle3 - angle2))*Leg_out.Tp_out;
//		Leg_out.TP_1_out=0;//fp32_constrain(Leg_out.TP_1_out,-TP_limit,TP_limit);
//	  Leg_out.TP_2_out=0;//fp32_constrain(Leg_out.TP_2_out,-TP_limit,TP_limit);
	#ifdef VI_debug
		Leg_out.TP_1_out=0;//fp32_constrain(Leg_out.TP_1_out,-TP_limit,TP_limit);
	  Leg_out.TP_2_out=0;//fp32_constrain(Leg_out.TP_2_out,-TP_limit,TP_limit);
	#endif
}
/**
 * @brief: VMC_Negative（虚拟力算法，逆解）
 * @author: Dandelion
 * @param {float} TP_H_Q 关节电机力矩,按照玺佬图，这两个输出都是向左
 * @param {float} F 沿杆方向的受力,五连杆末端向上的推力
 * @param {float} Tp 杆所受的力矩,输出向左
 * @return {*}
 */
//,关节电机反馈扭矩->足端反馈力
 void VmcClass::VMC_Negative()
{
	  WBR.F_fdb=(
										-((cos(WBR.angle0.now - angle2)
										*sin(angle2 - angle3))
										/(l1*cos(WBR.angle0.now - angle2)
										*sin(WBR.angle0.now - angle3)
										*sin(angle1 - angle2) 
										- l1*cos(WBR.angle0.now - angle3)*sin(WBR.angle0.now - angle2)
										*sin(angle1 - angle2)))
										*WBR.TP_1_fdb

										+((cos(WBR.angle0.now - angle3)*sin(angle2 - angle3))
										/(l4*cos(WBR.angle0.now - angle2)*sin(WBR.angle0.now - angle3)
										*sin(angle3 - angle4) - l4*cos(WBR.angle0.now - angle3)
										*sin(WBR.angle0.now - angle2)*sin(angle3 - angle4)))
										*WBR.TP_2_fdb);
							
	WBR.Tp_fdb=
									 ((WBR.L0_fdb.now*sin(WBR.angle0.now - angle2)*sin(angle2 - angle3))
									 /(l1*cos(WBR.angle0.now - angle2)
									 *sin(WBR.angle0.now - angle3)*sin(angle1 - angle2)
									 - l1*cos(WBR.angle0.now - angle3)*sin(WBR.angle0.now - angle2)
									 *sin(angle1 - angle2)))*WBR.TP_1_fdb

									+(-(WBR.L0_fdb.now*sin(WBR.angle0.now - angle3)*sin(angle2 - angle3))
									/(l4*cos(WBR.angle0.now - angle2)*sin(WBR.angle0.now - angle3)
									*sin(angle3 - angle4) - l4*cos(WBR.angle0.now - angle3)
									*sin(WBR.angle0.now - angle2)*sin(angle3 - angle4)))
									*WBR.TP_2_fdb;
}

//五连杆的速度运动学结算,传入w1和w4角速度,关节电机反馈的角速度------>足端的速度(非常完美的计算)
void VmcClass::VMC_Positive_Kinematics()//，需先正运动解(知angle1，angle2，angle3，angle4)
{
   Leg_dt=DWT_GetDeltaT(&Leg_DWT_Count);
	
	/*********************************************/
	 WBR.L0_fdb.dot_last=WBR.L0_fdb.dot_now;
	 
	 WBR.L0_fdb.dot_now=l1*sin(WBR.angle0.now - angle3) * sin(angle1 - angle2)/sin(angle3 - angle2)*angle1_dot +l4 * sin(WBR.angle0.now - angle2) * sin(angle3 - angle4) / sin(angle3 - angle2)*angle4_dot;
	 
	 WBR.L0_fdb.ddot=(WBR.L0_fdb.dot_now-WBR.L0_fdb.dot_last)/Leg_dt;

	 WBR.angle0.dot_now=+l1*cos(WBR.angle0.now-angle3)*sin(angle1 - angle2)/(WBR.L0_fdb.now * sin(angle3 - angle2))*angle1_dot+l4 * cos(WBR.angle0.now - angle2) * sin(angle3 - angle4) /(WBR.L0_fdb.now * sin(angle3 - angle2))*angle4_dot; 
	/*********************************************/

		WBR.Tilt_angle_0.dot_last=WBR.Tilt_angle_0.dot_now;
		WBR.Tilt_angle_0.dot_now =WBR.angle0.dot_now;

		WBR.Tilt_angle_0.ddot=(WBR.Tilt_angle_0.dot_now-WBR.Tilt_angle_0.dot_last)/Leg_dt;
	
}

//支持力解算
void VmcClass::Support_Force_resolving()//，放在正解后
{
	VMC_Negative();  //反馈更新末端力和摆转矩
	
	VMC_Positive_Kinematics();//更新L0和angle0的一阶微分和二阶微分
	
  WBR.FN_fdb= WBR.F_fdb *cos(WBR.Tilt_angle_0.now)
						 +WBR.Tp_fdb*sin(WBR.Tilt_angle_0.now)/WBR.L0_fdb.now
						 +Leg_Ctrl.Mw_Wheel*(g_HENGYANG+Leg_Ctrl.Leg_fdb.Accel_Z-WBR.L0_fdb.ddot*cos(WBR.Tilt_angle_0.now)
						 +2*WBR.L0_fdb.dot_now*WBR.Tilt_angle_0.dot_now*sin(WBR.Tilt_angle_0.now)
						 +WBR.L0_fdb.now*WBR.Tilt_angle_0.ddot*sin(WBR.Tilt_angle_0.now)
						 +WBR.L0_fdb.now*pow(WBR.Tilt_angle_0.dot_now,2)*cos(WBR.Tilt_angle_0.now));
}
void VmcClass::Absolute_Speed_KF_Init()//先跑一遍KF_Feedback_Update初始化观测值
{
	Leg_KF.xhat_data[0]=Leg_KF.z_data[0]=Distance_Observe_fdb;//,先初始化估计值为观测值
	Leg_KF.xhat_data[1]=Leg_KF.z_data[1]=Wheel_Speed_Forecast;//
	
	Leg_KF.A_data[0]=1;
	Leg_KF.A_data[1]=KF_dt;
	Leg_KF.A_data[2]=0;
	Leg_KF.A_data[3]=1;
	
	
  Leg_KF.B_data[0]=KF_dt;
	Leg_KF.B_data[1]=KF_dt*KF_dt/2;
	
	
	Leg_KF.W_AAccel_date=Wheel_Accel_Forecast;//加速度观测值

	Leg_KF.H_data[0]=1;
	Leg_KF.H_data[1]=0;
	Leg_KF.H_data[2]=0;
	Leg_KF.H_data[3]=1;

	Leg_KF.Q_melody =Q_DATE;
	Leg_KF.R0_melody=R0_DATE;
	Leg_KF.R1_melody=R1_DATE;

	Leg_KF.P_data[0]=0;
	Leg_KF.P_data[1]=0;
	Leg_KF.P_data[2]=0;
	Leg_KF.P_data[3]=0;

  mat_init(&Leg_KF.xhat, 2, 1, (float *)Leg_KF.xhat_data);
  mat_init(&Leg_KF.xhatminus, 2, 1, (float *)Leg_KF.xhatminus_data);
  mat_init(&Leg_KF.z, 2, 1, (float *)Leg_KF.z_data);
  mat_init(&Leg_KF.A, 2, 2, (float *)Leg_KF.A_data);
	mat_init(&Leg_KF.B, 2, 1, (float *)Leg_KF.B_data);
	mat_init(&Leg_KF.W_aaccel, 1, 1, (float *)&Leg_KF.W_AAccel_date);
  mat_init(&Leg_KF.H, 2, 2, (float *)Leg_KF.H_data);
  mat_init(&Leg_KF.Q, 2, 2, (float *)Leg_KF.Q_data);
  mat_init(&Leg_KF.R, 2, 2, (float *)Leg_KF.R_data);
  mat_init(&Leg_KF.P, 2, 2, (float *)Leg_KF.P_data);
  mat_init(&Leg_KF.Pminus, 2, 2, (float *)Leg_KF.Pminus_data);
  mat_init(&Leg_KF.K, 2, 2, (float *)Leg_KF.K_data);
  mat_init(&Leg_KF.AT, 2, 2, (float *)Leg_KF.AT_data);
  mat_trans(&Leg_KF.A, &Leg_KF.AT);
  mat_init(&Leg_KF.HT, 2, 2, (float *)Leg_KF.HT_data);
  mat_trans(&Leg_KF.H, &Leg_KF.HT);
	
  //matrix_value2 = F->A.pData[1];
}
void VmcClass::KF_Feedback_Update()//,计算轮子绝对速度
{
	 KF_dt=DWT_GetDeltaT(&KF_DWT_Count);
	 Last_Wheel_Speed_Forecast=Wheel_Speed_Forecast;
   Wheel_Speed_Forecast     =Wheel_Speed_fdb+WHEEL_RADIUS*(-Leg_Ctrl.Leg_fdb.Pitch_Gyro_fdb-WBR.Tilt_angle_0.dot_now)+WBR.L0_fdb.now*WBR.Tilt_angle_0.dot_now*cos(WBR.Tilt_angle_0.now)+WBR.L0_fdb.dot_now*sin(WBR.Tilt_angle_0.now);//速度观测值
	
	 Last_Wheel_Accel_Forecast=Wheel_Accel_Forecast;

	 Wheel_Accel_Forecast=INS.Accel[1];//,加速度观测值

	Leg_KF.Q_data[0]=Leg_KF.Q_melody;
	Leg_KF.Q_data[1]=Leg_KF.Q_melody;
	Leg_KF.Q_data[2]=Leg_KF.Q_melody;
	Leg_KF.Q_data[3]=Leg_KF.Q_melody;

	Leg_KF.R_data[0]=Leg_KF.R0_melody;
	Leg_KF.R_data[1]=0;
	Leg_KF.R_data[2]=0;
	Leg_KF.R_data[3]=Leg_KF.R1_melody;
}

void VmcClass::Absolute_Speed_KF_Forecast()//,卡尔曼更新速度位移
{
	float TEMP_data[4]   = {0, 0, 0, 0};
	float TEMP_data21[2] = {0, 0};
  mat TEMP, TEMP21;

  mat_init(&TEMP,  2, 2,(float *)TEMP_data);   //
  mat_init(&TEMP21,2, 1,(float *)TEMP_data21); //

	Leg_KF.A_data[0]=1;
	Leg_KF.A_data[1]=KF_dt;
	Leg_KF.A_data[2]=0;
	Leg_KF.A_data[3]=1;

	Leg_KF.B_data[0]=KF_dt*KF_dt/2;
	Leg_KF.B_data[1]=KF_dt;


	Leg_KF.W_AAccel_date=INS.Accel[1];//,加速度观测值
	
	if(
//		Chassis.Mode == CHASSIS_NO_MOVE
//		||Chassis.Flags.Liftoff_Flag[0]
//	  ||Chassis.Flags.Liftoff_Flag[1]
//		||Chassis.Mode==REVERSE_LITTLE_TOP
//	  ||Chassis.Mode==CHASSIS_LITTLE_TOP
//	||Chassis.Flags.Collapse_Flag
//	  ||
	(Leg_Ctrl.Leg_set.Wheel_Speed_set)
	  )
		Leg_KF.xhat.pData[0]=0;
	
	Leg_KF.z.pData[0]=Leg_KF.xhat.pData[0];//,位移观测
	Leg_KF.z.pData[1]=Wheel_Speed_Forecast;//,速度观测值
	
	mat_trans(&Leg_KF.A, &Leg_KF.AT);
	//,需先初始化估计值为观测值
  //1. xhat'(k)= A xhat(k-1)
  mat_mult(&Leg_KF.A, &Leg_KF.xhat, &Leg_KF.xhatminus);   //  x(k|k-1) = A*X(k-1|k-1)+B*U(k)+W(K)
  mat_mult(&Leg_KF.B, &Leg_KF.W_aaccel, &TEMP);           //  p(k|k-1) = A*p(k-1|k-1)*A'+Q
  mat_add(&TEMP, &Leg_KF.xhatminus, &Leg_KF.xhatminus);   //  p(k|k-1) = A*p(k-1|k-1)*A'+Q
  //2. P'(k) = A P(k-1) AT + Q
  mat_mult(&Leg_KF.A, &Leg_KF.P, &Leg_KF.Pminus);         //   p(k|k-1) = A*p(k-1|k-1)*A'+Q
  mat_mult(&Leg_KF.Pminus, &Leg_KF.AT, &TEMP);            //   p(k|k-1) = A*p(k-1|k-1)*A'+Q
  mat_add(&TEMP, &Leg_KF.Q, &Leg_KF.Pminus);              //   p(k|k-1) = A*p(k-1|k-1)*A'+Q

  //3. K(k) = P'(k) HT / (H P'(k) HT + R)
  mat_mult(&Leg_KF.H, &Leg_KF.Pminus, &Leg_KF.K);         //  kg(k) = p(k|k-1)*H'/(H*p(k|k-1)*H'+R)
  mat_mult(&Leg_KF.K, &Leg_KF.HT, &TEMP);                 //  kg(k) = p(k|k-1)*H'/(H*p(k|k-1)*H'+R)
  mat_add(&TEMP, &Leg_KF.R, &Leg_KF.K);                   //  kg(k) = p(k|k-1)*H'/(H*p(k|k-1)*H'+R)

  mat_inv(&Leg_KF.K, &Leg_KF.P);               //
  mat_mult(&Leg_KF.Pminus, &Leg_KF.HT, &TEMP); //
  mat_mult(&TEMP, &Leg_KF.P, &Leg_KF.K);       //

  //4. xhat(k) = xhat'(k) + K(k) (z(k) - H xhat'(k))
  mat_mult(&Leg_KF.H, &Leg_KF.xhatminus, &TEMP21);   //      x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))
  mat_sub(&Leg_KF.z, &TEMP21, &Leg_KF.xhat);         //      x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))
  mat_mult(&Leg_KF.K, &Leg_KF.xhat, &TEMP21);        //      x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))
  mat_add(&Leg_KF.xhatminus, &TEMP21, &Leg_KF.xhat); //      x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))

  //5. P(k) = (1-K(k)H)P'(k)
  mat_mult(&Leg_KF.K, &Leg_KF.H, &Leg_KF.P);         //      p(k|k) = (I-kg(k)*H)*P(k|k-1)
  mat_sub(&Leg_KF.Q, &Leg_KF.P, &TEMP);              //
  mat_mult(&TEMP, &Leg_KF.Pminus, &Leg_KF.P);  

	Wheel_Distance_Estimate_halt=Wheel_Distance_Estimate=Leg_KF.xhat.pData[0];
	Wheel_Speed_Estimate=Leg_KF.xhat.pData[1];
}
