#include "Mpc.h"
#include "LQR.h"
#include "Leg.h"
#include "app_preference.h"

MpcClass MPC_Ctrl;

MpcClass::MpcClass()
{	

}

void MpcClass::MPC_UpData(float L_leg,float R_leg)
{
	LQR_A_B_Clear(A_Data,B_Data);
	
	A_Data[0][0]=1;
	A_Data[1][1]=1;
	A_Data[2][2]=1;
  A_Data[3][3]=1;
	A_Data[4][4]=1;
	A_Data[5][5]=1;
	A_Data[6][6]=1;
	A_Data[7][7]=1;
	A_Data[8][8]=1;
	A_Data[9][9]=1;

	A_Data[0][1]=1*0.001;
	A_Data[2][3]=1*0.001;
	A_Data[4][5]=1*0.001;
	A_Data[6][7]=1*0.001;
	A_Data[8][9]=1*0.001;
	
	A_Data[1][4]=-27.6807*0.001;
	A_Data[1][6]=-27.6807*0.001;
	A_Data[1][8]= 0;
	A_Data[3][4]=-9.3791*0.001;
	A_Data[3][6]= 9.3791*0.001;
	A_Data[3][8]= 0;
	A_Data[5][4]= 378.1118*0.001;
	A_Data[5][6]= 20.7573*0.001;
	A_Data[5][8]= 0;	
	A_Data[7][4]= 20.7573*0.001;
	A_Data[7][6]= 378.1118*0.001;
	A_Data[7][8]= 0;	
	A_Data[9][4]=-82.7572*0.001;
	A_Data[9][6]=-82.7572*0.001;
	A_Data[9][8]= 98.7948*0.001;
	
	B_Data[1][0]= 9.0162*0.001;
	B_Data[1][1]= 9.0162*0.001;
	B_Data[1][2]=-2.2076*0.001;
	B_Data[1][3]=-2.2076*0.001;
	B_Data[3][0]=-6.4871*0.001;
	B_Data[3][1]= 6.4871*0.001;
	B_Data[3][2]=-0.7480*0.001;
	B_Data[3][3]= 0.7480*0.001;
	B_Data[5][0]=-107.2901*0.001;
	B_Data[5][1]=-10.0111*0.001;
	B_Data[5][2]= 30.1547*0.001;
	B_Data[5][3]= 1.6554*0.001;
	B_Data[7][0]=-10.0111*0.001;
	B_Data[7][1]=-107.2901*0.001;
	B_Data[7][2]= 1.6554*0.001;
	B_Data[7][3]= 30.1547*0.001;	
	B_Data[9][0]= 15.5185*0.001;
	B_Data[9][1]= 15.5185*0.001;
	B_Data[9][2]=-11.8631*0.001;
	B_Data[9][3]=-11.8631*0.001;

}

void MpcClass::MPC_Forecast_Init(void)
{
	mat_init(&A,10,10,(float *)A_Data);
	mat_init(&B,10, 4,(float *)B_Data);	

	for(uint8_t i=0;i<5;i++){
	mat_init(&F[0],10,10,(float *)A_Data);
	mat_init(&F[1],10,10,(float *)A_Data);
	mat_init(&F[2],10,10,(float *)A_Data);
	mat_init(&F[3],10,10,(float *)A_Data);
	mat_init(&F[4],10,10,(float *)A_Data);
	}
	for(uint8_t i=0;i<=5;i++){
		for(uint8_t j=0;j<=4;j++){
		  mat_init(&C[i][j],10,4,(float *)B_Data);
		}
	}

	mat_init(&xhat[0],10,1, (float *)xhat_data[0]);
	mat_init(&xhat[1],10,1, (float *)xhat_data[1]);
	mat_init(&xhat[2],10,1, (float *)xhat_data[2]);
	mat_init(&xhat[3],10,1, (float *)xhat_data[3]);
	mat_init(&xhat[4],10,1, (float *)xhat_data[4]);
	mat_init(&xhat[5],10,1, (float *)xhat_data[5]);
	
	mat_init(&U[0],4, 1,(float *)U_data[0]);
	mat_init(&U[1],4, 1,(float *)U_data[1]);
	mat_init(&U[2],4, 1,(float *)U_data[2]);
	mat_init(&U[3],4, 1,(float *)U_data[3]);
	mat_init(&U[4],4, 1,(float *)U_data[4]);
	mat_init(&U[5],4, 1,(float *)U_data[5]);

}
void MpcClass::MPC_U_i_Feedback(uint8_t i)
{
	U[i].pData[0]=-LQR_Ctrl.Wheel_Kgain1[0][0]*(Leg_Ctrl.Leg_set.Carload_Distance_set - xhat[i].pData[0])
	              -LQR_Ctrl.Wheel_Kgain1[0][1]*(Leg_Ctrl.Leg_set.Wheel_Speed_set      - xhat[i].pData[1])
								+LQR_Ctrl.Wheel_Kgain1[0][2]*(Leg_Ctrl.Leg_set.yaw_set              - xhat[i].pData[2])
                +LQR_Ctrl.Wheel_Kgain1[0][3]*(Leg_Ctrl.Leg_set.yaw_Gyro_set         - xhat[i].pData[3])
								+LQR_Ctrl.Wheel_Kgain1[0][4]*(Leg_Ctrl.Leg_set.Tilt_angle_0_set     - xhat[i].pData[4])
								+LQR_Ctrl.Wheel_Kgain1[0][5]*(0                                     - xhat[i].pData[5])
                +LQR_Ctrl.Wheel_Kgain1[0][6]*(Leg_Ctrl.Leg_set.Tilt_angle_0_set     - xhat[i].pData[6])
								+LQR_Ctrl.Wheel_Kgain1[0][7]*(0                                     - xhat[i].pData[7])
								+LQR_Ctrl.Wheel_Kgain1[0][8]*(0                                     - xhat[i].pData[8])
                +LQR_Ctrl.Wheel_Kgain1[0][9]*(0                                     - xhat[i].pData[9])
								;
	
	U[i].pData[1]=-LQR_Ctrl.Wheel_Kgain1[1][0]*(Leg_Ctrl.Leg_set.Carload_Distance_set - xhat[i].pData[0])
	              -LQR_Ctrl.Wheel_Kgain1[1][1]*(Leg_Ctrl.Leg_set.Wheel_Speed_set      - xhat[i].pData[1])
								+LQR_Ctrl.Wheel_Kgain1[1][2]*(Leg_Ctrl.Leg_set.yaw_set              - xhat[i].pData[2])
                +LQR_Ctrl.Wheel_Kgain1[1][3]*(Leg_Ctrl.Leg_set.yaw_Gyro_set         - xhat[i].pData[3])
								+LQR_Ctrl.Wheel_Kgain1[1][4]*(Leg_Ctrl.Leg_set.Tilt_angle_0_set     - xhat[i].pData[4])
								+LQR_Ctrl.Wheel_Kgain1[1][5]*(0                                     - xhat[i].pData[5])
                +LQR_Ctrl.Wheel_Kgain1[1][6]*(Leg_Ctrl.Leg_set.Tilt_angle_0_set     - xhat[i].pData[6])
								+LQR_Ctrl.Wheel_Kgain1[1][7]*(0                                     - xhat[i].pData[7])
								+LQR_Ctrl.Wheel_Kgain1[1][8]*(0                                     - xhat[i].pData[8])
                +LQR_Ctrl.Wheel_Kgain1[1][9]*(0                                     - xhat[i].pData[9])
								;		
								
  U[i].pData[2]=-LQR_Ctrl.Joint_Kgain1[0][0]*(Leg_Ctrl.Leg_set.Carload_Distance_set - xhat[i].pData[0])
		            -LQR_Ctrl.Joint_Kgain1[0][1]*(Leg_Ctrl.Leg_set.Wheel_Speed_set      - xhat[i].pData[1])
								+LQR_Ctrl.Joint_Kgain1[0][2]*(Leg_Ctrl.Leg_set.yaw_set              - xhat[i].pData[2])
								+LQR_Ctrl.Joint_Kgain1[0][3]*(Leg_Ctrl.Leg_set.yaw_Gyro_set         - xhat[i].pData[3])
								+LQR_Ctrl.Joint_Kgain1[0][4]*(Leg_Ctrl.Leg_set.Tilt_angle_0_set     - xhat[i].pData[4])
								+LQR_Ctrl.Joint_Kgain1[0][5]*(0                                     - xhat[i].pData[5])
								+LQR_Ctrl.Joint_Kgain1[0][6]*(Leg_Ctrl.Leg_set.Tilt_angle_0_set     - xhat[i].pData[6])
								+LQR_Ctrl.Joint_Kgain1[0][7]*(0                                     - xhat[i].pData[7])
								+LQR_Ctrl.Joint_Kgain1[0][8]*(0                                     - xhat[i].pData[8])
								+LQR_Ctrl.Joint_Kgain1[0][9]*(0                                     - xhat[i].pData[9])
								;		
								
	U[i].pData[3]=-LQR_Ctrl.Joint_Kgain1[1][0]*(Leg_Ctrl.Leg_set.Carload_Distance_set - xhat[i].pData[0])
								-LQR_Ctrl.Joint_Kgain1[1][2]*(Leg_Ctrl.Leg_set.Wheel_Speed_set      - xhat[i].pData[1])
								+LQR_Ctrl.Joint_Kgain1[1][2]*(Leg_Ctrl.Leg_set.yaw_set              - xhat[i].pData[2])
								+LQR_Ctrl.Joint_Kgain1[1][3]*(Leg_Ctrl.Leg_set.yaw_Gyro_set         - xhat[i].pData[3])
								+LQR_Ctrl.Joint_Kgain1[1][4]*(Leg_Ctrl.Leg_set.Tilt_angle_0_set     - xhat[i].pData[4])
								+LQR_Ctrl.Joint_Kgain1[1][5]*(0                                     - xhat[i].pData[5])
								+LQR_Ctrl.Joint_Kgain1[1][6]*(Leg_Ctrl.Leg_set.Tilt_angle_0_set     - xhat[i].pData[6])
								+LQR_Ctrl.Joint_Kgain1[1][7]*(0                                     - xhat[i].pData[7])
								+LQR_Ctrl.Joint_Kgain1[1][8]*(0                                     - xhat[i].pData[8])
								+LQR_Ctrl.Joint_Kgain1[1][9]*(0                                     - xhat[i].pData[9])
								;
}

void MpcClass::MPC_Forecast_resolving(float L_leg,float R_leg)
{
	MPC_dt=DWT_GetDeltaT(&MPC_DWT_Count);
	
	float X_data101[10] = {0};
	float U_data41 [10] = {0};
	mat  BU,AX;

	mat_init(&BU,10,1,   (float *)U_data41);
	mat_init(&AX,10,1,   (float *)X_data101);
	
	xhat[0].pData[0]= LQR_Ctrl.Leg_state.Distance;
	xhat[0].pData[1]= LQR_Ctrl.Leg_state.Distance_dot;
	xhat[0].pData[2]= LQR_Ctrl.Leg_state.phi;
	xhat[0].pData[3]= LQR_Ctrl.Leg_state.phi_dot;
	xhat[0].pData[4]= LQR_Ctrl.Leg_state.theta_ll;
	xhat[0].pData[5]= LQR_Ctrl.Leg_state.theta_ll_dot;
	xhat[0].pData[6]= LQR_Ctrl.Leg_state.theta_lr;
	xhat[0].pData[7]= LQR_Ctrl.Leg_state.theta_lr_dot;
	xhat[0].pData[8]= LQR_Ctrl.Leg_state.theta;
	xhat[0].pData[9]= LQR_Ctrl.Leg_state.theta_dot;
	U[0].pData[0]=LQR_Ctrl.LQR_out[0].Wheel_torque_out;
	U[0].pData[1]=LQR_Ctrl.LQR_out[1].Wheel_torque_out;
	U[0].pData[2]=LQR_Ctrl.LQR_out[0].Tp_out;
	U[0].pData[3]=LQR_Ctrl.LQR_out[1].Tp_out;

	mat_mult(&A,&xhat[0],&AX); //AX(K)
	mat_mult(&B,&   U[0],&BU); //BU(K)
	mat_add(&AX,&BU,&xhat[1]); //X(K+1)=AX(K)+BU(K)
	MPC_U_i_Feedback(1);       //¸üÐÂU(K+1)

  mat_mult(&A,&xhat[1],&AX); //AX(K+1)
	mat_mult(&B,&   U[1],&BU); //BU(K+1)
  mat_add(&AX,&BU,&xhat[2]); //X(K+2)=AX(K+1)+BU(K+1)  
	MPC_U_i_Feedback(2);       //¸üÐÂU(K+2)

  mat_mult(&A,&xhat[2],&AX);
	mat_mult(&B,&   U[2],&BU); 
  mat_add(&AX,&BU,&xhat[3]);  
	MPC_U_i_Feedback(3);     

  mat_mult(&A,&xhat[3],&AX); 
	mat_mult(&B,&   U[3],&BU); 
  mat_add(&AX,&BU,&xhat[4]); 
	MPC_U_i_Feedback(4);     

  mat_mult(&A,&xhat[4],&AX); 
	mat_mult(&B,&   U[4],&BU);
  mat_add(&AX,&BU,&xhat[5]);   
	MPC_U_i_Feedback(5);      

	MPC_forecast_X[0]=xhat[5].pData[0];
	MPC_forecast_X[1]=xhat[5].pData[1];
	MPC_forecast_X[2]=xhat[5].pData[2];
	MPC_forecast_X[3]=xhat[5].pData[3];
	MPC_forecast_X[4]=xhat[5].pData[4];
	MPC_forecast_X[5]=xhat[5].pData[5];
	MPC_forecast_X[6]=xhat[5].pData[6];
	MPC_forecast_X[7]=xhat[5].pData[7];
	MPC_forecast_X[8]=xhat[5].pData[8];
	MPC_forecast_X[9]=xhat[5].pData[9];
	
	MPC_forecast_erfa_dot[0]=(MPC_forecast_X[1]-MPC_forecast_X[3])/WHEEL_RADIUS;
	MPC_forecast_erfa_dot[1]=(MPC_forecast_X[1]+MPC_forecast_X[3])/WHEEL_RADIUS;
	
	MPC_forecast_U[0]=U[5].pData[0];
	MPC_forecast_U[1]=U[5].pData[1];
}

void MpcClass::LQR_A_B_Clear(float A_Data[10][10],float B_Data[10][4])
{
	for(uint8_t i=0;i<10;i++)
	{
		for(uint8_t j=0;j<10;j++)
		{
			A_Data[i][j]=0;
		}
	}
	
}
