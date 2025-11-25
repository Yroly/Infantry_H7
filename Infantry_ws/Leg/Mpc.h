#ifndef __MPC_H
#define __MPC_H

#ifdef __cplusplus
extern "C" {
#endif

#include "bsp_dwt.h"
#include "algorithm_kalman.h"
#include "QuaternionEKF.h"
	
#ifdef __cplusplus
	
}
#endif


class MpcClass
{
	private:
		
	public:
	
	uint32_t MPC_DWT_Count;
  float MPC_dt;
	
	mat xhat[6],U[6],A,B;
	mat  F[5],C[5][4];
	mat U_K;
	

	
	float xhat_data[ 6][10],
		       U_data[ 6][ 4],
	         A_Data[10][10],
	         B_Data[10][ 4];
	
	float Q[10],R[4];
	

	float	MPC_forecast_X[10];
	float	MPC_forecast_U[4];
	float MPC_forecast_erfa_dot[2];
	
	void MPC_Forecast_Init(void);
  void MPC_UpData(float L_leg,float R_leg);
	void MPC_Forecast_resolving(float L_leg,float R_leg);
	void MPC_U_i_Feedback(uint8_t i);
	bool SolveLinearMpc(void);
	
	
	void LQR_A_B_Clear(float LQR_A[10][10],float LQR_B[10][4]);
	
	
  MpcClass();
	
};

extern MpcClass MPC_Ctrl;

#endif
