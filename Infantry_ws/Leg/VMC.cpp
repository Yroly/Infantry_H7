#include "vmc.h"

VMC_Class VMC;

void VMC_Class::vmc_calc(float pitch, float pitch_Gyro, float dt, float Joint_motor_front_vel, float Joint_motor_rear_vel){
	kinematics_forward();
	point.d_phi0 = jacobi.j12 * Joint_motor_front_vel + jacobi.j22 * Joint_motor_rear_vel;

	theta = PI / 2.0f - pitch - point.phi0;
	d_theta = -pitch_Gyro - point.d_phi0;
	dd_theta = (d_theta - last_d_theta) / dt;
	last_d_theta = d_theta;

	point.d_L0 = jacobi.j11 * Joint_motor_front_vel + jacobi.j21 * Joint_motor_rear_vel;  

	point.dd_L0 = (point.d_L0 - point.last_d_L0) / dt;

	point.last_d_L0 = point.d_L0;
}
void VMC_Class::kinematics_forward(){
	point.xD = five_link.L5 + five_link.L4 * cos(point.phi4);
	point.yD = five_link.L4 * sin(point.phi4);
	
	point.xB = five_link.L1 * cos(point.phi1);
	point.yB = five_link.L1 * sin(point.phi1);

	point.xC = point.xB + five_link.L2 * cos(point.phi2);
	point.yC = point.yB + five_link.L2 * sin(point.phi2);

	float L_BD = sqrt(pow((point.xD - point.xB), 2) + pow((point.yD - point.yB), 2));
	float A0 = 2 * five_link.L2 * (point.xD - point.xB);
	float B0 = 2 * five_link.L2 * (point.yD - point.yB);
	float C0 = pow(five_link.L2, 2) + pow(L_BD, 2) - pow(five_link.L3, 2);
    
	point.phi2 = 2 * atan2((B0 + sqrt(pow(A0, 2) + pow(B0, 2) - pow(C0, 2))) , (A0 + C0));
	point.phi3 = atan2((point.yC - point.yD) ,(point.xC - point.xD));
	
	point.L0 = sqrt(pow((point.xC - five_link.L5/2.0f) ,2) + pow( point.yC ,2));
	point.phi0 = atan2(point.yC ,point.xC - five_link.L5/2.0f);
	
	point.alpha = pi/2.0f-point.phi0;
	
	jacobi.j11 = (five_link.L1 * sin(point.phi0 - point.phi3) * sin(point.phi1 - point.phi2))/sin(point.phi3 - point.phi2);
	jacobi.j12 = (five_link.L1 * cos(point.phi0 - point.phi3) * sin(point.phi1 - point.phi2))/(point.L0 * sin(point.phi3 - point.phi2));
	jacobi.j21 = (five_link.L4 * sin(point.phi0 - point.phi2) * sin(point.phi3 - point.phi4))/sin(point.phi3 - point.phi2);
	jacobi.j22 = (five_link.L4 * cos(point.phi0 - point.phi2) * sin(point.phi3 - point.phi4))/(point.L0 * sin(point.phi3 - point.phi2));
}
void VMC_Class::vmc_forward(){
	torque_set[0] = jacobi.j11 * F0 + jacobi.j12 * Tp;
	torque_set[1] = jacobi.j21 * F0 + jacobi.j22 * Tp;
}

void VMC_Class::vmc_reverse(){
	
}
float VMC_Class::LQR_K_Calc(float *coe,float len){
  return coe[0]*len*len*len+coe[1]*len*len+coe[2]*len+coe[3];	
}
void Limit_min_max(float *in,float min,float max){
  if(*in < min)
  {
    *in = min;
  }
  else if(*in > max)
  {
    *in = max;
  }
}
