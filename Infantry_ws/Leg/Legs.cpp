#include "legs.h"

leg_class leg;

leg_class::leg_class()
{
  stand.Stand_Speed = 0.0f;
  stand.L0Speed = 0.0f;
  stand.Stand_Angle = 0.0f;
  stand.Stand_Sign[0] = 0;
  stand.Stand_Sign[1] = 0;
  wheel_T[0] = 0;
  wheel_T[1] = 0;

  leg_flag.Blance_flag = false;
  leg_flag.Stand_flag = false;
  leg_flag.Revolve_flag_L = false;
  leg_flag.Revolve_flag_R = false;

  vmc[0].five_link.L1 =vmc[1].five_link.L1 = 0.105f;//大腿
  vmc[0].five_link.L2 =vmc[1].five_link.L2 = 0.125f;//小腿
  vmc[0].five_link.L3 =vmc[1].five_link.L3 = 0.105f;
  vmc[0].five_link.L4 =vmc[1].five_link.L4 = 0.125f;
  vmc[0].five_link.L5 =vmc[1].five_link.L5 = 0.0f;

  leg_set.L0_set_max = 0.57f;
  leg_set.L0_set_middle = 0.25;
  leg_set.L0_set_min = 0.23;
}

void leg_class::leg_data_clear(uint8_t count)
{
	if(count == 2)
	{
	  leg.wheel_T[0] = 0;
		leg.wheel_T[1] = 0;
		leg.vmc[0].torque_set[0] = 0;
		leg.vmc[0].torque_set[1] = 0;
		leg.vmc[1].torque_set[0] = 0;
		leg.vmc[1].torque_set[1] = 0;
		
		
		leg.vmc[0].Tp = 0;
		leg.vmc[1].Tp = 0;
		
		leg.vmc[0].F0 = 0;
		leg.vmc[1].F0 = 0;
	}
	else if(count == 0)
	{
		leg.wheel_T[0] = 0;
		
		leg.vmc[0].torque_set[0] = 0;
		leg.vmc[0].torque_set[1] = 0;
		
		leg.vmc[0].Tp = 0;
	
		leg.vmc[0].F0 = 0;
		
	}
	else if(count == 1)
	{
		leg.wheel_T[1] = 0;
		
		leg.vmc[1].torque_set[0] = 0;
		leg.vmc[1].torque_set[1] = 0;
		
		leg.vmc[1].Tp = 0;
	
		leg.vmc[1].F0 = 0;
		
	}
}
