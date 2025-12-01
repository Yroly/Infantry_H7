#pragma once

#include "vmc.h"

typedef struct 
{
	/*旋转速度 收腿速度 角度*/
	float Stand_Speed;
	float L0Speed;
	float Stand_Angle;

	int8_t Stand_Sign[2];

}Stand_t;

typedef struct
{
	float L0_set_max;
	float L0_set_middle;
	float L0_set_min;
	float L0_set;

	float L0_set_final;

} leg_set_t;

typedef struct
{
	bool Blance_flag;
	bool Revolve_flag_L;
	bool Revolve_flag_R;

	
	uint8_t Stand_flag;
	
	
}leg_flag_t;

class leg_class
{
private:
public:

	Stand_t stand;

	float stand_delay_time;

	float wheel_T[2];

	leg_flag_t leg_flag;
	VMC_Class vmc[2];
	leg_set_t leg_set;
	
	void leg_data_clear(uint8_t count);

	leg_class();
};
extern leg_class leg;
