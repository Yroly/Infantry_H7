#include "drv_dji_motor.h"

#include <cstdint>

RM_Motor_Class RM_Class;

void RM_Motor_Class::rm3508_read(RM3508_TypeDef *Dst,uint8_t *data,uint32_t data_len){
	Dst->mechanicalangle = (uint16_t)(data[0] << 8 | data[1]);
	Dst->speed = (int16_t)(data[2] << 8 | data[3]);
	Dst->current = (uint16_t)(data[4] << 8 | data[5]);
	Dst->temp = data[6];
	Dst->err =  static_cast<motor_err>(data[7]);
	int16_t diff = Dst->mechanicalangle - Dst->LsatAngle;
	if(diff != Dst->mechanicalangle)
			Dst->flag = 1;
	if(Dst->flag == 1){
		if (diff > 4000)
				Dst->r--;
		if (diff < -4000)
				Dst->r++;
	}
	Dst->angle = Dst->r * 8192 + Dst->mechanicalangle;
	Dst->torque = (float)(Dst->current/819.2f*0.02f); 
	Dst->Angle_DEG = Dst->angle * 0.0439453125f;
	Dst->LsatAngle = Dst->mechanicalangle;
}
void RM_Motor_Class::rm3508_ctrl(hfdcan_t *hfdcan,uint16_t id,int16_t motorl,int16_t motorr){
	uint8_t temp[8];

	temp[0] = 0;
	temp[1] = 0;
	temp[2] = motorl >> 8;
	temp[3] =	motorl;
	temp[4] = motorr >> 8;
	temp[5] = motorr;
	temp[6] = 0;
	temp[7] = 0;

	canx_send_data(hfdcan,id,temp,8);
}

