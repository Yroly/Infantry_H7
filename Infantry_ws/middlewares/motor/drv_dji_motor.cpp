#include "drv_dji_motor.h"

#include <cstdint>

RM_Motor_Class RM_Class;

void RM_Motor_Class::rm3508_read(RM3508_TypeDef *Dst,uint8_t *data,uint32_t data_len){
	Dst->MchanicalAngle = (uint16_t)(data[0] << 8 | data[1]);
	Dst->speed = (int16_t)(data[2] << 8 | data[3]);
	Dst->raw = (uint16_t)(data[4] << 8 | data[5]);
	Dst->temp = data[6];

	int16_t diff = Dst->MchanicalAngle - Dst->LsatAngle;
	if(diff != Dst->MchanicalAngle)
			Dst->flag = 1;
	if(Dst->flag == 1)
	{
			if (diff > 4000)
					Dst->r--;
			if (diff < -4000)
					Dst->r++;
	}
	Dst->angle = Dst->r * 8192 + Dst->MchanicalAngle;
	Dst->torque = (float)Dst->raw/16384*0.3; 
	Dst->Angle_DEG = Dst->angle * 0.0439453125f;
	Dst->LsatAngle = Dst->MchanicalAngle;
	Dst->tx_id = 0x200;
}
void RM_Motor_Class::rm3508_ctrl(hfdcan_t *hfdcan,uint16_t id,float *data){
	uint8_t temp[8];
	int16_t Data[4] = {0};
	for(int i=0;i<4;i++){
		Data[i] = (int16_t)data[i];
	}
	temp[0] = (uint8_t)(Data[0] >> 8);
	temp[1] = (uint8_t)(Data[0] & 0xff);
	temp[2] = (uint8_t)(Data[1] >> 8);
	temp[3] = (uint8_t)(Data[1] & 0xff);
	temp[4] = (uint8_t)(Data[2] >> 8);
	temp[5] = (uint8_t)(Data[2] & 0xff);
	temp[6] = (uint8_t)(Data[3] >> 8);
	temp[7] = (uint8_t)(Data[3] & 0xff);

	canx_send_data(hfdcan,id,temp,8);
}

