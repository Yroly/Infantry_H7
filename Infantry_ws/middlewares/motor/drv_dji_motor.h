#pragma once

#include <cstdint>
#include "fdcan.h"
#include "bsp_can.h"

typedef struct{
	uint16_t tx_id;
	uint16_t MchanicalAngle;    //!<@brief 机械角度
	int16_t raw;
	int32_t angle;              //!<@brief 连续化机械角度
	int16_t speed;              //!<@brief 转速(转子转速RPM)
	int16_t torque;      				//!<@brief 转矩
	uint8_t temp;               //!<@brief 温度
	float Power;                //!<@brief 功率
	uint16_t LsatAngle;         //!<@brief 上一次的机械角度
	int16_t r;                  //!<@brief 圈数
	float Angle_DEG;            //!<@brief 连续化角度制角度
	uint8_t flag;               //!<@brief 统计连续机械角度标志位，用于解决一圈偏差的问题，用户使用时应忽略，不要对其进行赋值   
}RM3508_TypeDef;

class RM_Motor_Class{
public:
	void rm3508_read(RM3508_TypeDef *Dst,uint8_t *data,uint32_t data_len);
	void rm3508_ctrl(hfdcan_t *hfdcan,uint16_t id,float *data);
private:
};

extern RM_Motor_Class RM_Class;
