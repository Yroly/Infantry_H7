#pragma once

#include <cstdint>
#include "fdcan.h"
#include "bsp_can.h"

enum class motor_err : uint8_t{
	NO_ERR = 0,
	UN_ACCESS = 1,
	OVER_V = 2,
	NO_CON = 3,
	LOST_DATA = 4,
	OVER_TEMP = 5,//180
	FAILED = 7,
	OVER_HOT = 8//125
};
typedef struct{
	uint16_t mechangle;//!<@brief 机械角度
	int16_t rotatespeed;     //!<@brief 转速(转子转速RPM)
	int16_t current;				 //!<@brief 转矩电流
	uint8_t temp;            //!<@brief 温度
	motor_err err;					 //!<@brief 错误码
	int32_t continuemechanle;   //!<@brief 连续化机械角度
	uint16_t lastmechangle;     //!<@brief 上一次的机械角度
	float torque;      					//!<@brief 转矩
	int16_t r;                  //!<@brief 圈数
	float angle;            		//!<@brief 连续化角度
	float speed;
	int16_t give_current;
}RM3508_TypeDef;

class RM_Motor_Class{
public:
	void rm3508_read(RM3508_TypeDef *Dst,uint8_t *data,uint32_t data_len);
	void rm3508_ctrl(hfdcan_t *hfdcan,uint16_t id,int16_t motorl,int16_t motorr);
private:
};

extern RM_Motor_Class RM_Class;
