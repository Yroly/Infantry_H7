#ifndef __dm_motor_drv_H__
#define __dm_motor_drv_H__

#include "main.h"
#include "fdcan.h"
#include "bsp_can.h"

#ifdef __cplusplus
extern "C"{
#endif

#define MIT_MODE 			0x000
#define POS_MODE			0x100
#define SPEED_MODE		0x200

#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -30.0f
#define V_MAX 30.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -10.0f
#define T_MAX 10.0f

#define P_MIN2 -12.0f
#define P_MAX2 12.0f
#define V_MIN2 -45.0f
#define V_MAX2 45.0f
#define KP_MIN2 0.0f
#define KP_MAX2 500.0f
#define KD_MIN2 0.0f
#define KD_MAX2 5.0f
#define T_MIN2 -18.0f
#define T_MAX2 18.0f
typedef  union  
{
	float fdata;
	int   Idata;
}int16_float;
typedef struct 
{
	uint16_t id;
	uint16_t state;
	int p_int;
	int v_int;
	int t_int;
	int kp_int;
	int kd_int;
	int16_float POS;
	int16_float VEL;
	int16_float Torque;
	float Kp;
	float Kd;
	float Tmos;
	float Tcoil;
}DM_motor_fbpara_t;


typedef struct
{
	DM_motor_fbpara_t para;
	uint16_t mode;
}Joint_Motor_t;

typedef struct{
	DM_motor_fbpara_t para;
	uint16_t mode;
	float angle;
	float speed;
	float wheel_T;
}Wheel_Motor_t;

class DM_Motor_Class{
public:
void DM4310_fbdata(Joint_Motor_t *motor, uint8_t *rx_data,uint32_t data_len);
void DM6215_fbdata(Wheel_Motor_t *motor, uint8_t *rx_data,uint32_t data_len);

void enable_motor_mode(hfdcan_t* hfdcan, uint16_t motor_id, uint16_t mode_id);
void disable_motor_mode(hfdcan_t* hfdcan, uint16_t motor_id, uint16_t mode_id);

void mit_ctrl(hfdcan_t* hfdcan, uint16_t motor_id, float pos, float vel,float kp, float kd, float torq);
void pos_speed_ctrl(hfdcan_t* hfdcan,uint16_t motor_id, float pos, float vel);
void speed_ctrl(hfdcan_t* hfdcan,uint16_t motor_id, float _vel);

void mit_ctrl2(hfdcan_t* hfdcan, uint16_t motor_id, float pos, float vel,float kp, float kd, float torq);

void Joint_Motor_Init(Joint_Motor_t *motor,uint16_t id,uint16_t mode);
void Wheel_Motor_Init(Wheel_Motor_t *motor,uint16_t id,uint16_t mode);
	
float Hex_To_Float(uint32_t *Byte,int num);
uint32_t FloatTohex(float HEX);

float uint_to_float(int x_int, float x_min, float x_max, int bits);
int float_to_uint(float x_float, float x_min, float x_max, int bits);		
};

extern void enable_motor_mode(hfdcan_t* hfdcan, uint16_t motor_id, uint16_t mode_id);
extern DM_Motor_Class DM_Class;

#ifdef __cplusplus
}
#endif

#endif /* __dm_motor_drv_H__ */
