#pragma once 

#include "main.h"
#include "Chassis_Task.h"
#include "INS_Task.h"

#define PSB_SELECT      1
#define PSB_L3          2
#define PSB_R3          3
#define PSB_START       4
#define PSB_PAD_UP      5
#define PSB_PAD_RIGHT   6
#define PSB_PAD_DOWN    7
#define PSB_PAD_LEFT    8
#define PSB_L2          9
#define PSB_R2          10
#define PSB_L1          11
#define PSB_R1          12
#define PSB_GREEN       13
#define PSB_RED         14
#define PSB_BLUE        15
#define PSB_PINK        16

#define PSB_TRIANGLE    13
#define PSB_CIRCLE      14
#define PSB_CROSS       15
#define PSB_SQUARE      16

#define PSS_RX 5 
#define PSS_RY 6
#define PSS_LX 7
#define PSS_LY 8

typedef struct{
	int16_t key;
	int16_t last_key;
	int16_t lx;
	int16_t ly;
	int16_t rx;
	int16_t ry;
}PS2Data_t;

class PS2_Class{
public:
	PS2Data_t psData;
	uint8_t Data[9] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
	uint8_t Comd[2] = {0x01,0x42};
	uint16_t MASK[16] = {
		PSB_SELECT,
		PSB_L3,
		PSB_R3 ,
		PSB_START,
		PSB_PAD_UP,
		PSB_PAD_RIGHT,
		PSB_PAD_DOWN,
		PSB_PAD_LEFT,
		PSB_L2,
		PSB_R2,
		PSB_L1,
		PSB_R1 ,
		PSB_GREEN,
		PSB_RED,
		PSB_BLUE,
		PSB_PINK		
	};
	uint16_t Handkey;
	uint32_t PS2_TIME = 10;

	uint8_t PS2_RedLight(void);
	uint8_t PS2_AnologData(uint8_t button);
	void PS2_Vibration(uint8_t motor1, uint8_t motor2);
	void PS2_VibrationMode(void);
	
	void PS2_SetInit(void);
	void PS2_Data_Read(void);
	void PS2_Data_Process(float dt);
	
	void PS2_ShortPoll(void);
	void PS2_Cmd(uint8_t CMD);
	void PS2_EnterConfing(void);
	void PS2_TurnOnAnalogMode(void);
	void PS2_ExitConfing(void);
	
	uint8_t PS2_DataKey(void);
	void PS2_ReadData(void); 
	void PS2_ClearData(void);
	
	void Jump(void);
};
