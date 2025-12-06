#pragma once

#include "usart.h"

#define SBUS_BUFF_SIZE 36u
#define RC_FRAME_LENGTH 18u

#define RC_CH_VALUE_OFFSET      ((uint16_t)1024)
#define rc_deadline_limit(value, dealine)                  \
	{                                                      \
		if ((value) <= (dealine) && (value) >= -(dealine)) \
			value = 0;                                     \
	}
#define limit(IN, MAX, MIN) \
    if (IN < MIN)           \
        IN = MIN;           \
    if (IN > MAX)           \
        IN = MAX
/* ----------------------- Data Struct ------------------------------------- */
typedef struct{
		int16_t ch[5];//2lx 3ly 0 rx 1ry 
		char s[2];		//0 l 1 r
	}Remote;
typedef struct{
    int16_t x;              //!<@brief 鼠标x轴速度
    int16_t y;              //!<@brief 鼠标y轴速度
    int16_t z;              //!<@brief 鼠标z轴速度
    uint8_t press_l;        //!<@brief 左键状态
    uint8_t press_r;        //!<@brief 右键状态
    uint8_t last_press_l;   //!<@brief 上一次左键状态
    uint8_t last_press_r;   //!<@brief 上一次右键状态
	}Mouse;
typedef struct{
    uint16_t W: 1;
    uint16_t S: 1;
    uint16_t A: 1;
    uint16_t D: 1;
    uint16_t Shift: 1;
    uint16_t Ctrl: 1;
    uint16_t Q: 1;
    uint16_t E: 1;
    uint16_t R: 1;
    uint16_t F: 1;
    uint16_t G: 1;
    uint16_t Z: 1;
    uint16_t X: 1;
    uint16_t C: 1;
    uint16_t V: 1;
    uint16_t B: 1;
	}Key_t;
typedef enum{
    REMOTE_INPUT = 1,        //!<@brief 遥控器输入
    KEY_MOUSE_INPUT = 3,     //!<@brief 键盘输入
    STOP = 2,                //!<@brief 急停模式	
}InputMode_e;
class RC_Ctrl{
public:
	Remote rc;        //!<@brief 遥控器数据
	Mouse mouse;      //!<@brief 鼠标数据
	Key_t key;        //!<@brief 键盘数据
	Key_t Lastkey;    //!<@brief 上一帧键盘数据
	InputMode_e RemoteMode;
	/*遥控器处理值*/
	float Key_ch[4]   = {0};
	float Mouse_ch[3] = {0};
	void init(UART_HandleTypeDef * huart, bool use_dma);
	void request();
	void sbus_to_rc(uint16_t size, uint32_t stamp_ms);
	void RemoteClear();
	/* 3个遥控器数据处理函数 */
	void RemoteControlProcess(Remote *rc);
	void STOPControlProcess();
	void MouseKeyControlProcess(Mouse *mouse, Key_t key, Key_t Lastkey) ;
private:
	UART_HandleTypeDef *huart_;
	bool use_dma_;
	bool has_read_;
	uint32_t last_read_ms_;
	uint8_t sbus_buff_[SBUS_BUFF_SIZE];
};
extern RC_Ctrl remote;
