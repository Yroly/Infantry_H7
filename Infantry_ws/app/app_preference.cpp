#include "app_preference.h"

ID_Data_t ID_Data[ID_e_count];

void Prefence_Init(void)
{
	for(int i = 0;i < ID_e_count;i++)
	{
		ID_Data[i].Data_ID = ID_e(i);
	}
}

#ifndef __APP_PREFERENCE_H
#error È±ÉÙÅäÖÃÎÄ¼þ£¬ÇëÊ¹ÓÃÏÂÃæµÄ±¸·Ý´´½¨ app_preference.h ÔÚApp/IncÏÂ
#endif
/*2023.11.14±¸·Ý
#ifndef __APP_PREFERENCE_H
#define __APP_PREFERENCE_H

//ºöÂÔ¾¯¸æ£¬°ÚÀÃ
#pragma diag_suppress 177
#pragma diag_suppress 550
#pragma diag_suppress 3337
// #pragma diag_suppress 1299

//·¢ËÍ ¸ñÊ½ Serialx_Ctrl £¨xÓÃ3£¬6£¬7£¬8´úÌæ£©£¬´®¿Ú²»ÄÜÖØ¸´£¡
//SerialDatax ÓÃÓÚÅÐ¶ÏÍâÉèÊÇ·ñÔÚÏß
#define VISUAL_SERIAL Serial1_Ctrl
#define VisualData SerialData1
#define VISUAL_SERIAL_BAUD 115200

#define CHASSIS_SERIAL Serial7_Ctrl
#define ChassisData SerialData7
#define CHASSIS_SERIAL_BAUD 115200

#define GYRO_SERIAL Serial8_Ctrl
#define GYRO_SERIAL_BAUD 115200
#define GYRO_BUF_NUM 25
#define GYRO_SERIAL_Data_Lenth 11

#define DR16_SERIAL Serial4_Ctrl
#define DbusData SerialData4
#define DR16_SERIAL_Data_Lenth 18
#define DR16_SERIAL_BAUD 100000

#define VISUAL_SERIAL_HEADER 0xff
#define VISUAL_SERIAL_TAIL 0xfe

#define CHASSIS_SERIAL_HEADER 0xff
#define CHASSIS_SERIAL_TAIL 0xfe

//ÎÞ²ÎÊýÔòÎª NULL £¬»áÖ±½ÓÌø¹ý¡£
//LenthÓÃÓÚ¼ìÑéÊý¾Ý³¤¶È£¬Buffer_sizeÓÃÓÚ´´½¨»·ÐÎ»º³åÇøºÍÊý¾Ý»º³åÇø£¬Îª0ÔòÖÐ¶ÏÖ±½Ó·¢ËÍÍ¨Öª¸øMeesge,Buffer_sizeÓ¦´óÓÚLenth
//Serialx_ITPending ¿ÉÑ¡ USART_IT_IDLE USART_IT_RXNE USART_IT_RXNE_AND_IDLE

#define Serial_NORMAL_Mode 0
#define Serial_DMA_Mode 1


#define Serial1_Data_Header 0xff
#define Serial1_Data_Tail 0xfe
#define Serial1_Data_Lenth0 10
#define Serial1_Data_Lenth1 12
#define Serial1_Data_Lenth2 NULL
#define Serial1_Data_Lenth3 NULL
#define Serial1_Buffer_Size 38
#define Serial1_ITPending USART_IT_RXNE_AND_IDLE
#define Serial1_Mode Serial_DMA_Mode

#define Serial3_Data_Header 0xff
#define Serial3_Data_Tail 0xfe
#define Serial3_Data_Lenth0 10
#define Serial3_Data_Lenth1 12
#define Serial3_Data_Lenth2 18
#define Serial3_Data_Lenth3 18
#define Serial3_Buffer_Size 38
#define Serial3_ITPending USART_IT_RXNE_AND_IDLE
#define Serial3_Mode Serial_DMA_Mode

#define Serial4_Data_Header NULL
#define Serial4_Data_Tail NULL
#define Serial4_Data_Lenth0 DR16_SERIAL_Data_Lenth
#define Serial4_Data_Lenth1 NULL
#define Serial4_Data_Lenth2 NULL
#define Serial4_Data_Lenth3 NULL
#define Serial4_Buffer_Size 38
#define Serial4_ITPending USART_IT_IDLE
#define Serial4_Mode Serial_NORMAL_Mode

#define Serial7_Data_Header 0xff
#define Serial7_Data_Tail 0xfe
#define Serial7_Data_Lenth0 10
#define Serial7_Data_Lenth1 12
#define Serial7_Data_Lenth2 NULL
#define Serial7_Data_Lenth3 NULL
#define Serial7_Buffer_Size 38
#define Serial7_ITPending USART_IT_IDLE
#define Serial7_Mode Serial_DMA_Mode

#define Serial8_Data_Header 0x55
#define Serial8_Data_Tail NULL
#define Serial8_Data_Lenth0 GYRO_SERIAL_Data_Lenth
#define Serial8_Data_Lenth1 NULL
#define Serial8_Data_Lenth2 NULL
#define Serial8_Data_Lenth3 NULL
#define Serial8_Buffer_Size GYRO_BUF_NUM
#define Serial8_ITPending USART_IT_RXNE_AND_IDLE
#define Serial8_Mode Serial_DMA_Mode

#define FDCAN1_Buffer_Size 40
#define FDCAN2_Buffer_Size 40
#define FDCAN3_Buffer_Size 40

//Ñ¡ÔñÒ£¿ØÆ÷¿ØÖÆÄ£Ê½(Èýµ²Î»¹¦ÄÜÈçÏÂ0:²»¸úËæ ¸úËæ Ð¡ÍÓÂÝ 1:²»¸úËæ ÊÓ¾õ ÊÓ¾õ·¢µ¯)
#define RC_CONTRAL_MODE 1
//²¦µ¯Á¬·¢(¿É¸ÄÐ´°´¼ü)
#define AUTOSHOOT 1
//yaw£¬pitch½Ç¶ÈÓëÒ£¿ØÆ÷ÊäÈë±ÈÀý
#define Yaw_RC_SEN 0.0003f
#define Pitch_RC_SEN 0.0001f
//yaw,pitch½Ç¶ÈºÍÊó±êÊäÈëµÄ±ÈÀý
#define Yaw_Mouse_SEN 0.0005f
#define Pitch_Mouse_SEN 0.001f

//µ¯²ÖPWM
#define PWM_IO PA0
#define LOADING_OPEN_DUTY 1860//1750
#define LOADING_CLOSE_DUTY 1694//1940

//pitchÖáECDÉèÖÃ£¨×¢ÒâÊý¾Ý¿ÉÄÜ¹ýÁãµã»òmin´óÓÚmaxµÄÇé¿ö£¬»á×Ô¶¯´¦Àí£©
#define GIMBAL_PITCH_OFFSET_ECD 2055
#define GIMBAL_PITCH_MAX_ECD 1300
#define GIMBAL_PITCH_MIN_ECD 2381

//²»Í¬µÈ¼¶µÄµ¯ËÙÉèÖÃ ±ê×¼ÅäÖÃ15 15 15 / 15 18 18 / 30 30 30
#define FRIC_GEAR_SET_1 15
#define FRIC_GEAR_SET_2 18
#define FRIC_GEAR_SET_3 30
//Ä¦²ÁÂÖµç»úËÙ¶È(¸Ã²ÎÊýÒÑÓÐ¶¯Ì¬µ÷Õû),ÔÚµ×ÅÌÍ¨ÐÅÕý³£Çé¿öÏÂ£¬´óÁ¿·¢Éäµ¯Íè¼´¿ÉµÃ³öµ±Ç°ÉäËÙ¶ÔÓ¦Ä¦²ÁÂÖ×ªËÙ
#define FRIC_SPEED_SET_15 4050
#define FRIC_SPEED_SET_18 4430
#define FRIC_SPEED_SET_30 6630

//²¦µ¯ÅÌÐý×ª·½Ïò (1,-1)
#define TRIGGER_MOTOR_REVERSE -1
//²¦µ¯ÂÖ¼õËÙ±È
#define TRIGGER_REDUCTION_RATIO 36
//²¦µ¯ÂÖÒ»È¦×Ü¹²¶àÉÙ·¢
#define TRIGGER_ONCE_SHOOT_NUM 8
//²¦µ¯ÂÖÁ¬·¢ËÙ¶È£¨1Ãë¶àÉÙ·¢£©
#define TRIGGER_ONE_S_SHOOT_NUM1 6.0f
#define TRIGGER_ONE_S_SHOOT_NUM2 8.0f
#define TRIGGER_ONE_S_SHOOT_NUM3 12.0f
//²¦µ¯ÂÖ¶Â×ªÅÐ¶ÏµçÁ÷
#define TRIGGER_BLOCKED_CURRENT 5000

//µÍÍ¨ÂË²¨
#define GIMBAL_YAW_SPEED_NUM 0.007f
#define follow_x_NUM 0.3f
#define follow_y_NUM 0.3f

//yaw ½Ç¶È»· PID²ÎÊýÒÔ¼° PID×î´óÊä³ö£¬»ý·ÖÊä³ö
#define YAW_POSITION_PID_KP 3.0f//80.0f
#define YAW_POSITION_PID_KI 0.0f//0.0f
#define YAW_POSITION_PID_KD 0.0f//3.0f
#define YAW_POSITION_PID_MAX_OUT 30000.0f
#define YAW_POSITION_PID_MAX_IOUT 5000.0f
#define YAW_POSITION_PID_BAND_I 3000.0f
//yaw ËÙ¶È»· PID²ÎÊýÒÔ¼° PID×î´óÊä³ö£¬»ý·ÖÊä³ö
#define YAW_SPEED_PID_KP 225.0f//27.0f
#define YAW_SPEED_PID_KI 2.0f//0.025f
#define YAW_SPEED_PID_KD 0.0f//0.0f
#define YAW_SPEED_PID_MAX_OUT 30000.0f
#define YAW_SPEED_PID_MAX_IOUT 4000.0f
#define YAW_SPEED_PID_BAND_I 3000.0f
//pitch ½Ç¶È»· PID²ÎÊýÒÔ¼° PID×î´óÊä³ö£¬»ý·ÖÊä³ö
#define PITCH_POSITION_PID_KP 5.0f//500.0f 100.0f
#define PITCH_POSITION_PID_KI 0.00f
#define PITCH_POSITION_PID_KD 15.0f
#define PITCH_POSITION_PID_MAX_OUT 30000.0f
#define PITCH_POSITION_PID_MAX_IOUT 5000.0f
#define PITCH_POSITION_PID_BAND_I 3000.0f
//pitch ËÙ¶È»· PID²ÎÊýÒÔ¼° PID×î´óÊä³ö£¬»ý·ÖÊä³ö
#define PITCH_SPEED_PID_KP 300.0f
#define PITCH_SPEED_PID_KI 1.0f
#define PITCH_SPEED_PID_KD 0.0f
#define PITCH_SPEED_PID_MAX_OUT 30000.0f
#define PITCH_SPEED_PID_MAX_IOUT 30000.0f
#define PITCH_SPEED_PID_BAND_I 3000.0f

//yaw ½Ç¶È»· PID²ÎÊýÒÔ¼° PID×î´óÊä³ö£¬»ý·ÖÊä³ö ÊÓ¾õ(pid)
#define YAW_FOLLOW_POSITION_PID_KP 1.5f//13.0f//5.0f//35.0f//130.0f//1.8f
#define YAW_FOLLOW_POSITION_PID_KI 0.0f//0.0f//0.0f//0.0005f
#define YAW_FOLLOW_POSITION_PID_KD 2.0f//0.0f//20.0f//30.0f//15.0f//200.0f
#define YAW_FOLLOW_POSITION_PID_MAX_OUT 30000.0f
#define YAW_FOLLOW_POSITION_PID_MAX_IOUT 5000.0f
#define YAW_FOLLOW_POSITION_PID_BAND_I 3000.0f
//yaw ËÙ¶È»· PID²ÎÊýÒÔ¼° PID×î´óÊä³ö£¬»ý·ÖÊä³ö ÊÓ¾õ(pid)
#define YAW_FOLLOW_SPEED_PID_KP 300.0f//45.0f//100.0f//15.0f//4.0f//13.0f
#define YAW_FOLLOW_SPEED_PID_KI 0.3f//0.003f//0.1f//0.1f//0.001f//0.18f
#define YAW_FOLLOW_SPEED_PID_KD 0.0f//0.0f//0.0f//0.0f
#define YAW_FOLLOW_SPEED_PID_MAX_OUT 30000.0f
#define YAW_FOLLOW_SPEED_PID_MAX_IOUT 10000.0f
#define YAW_FOLLOW_SPEED_PID_BAND_I 3000.0f
//pitch ½Ç¶È»· PID²ÎÊýÒÔ¼° PID×î´óÊä³ö£¬»ý·ÖÊä³ö ÊÓ¾õ(pid)
#define PITCH_FOLLOW_POSITION_PID_KP 3.0f//30.0f//45.0f//10.0f//5.5f//20.0f
#define PITCH_FOLLOW_POSITION_PID_KI 0.0f//0.0f//0.5f//0.0f//0.0f//0.0f
#define PITCH_FOLLOW_POSITION_PID_KD 0.0f//5.0f//0.0f//8.0f//10.0f
#define PITCH_FOLLOW_POSITION_PID_MAX_OUT 30000.0f
#define PITCH_FOLLOW_POSITION_PID_MAX_IOUT 5000.0f
#define PITCH_FOLLOW_POSITION_PID_BAND_I 3000.0f
//pitch ËÙ¶È»· PID²ÎÊýÒÔ¼° PID×î´óÊä³ö£¬»ý·ÖÊä³ö ÊÓ¾õ(pid)
#define PITCH_FOLLOW_SPEED_PID_KP 120.0f//80.0f//45.0f//200.0f//265.0f//120.0f
#define PITCH_FOLLOW_SPEED_PID_KI 0.05f//0.28f//0.5f//0.5f//4.0f//3.0f
#define PITCH_FOLLOW_SPEED_PID_KD 0.0f//0.0f//0.0f//0.0f//0.0f//0.0f
#define PITCH_FOLLOW_SPEED_PID_MAX_OUT 30000.0f
#define PITCH_FOLLOW_SPEED_PID_MAX_IOUT 20000.0f
#define PITCH_FOLLOW_SPEED_PID_BAND_I 3000.0f

//yaw ½Ç¶È»· PID²ÎÊýÒÔ¼° PID×î´óÊä³ö£¬»ý·ÖÊä³ö(ÄÜÁ¿»ú¹Ø)
#define YAW_ENERGY_POSITION_PID_KP 30.0f
#define YAW_ENERGY_POSITION_PID_KI 0.0f
#define YAW_ENERGY_POSITION_PID_KD 0.5f
#define YAW_ENERGY_POSITION_PID_MAX_OUT 30000.0f
#define YAW_ENERGY_POSITION_PID_MAX_IOUT 5000.0f
#define YAW_ENERGY_POSITION_PID_BAND_I 3000.0f
//yaw ËÙ¶È»· PID²ÎÊýÒÔ¼° PID×î´óÊä³ö£¬»ý·ÖÊä³ö(ÄÜÁ¿»ú¹Ø)
#define YAW_ENERGY_SPEED_PID_KP 5.0f
#define YAW_ENERGY_SPEED_PID_KI 0.015f
#define YAW_ENERGY_SPEED_PID_KD 0.0f
#define YAW_ENERGY_SPEED_PID_MAX_OUT 30000.0f
#define YAW_ENERGY_SPEED_PID_MAX_IOUT 10000.0f
#define YAW_ENERGY_SPEED_PID_BAND_I 3000.0f
//pitch ½Ç¶È»· PID²ÎÊýÒÔ¼° PID×î´óÊä³ö£¬»ý·ÖÊä³ö(ÄÜÁ¿»ú¹Ø)
#define PITCH_ENERGY_POSITION_PID_KP 35.0f
#define PITCH_ENERGY_POSITION_PID_KI 0.0f
#define PITCH_ENERGY_POSITION_PID_KD 5.0f
#define PITCH_ENERGY_POSITION_PID_MAX_OUT 30000.0f
#define PITCH_ENERGY_POSITION_PID_MAX_IOUT 5000.0f
#define PITCH_ENERGY_POSITION_PID_BAND_I 3000.0f
//pitch ËÙ¶È»· PID²ÎÊýÒÔ¼° PID×î´óÊä³ö£¬»ý·ÖÊä³ö(ÄÜÁ¿»ú¹Ø)
#define PITCH_ENERGY_SPEED_PID_KP 5.0f
#define PITCH_ENERGY_SPEED_PID_KI 0.02f
#define PITCH_ENERGY_SPEED_PID_KD 0.0f
#define PITCH_ENERGY_SPEED_PID_MAX_OUT 30000.0f
#define PITCH_ENERGY_SPEED_PID_MAX_IOUT 5000.0f
#define PITCH_ENERGY_SPEED_PID_BAND_I 3000.0f

//²¦µ¯ÂÖµç»ú½Ç¶È»·
#define TRIGGER_ANGLE_PID_KP 10.0f
#define TRIGGER_ANGLE_PID_KI 0.0f
#define TRIGGER_ANGLE_PID_KD 10.0f
#define TRIGGER_ANGLE_PID_MAX_OUT 9000.0f
#define TRIGGER_ANGLE_PID_MAX_IOUT 5000.0f
#define TRIGGER_ANGLE_PID_BAND_I 3000.0f
//²¦µ¯ÂÖµç»úËÙ¶È»·
#define TRIGGER_SPEED_PID_KP 10.0f
#define TRIGGER_SPEED_PID_KI 0.5f
#define TRIGGER_SPEED_PID_KD 0.0f
#define TRIGGER_SPEED_PID_MAX_OUT 9000.0f
#define TRIGGER_SPEED_PID_MAX_IOUT 5000.0f
#define TRIGGER_SPEED_PID_BAND_I 3000.0f
//Ä¦²ÁÂÖµç»úËÙ¶È
#define FRIC1_SPEED_PID_KP 10.0f//10.0f
#define FRIC1_SPEED_PID_KI 0.1f
#define FRIC1_SPEED_PID_KD 0.01f
#define FRIC1_PID_MAX_OUT 15000.0f
#define FRIC1_PID_MAX_IOUT 8000.0f
#define FRIC1_PID_BAND_I 3000.0f

#define FRIC2_SPEED_PID_KP 10.0f//10.0f
#define FRIC2_SPEED_PID_KI 0.1f
#define FRIC2_SPEED_PID_KD 0.01f
#define FRIC2_PID_MAX_OUT 15000.0f
#define FRIC2_PID_MAX_IOUT 8000.0f
#define FRIC2_PID_BAND_I 3000.0f

typedef enum
{
	FaultData = 0x00,
	CanData1,
	CanData2,
	SerialData1,
	SerialData3,
	SerialData4,
	SerialData7,
	SerialData8,
	RCData,
	GyroData,
	MessageData,
	GimbalData,
	UIdrawData,
	CorrespondenceData,
	ID_e_count
}ID_e;

typedef struct 
{
	ID_e Data_ID;
	void *Data_Ptr;
}ID_Data_t;

extern ID_Data_t ID_Data[ID_e_count];

void Prefence_Init(void);
#endif
*/
