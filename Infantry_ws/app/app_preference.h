#ifndef __APP_PREFERENCE_H
#define __APP_PREFERENCE_H

#define useBalance

#define useInfantry

#define CPXT

#define Nom_debug

#define VISUAL_SERIAL      Serial1_Ctrl
#define VisualData         SerialData1
#define VISUAL_SERIAL_BAUD 115200

#define JUDGE_SERIAL       Serial7_Ctrl
#define JudgeData          SerialData7
#define JUDGE_SERIAL_BAUD  115200
#define JUDGERX_BUF_NUM    255u
#define JUDGETX_BUF_NUM    255u

#define GYRO_SERIAL             Serial8_Ctrl
#define GYRO_SERIAL_BAUD        115200
#define GYRO_BUF_NUM            50
#define GYRO_SERIAL_Data_Lenth1 25

#define JUDGES_SERIAL           Serial3_Ctrl

#define DR16_SERIAL             Serial4_Ctrl
#define DbusData                SerialData4
#define DR16_SERIAL_Data_Lenth  18
#define DR16_SERIAL_BAUD        100000

#define VISUAL_SERIAL_HEADER    0xff
#define VISUAL_SERIAL_TAIL      0xfe

#define CHASSIS_SERIAL_HEADER   0xff
#define CHASSIS_SERIAL_TAIL     0xfe

//ÎÞ²ÎÊýÔòÎª NULL £¬»áÖ±½ÓÌø¹ý¡£
//LenthÓÃÓÚ¼ìÑéÊý¾Ý³¤¶È£¬Buffer_sizeÓÃÓÚ´´½¨»·ÐÎ»º³åÇøºÍÊý¾Ý»º³åÇø£¬Îª0ÔòÖÐ¶ÏÖ±½Ó·¢ËÍÍ¨Öª¸øMeesge,Buffer_sizeÓ¦´óÓÚLenth
//Serialx_ITPending ¿ÉÑ¡ USART_IT_IDLE USART_IT_RXNE USART_IT_RXNE_AND_IDLE

#define Serial_NORMAL_Mode 0
#define Serial_DMA_Mode 1

#define Serial1_Data_Header 0xFF
#define Serial1_Data_Tail   NULL
#define Serial1_Data_Lenth0 32
#define Serial1_Data_Lenth1 NULL
#define Serial1_Data_Lenth2 NULL
#define Serial1_Data_Lenth3 NULL
#define Serial1_Buffer_Size 64
#define Serial1_ITPending   USART_IT_RXNE_AND_IDLE
#define Serial1_Mode        Serial_NORMAL_Mode

#define Serial3_Data_Header 0xAA
#define Serial3_Data_Tail   NULL
#define Serial3_Data_Lenth0 10
#define Serial3_Data_Lenth1 NULL
#define Serial3_Data_Lenth2 NULL
#define Serial3_Data_Lenth3 NULL
#define Serial3_Buffer_Size 38
#define Serial3_ITPending   USART_IT_RXNE_AND_IDLE
#define Serial3_Mode        Serial_NORMAL_Mode

#define Serial4_Data_Header NULL
#define Serial4_Data_Tail   NULL
#define Serial4_Data_Lenth0 DR16_SERIAL_Data_Lenth
#define Serial4_Data_Lenth1 NULL
#define Serial4_Data_Lenth2 NULL
#define Serial4_Data_Lenth3 NULL
#define Serial4_Buffer_Size 38
#define Serial4_ITPending   USART_IT_IDLE
#define Serial4_Mode        Serial_DMA_Mode

#define Serial7_Data_Header 0xA5
#define Serial7_Data_Tail   NULL
#define Serial7_Data_Lenth0 NULL
#define Serial7_Data_Lenth1 NULL
#define Serial7_Data_Lenth2 NULL
#define Serial7_Data_Lenth3 NULL
#define Serial7_Buffer_Size JUDGERX_BUF_NUM
#define Serial7_ITPending   USART_IT_RXNE_AND_IDLE
#define Serial7_Mode        Serial_NORMAL_Mode

#define Serial8_Data_Header 0xAA
#define Serial8_Data_Tail   NULL
#define Serial8_Data_Lenth0 37
#define Serial8_Data_Lenth1 NULL
#define Serial8_Data_Lenth2 NULL
#define Serial8_Data_Lenth3 NULL
#define Serial8_Buffer_Size 76
#define Serial8_ITPending   USART_IT_RXNE_AND_IDLE
#define Serial8_Mode        Serial_NORMAL_Mode

#define FDCAN1_Buffer_Size 70
#define FDCAN2_Buffer_Size 70
#define FDCAN3_Buffer_Size 70

#define g_HENGYANG  9.7907f//ºâÑô
#define g_WUHAN     9.7936f//Îäºº
#define g_CHANGSHA  9.7915f//³¤É³


//Ñ¡ÔñÒ£¿ØÆ÷¿ØÖÆÄ£Ê½(Èýµ²Î»¹¦ÄÜÈçÏÂ0:²»¸úËæ ¸úËæ Ð¡ÍÓÂÝ 1:²»¸úËæ ÊÓ¾õ ÊÓ¾õ·¢µ¯)
#define Gimbal_RC_CONTRAL_MODE       1
#define Chassis_RC_CONTRAL_MODE      0
//²¦µ¯Á¬·¢(¿É¸ÄÐ´°´¼ü)
#define AUTOSHOOT 1
//yaw£¬pitch½Ç¶ÈÓëÒ£¿ØÆ÷ÊäÈë±ÈÀý
#define Yaw_RC_SEN   0.0003f
#define Pitch_RC_SEN 0.00015f
//yaw,pitch½Ç¶ÈºÍÊó±êÊäÈëµÄ±ÈÀý
#define Yaw_Mouse_SEN    0.00035f
#define Pitch_Mouse_SEN  0.0004f
#define L0_SET_G_SEN     0.0008f
#define L0_SET_RC_SEN    0.000275f
#define L0_SET_ROLL_SEN  0.00064f
#define Yaw_RC_Sen      -0.003f

//ÔÆÌ¨²ÎÊý------------------------------------------------------------------------------------------------------

//µ¯²ÖPWM
#define PWM_IO             PA0
#define LOADING_OPEN_DUTY  12 //1750£¬8.75-42.875
#define LOADING_CLOSE_DUTY 35//1940

//pitchÖáECDÉèÖÃ£¨×¢ÒâÊý¾Ý¿ÉÄÜ¹ýÁãµã»òmin´óÓÚmaxµÄÇé¿ö£¬»á×Ô¶¯´¦Àí£©
#define GIMBAL_PITCH_OFFSET_ECD 650
#define GIMBAL_PITCH_MAX_ECD    1150
#define GIMBAL_PITCH_MIN_ECD    123


#define GIMBAL_PITCH_MAX_ANGLE   25.5
#define GIMBAL_PITCH_MIN_ANGLE  -16



//²¦µ¯ÅÌÐý×ª·½Ïò (1,-1)
#define TRIGGER_MOTOR_REVERSE    1.0f
//²¦µ¯ÂÖÒ»È¦×Ü¹²¶àÉÙ·¢
#define TRIGGER_ONCE_SHOOT_NUM   7.0f
//²¦µ¯ÂÖÁ¬·¢ËÙ¶È£¨1Ãë¶àÉÙ·¢£©           /*ÈÈÁ¿ ÀäÈ´ */
#define TRIGGER_ONE_S_SHOOT_NUM1  6.0f //50    40
#define TRIGGER_ONE_S_SHOOT_NUM2  7.0f //85    45
#define TRIGGER_ONE_S_SHOOT_NUM3  8.0f //120   50
#define TRIGGER_ONE_S_SHOOT_NUM4  8.0f //155   55 
#define TRIGGER_ONE_S_SHOOT_NUM5  9.0f //190   60
#define TRIGGER_ONE_S_SHOOT_NUM6  10.0f//225   65
#define TRIGGER_ONE_S_SHOOT_NUM7  11.0f//260   70
#define TRIGGER_ONE_S_SHOOT_NUM8  12.0f//295   75
#define TRIGGER_ONE_S_SHOOT_NUM9  14.0f//330   80
#define TRIGGER_ONE_S_SHOOT_NUM10 16.0f//365   85


//²¦µ¯ÂÖ¼õËÙ±È
#define TRIGGER_REDUCTION_RATIO 36
//²¦µ¯ÂÖ¶Â×ªÅÐ¶ÏµçÁ÷
#define TRIGGER_BLOCKED_CURRENT 5000


/*pitch ½Ç¶È»·*/
#define PITCH_POSITION_PID_MAX_OUT     15
#define PITCH_POSITION_PID_MAX_IOUT    3
#define PITCH_POSITION_PID_Deadband	   0
#define PITCH_POSITION_PID_KP  			   0.6
#define PITCH_POSITION_PID_KI				   0.0
#define PITCH_POSITION_PID_KD				   0 
#define PITCH_POSITION_PID_I_KA			   3
#define PITCH_POSITION_PID_I_KB			   2
#define PITCH_POSITION_PID_OUT_Filter  0
#define PITCH_POSITION_PID_D_Filter	   0 
#define PITCH_POSITION_PID_ols_order	 0
#define PITCH_POSITION_PID_IP				   0x23 //0010 0011 
/*pitch ËÙ¶È»·*/
#define PITCH_SPEED_PID_MAX_OUT    8.0
#define PITCH_SPEED_PID_MAX_IOUT   2.0  
#define PITCH_SPEED_PID_Deadband	 0
#define PITCH_SPEED_PID_KP  			 0.35
#define PITCH_SPEED_PID_KI				 0
#define PITCH_SPEED_PID_KD				 0 
#define PITCH_SPEED_PID_I_KA			 3
#define PITCH_SPEED_PID_I_KB			 2
#define PITCH_SPEED_PID_OUT_Filter 0
#define PITCH_SPEED_PID_D_Filter	 0 
#define PITCH_SPEED_PID_ols_order	 0
#define PITCH_SPEED_PID_IP				 0x23  //0010 0011

/*YAW½Ç¶È»·*/
#define Yaw_POSITION_PID_MAX_OUT     15
#define Yaw_POSITION_PID_MAX_IOUT    3
#define Yaw_POSITION_PID_Deadband	   0.0
#define Yaw_POSITION_PID_KP  			   0.3
#define Yaw_POSITION_PID_KI				   0
#define Yaw_POSITION_PID_KD          0 
#define Yaw_POSITION_PID_I_KA			   3
#define Yaw_POSITION_PID_I_KB			   2
#define Yaw_POSITION_PID_OUT_Filter  0
#define Yaw_POSITION_PID_D_Filter	   0 
#define Yaw_POSITION_PID_ols_order	 0
#define Yaw_POSITION_PID_IP				   0x23 //0010 0011 
  /*YAW ËÙ¶È»·*/
#define Yaw_SPEED_PID_MAX_OUT    6.5
#define Yaw_SPEED_PID_MAX_IOUT   3.0  
#define Yaw_SPEED_PID_Deadband	 0
#define Yaw_SPEED_PID_KP  			 0.75
#define Yaw_SPEED_PID_KI				 0
#define Yaw_SPEED_PID_KD				 0 
#define Yaw_SPEED_PID_I_KA			 3
#define Yaw_SPEED_PID_I_KB			 2
#define Yaw_SPEED_PID_OUT_Filter 0
#define Yaw_SPEED_PID_D_Filter	 0 
#define Yaw_SPEED_PID_ols_order	 0
#define Yaw_SPEED_PID_IP				 0x23  //0010 0011



/*YAW ÊÓ¾õ½Ç¶È»·*/
#define Yaw_POSITION_VisualR_PID_MAX_OUT     15.0
#define Yaw_POSITION_VisualR_PID_MAX_IOUT    3.0
#define Yaw_POSITION_VisualR_PID_Deadband	   0.0
#define Yaw_POSITION_VisualR_PID_KP  			   0.30
#define Yaw_POSITION_VisualR_PID_KI				   0.15
#define Yaw_POSITION_VisualR_PID_KD          0 
#define Yaw_POSITION_VisualR_PID_I_KA			   0.3
#define Yaw_POSITION_VisualR_PID_I_KB			   0.5
#define Yaw_POSITION_VisualR_PID_OUT_Filter  0
#define Yaw_POSITION_VisualR_PID_D_Filter	   0 
#define Yaw_POSITION_VisualR_PID_ols_order	 0
#define Yaw_POSITION_VisualR_PID_IP				   0x63 //0110 0011 
/*YAWÊÓ¾õËÙ¶È»·*/
#define Yaw_SPEED_PID_VisualR_MAX_OUT       	6.5
#define Yaw_SPEED_PID_VisualR_MAX_IOUT      	3.0  
#define Yaw_SPEED_PID_VisualR_Deadband	     	0
#define Yaw_SPEED_PID_VisualR_KP  			 			0.85
#define Yaw_SPEED_PID_VisualR_KI				 			0.0
#define Yaw_SPEED_PID_VisualR_KD				 			0 
#define Yaw_SPEED_PID_VisualR_I_KA			 			3
#define Yaw_SPEED_PID_VisualR_I_KB			 			2
#define Yaw_SPEED_PID_VisualR_OUT_Filter 			0
#define Yaw_SPEED_PID_VisualR_D_Filter	 			0.0015
#define Yaw_SPEED_PID_VisualR_ols_order	 			0
#define Yaw_SPEED_PID_VisualR_IP				 			0x63  //0110 0011


/*pitchÊÓ¾õ ½Ç¶È»·*/
#define PITCH_POSITION_PID_VisualR_MAX_OUT     15
#define PITCH_POSITION_PID_VisualR_MAX_IOUT    3.0
#define PITCH_POSITION_PID_VisualR_Deadband	   0.0
#define PITCH_POSITION_PID_VisualR_KP  			   0.35
#define PITCH_POSITION_PID_VisualR_KI				   0.21
#define PITCH_POSITION_PID_VisualR_KD				   0 
#define PITCH_POSITION_PID_VisualR_I_KA			   0.3
#define PITCH_POSITION_PID_VisualR_I_KB			   0.5
#define PITCH_POSITION_PID_VisualR_OUT_Filter  0
#define PITCH_POSITION_PID_VisualR_D_Filter	   0 
#define PITCH_POSITION_PID_VisualR_ols_order	 0
#define PITCH_POSITION_PID_VisualR_IP				   0x63 //0110 0011 
/*pitchÊÓ¾õ ËÙ¶È»·*/
#define PITCH_SPEED_PID_VisualR_MAX_OUT    6.5
#define PITCH_SPEED_PID_VisualR_MAX_IOUT   2.0  
#define PITCH_SPEED_PID_VisualR_Deadband	 0
#define PITCH_SPEED_PID_VisualR_KP  			 0.55
#define PITCH_SPEED_PID_VisualR_KI				 0.0
#define PITCH_SPEED_PID_VisualR_KD				 0.0
#define PITCH_SPEED_PID_VisualR_I_KA			 3
#define PITCH_SPEED_PID_VisualR_I_KB			 2
#define PITCH_SPEED_PID_VisualR_OUT_Filter 0
#define PITCH_SPEED_PID_VisualR_D_Filter	 0.0015
#define PITCH_SPEED_PID_VisualR_ols_order	 0
#define PITCH_SPEED_PID_VisualR_IP				 0x63    //0110 0011



//²¦µ¯ÂÖµç»ú½Ç¶È»·
#define TRIGGER_ANGLE_PID_KP       10.0f
#define TRIGGER_ANGLE_PID_KI       0.0f
#define TRIGGER_ANGLE_PID_KD       10.0f
#define TRIGGER_ANGLE_PID_MAX_OUT  9000.0f
#define TRIGGER_ANGLE_PID_MAX_IOUT 5000.0f
#define TRIGGER_ANGLE_PID_BAND_I   3000.0f
//²¦µ¯ÂÖµç»úËÙ¶È»·
#define TRIGGER_SPEED_PID_KP 5.0f
#define TRIGGER_SPEED_PID_KI 0.01f
#define TRIGGER_SPEED_PID_KD 0.0f
#define TRIGGER_SPEED_PID_MAX_OUT 9000.0f
#define TRIGGER_SPEED_PID_MAX_IOUT 5000.0f
#define TRIGGER_SPEED_PID_BAND_I 3000.0f
//Ä¦²ÁÂÖµç»úËÙ¶È
#define FRIC1_SPEED_PID_KP 15.0f//10.0f
#define FRIC1_SPEED_PID_KI 0.1f
#define FRIC1_SPEED_PID_KD 0.01f
#define FRIC1_PID_MAX_OUT  15000.0f
#define FRIC1_PID_MAX_IOUT 8000.0f
#define FRIC1_PID_BAND_I   3000.0f

#define FRIC2_SPEED_PID_KP 15.0f//10.0f
#define FRIC2_SPEED_PID_KI 0.1f
#define FRIC2_SPEED_PID_KD 0.01f
#define FRIC2_PID_MAX_OUT  15000.0f
#define FRIC2_PID_MAX_IOUT 8000.0f
#define FRIC2_PID_BAND_I   3000.0f

#define FRIC3_SPEED_PID_KP 10.0f//10.0f
#define FRIC3_SPEED_PID_KI 0.1f
#define FRIC3_SPEED_PID_KD 0.01f
#define FRIC3_PID_MAX_OUT  15000.0f
#define FRIC3_PID_MAX_IOUT 8000.0f
#define FRIC3_PID_BAND_I   3000.0f

//,ÔÆÌ¨Ðý×ª¸úËæPID
#define GIMBAL_CHASSIS_FOLLOW_PID_KP -0.02f
#define GIMBAL_CHASSIS_FOLLOW_PID_KP3 -0.075//-0.02f
#define GIMBAL_CHASSIS_FOLLOW_PID_KI 0.00f
#define GIMBAL_CHASSIS_FOLLOW_PID_KD -0.5f
#define GIMBAL_CHASSIS_FOLLOW_PID_MAX_OUT 5.0f
#define GIMBAL_CHASSIS_FOLLOW_PID_MAX_IOUT 0.5f
#define GIMBAL_CHASSIS_FOLLOW_PID_BAND_I 3000.0f
//µ×ÅÌ²ÎÊý------------------------------------------------------------------------------------------------------

//ÔÆÌ¨µç»úÏà¶Ôµ×ÅÌÕý·½ÏòµÄECD
#define Gimbal_Motor_Yaw_Offset_ECD  202
#define Chassis_Motor_Yaw_Offset_ECD 2223

//Ò£¿ØÆ÷Ç°½øÒ¡¸Ë£¨max 660£©×ª»¯³É³µÌåÇ°½øËÙ¶È£¨m/s£©µÄ±ÈÀý
#define CHASSIS_VX_RC_SEN 0.0075757575757576
//Ò£¿ØÆ÷×óÓÒÒ¡¸Ë£¨max 660£©×ª»¯³É³µÌå×óÓÒËÙ¶È£¨m/s£©µÄ±ÈÀý
#define CHASSIS_VY_RC_SEN 0.003030303f//0.0015f
//¸úËæµ×ÅÌyawÄ£Ê½ÏÂ£¬Ò£¿ØÆ÷µÄyawÒ£¸Ë£¨max 660£©Ôö¼Óµ½³µÌå½Ç¶ÈµÄ±ÈÀý
#define CHASSIS_ANGLE_Z_RC_SEN 0.000002f
//²»¸úËæÔÆÌ¨µÄÊ±ºò Ò£¿ØÆ÷µÄyawÒ£¸Ë£¨max 660£©×ª»¯³É³µÌåÐý×ªËÙ¶ÈµÄ±ÈÀý
#define CHASSIS_WZ_RC_SEN 0.01f

//µÈ¼¶ËÙ¶È¶ÔÓ¦±í
#define CHASSIS_SPEED_GEAR_0 {0.5f, 1.f, 1.f}
#define CHASSIS_SPEED_GEAR_1 {1.f, 1.5f, 1.5f}
#define CHASSIS_SPEED_GEAR_2 {2.f, 2.5f, 2.f}
#define CHASSIS_SPEED_GEAR_3 {3.f, 3.5f, 2.f}

//Ò£¿ØÆ÷ËÀÇø
#define CHASSIS_RC_DEADLINE 10

#define MOTOR_SPEED_TO_CHASSIS_SPEED_VX 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VY 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_WZ 0.25f

#define MOTOR_DISTANCE_TO_CENTER 0.2f

//µ×ÅÌÈÎÎñ¿ØÖÆ¼ä¸ô 1ms
#define CHASSIS_CONTROL_TIME_MS   1
//µ×ÅÌÈÎÎñ¿ØÖÆ¼ä¸ô
#define CHASSIS_CONTROL_TIME      (0.001 * CHASSIS_CONTROL_TIME_MS)
//µ×ÅÌÈÎÎñ¿ØÖÆÆµÂÊ
#define CHASSIS_CONTROL_FREQUENCE (1000 / CHASSIS_CONTROL_TIME_MS)

//µ×ÅÌ3508×î´ócan·¢ËÍµçÁ÷Öµ
#define MAX_MOTOR_3508_CAN_CURRENT 16384.0f
//µ×ÅÌ6020×î´ócan·¢ËÍµçÑ¹Öµ
#define MAX_MOTOR_6020_CAN_CURRENT 30000.0f

//³µÂÖ°ë¾¶
#define WHEEL_RADIUS           0.062f

#define J8009_MOTOR_REDUCATION 9.0f

//m3508µç»úµÄ¼õËÙ±È
#define M3508_MOTOR_REDUCATION 15.76470588235294f


//ËÙ¶È×ª»¯ÎªÁ¦¾Ø£¬
#define CHASSIS_MOTOR_RPM_TO_TORQUE_SEN    0.000002824412704938f
//Á¦¾Ø×ª»¯ÎªËÙ¶È£¬v=w*r*dt=M*r/I*dt
#define CHASSIS_MOTOR_TORQUE_TO_VECTOR_SEN 112.0364128083924041f

//m3508×ª×Ó×ªËÙ(rpm)×ª»»ÎªÊä³öÖá½ÇËÙ¶È(rad/s)µÄ±ÈÀý
#define CHASSIS_MOTOR_RPM_TO_OMG_SEN       0.0066426710345771f

//m3508×ª×Ó×ªËÙ(rpm)×ª»¯³Éµ×ÅÌËÙ¶È(m/s)µÄ±ÈÀý
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN    CHASSIS_MOTOR_RPM_TO_OMG_SEN*WHEEL_RADIUS


//m3508×ª¾ØµçÁ÷(-16384~16384)×ªÎª³Éµç»úÊä³ö×ª¾Ø(N.m)µÄ±ÈÀý
//c=20/16384*0.246£¬0.246Îª×ª¾Ø³£Êý(N.m/A),×î´óµçÁ÷20A*0.246=×î´ó×ª¾Ø4.92N.m,È»ºó³ý×î´ó·¢ËÍµçÁ÷Öµ16384
#define CHASSIS_MOTOR_CURRENT_TO_TORQUE_SEN   0.00030029296875f //µçÁ÷Öµ³ËÕâ¸ö×ª»¯ÎªÁ¦¾Ø
#define CHASSIS_MOTOR_TORQUE_TO_CURRENT_SEN   3330.081300813008f//Á¦¾Ø  ³ËÕâ¸ö×ª»¯ÎªµçÁ÷Öµ

//µ×ÅÌµç»ú×î´óÁ¦¾Ø
#define MAX_WHEEL_TORQUE   4.92f


//µ×ÅÌ¹¦ÂÊËÙ¶È¿ØÖÆPID
#define POWER_BUFFER_PID_KP -1.5f
#define POWER_BUFFER_PID_KI 0.0f
#define POWER_BUFFER_PID_KD 0.0f
#define POWER_BUFFER_PID_MAX_OUT 30.0f
#define POWER_BUFFER_PID_MAX_IOUT 4.0f
#define POWER_BUFFER_BAND_I  3000.0f


//µ×ÅÌÐý×ª¸úËæPID 
#define CHASSIS_FOLLOW_GIMBAL_PID_KP      -8.00f
#define CHASSIS_FOLLOW_GIMBAL_PID_KI       0.00f
#define CHASSIS_FOLLOW_GIMBAL_PID_KD      -0.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT  5.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT 0.5f
#define CHASSIS_FOLLOW_GIMBAL_PID_BAND_I   3.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KP2  0.00007


#define Roll_Frame_Period   0.0003
#define L0_Frame_Period     0.0003
#define Vx_Set_Frame_Period 0.07
#define Vy_Set_Frame_Period 0.07
#define TOP_Frame_Period    0.07

/*ROLLÖá²¹³¥PID*/
#define LEG_ROLL_PID_KP       500.0f
#define LEG_ROLL_PID_KI       0.0f
#define LEG_ROLL_PID_KD       0.0f
#define LEG_ROLL_PID_MAX_OUT  400.0f
#define LEG_ROLL_PID_MAX_IOUT 20.0f
#define LEG_ROLL_PID_BAND_I   30.0f
/*ROLLÖá²¹³¥ËÙ¶ÈPID*/
#define LEG_GYRO_X_PID_KP       10.0f//5.0f
#define LEG_GYRO_X_PID_KI       0.0f
#define LEG_GYRO_X_PID_KD       0.0f
#define LEG_GYRO_X_PID_MAX_OUT  200.0f
#define LEG_GYRO_X_PID_MAX_IOUT 6.0f
#define LEG_GYRO_X_PID_BAND_I   30.0f


/*Angle0·ÀÅü²æ¿ØÖÆPID*/
#define LEG_ANGLE0_ERR_PID_KP       40.0f
#define LEG_ANGLE0_ERR_PID_KI       0.0f
#define LEG_ANGLE0_ERR_PID_KD       0.0f
#define LEG_ANGLE0_ERR_PID_MAX_OUT  500.0f
#define LEG_ANGLE0_ERR_PID_MAX_IOUT 10.0f
#define LEG_ANGLE0_ERR_PID_BAND_I   0.06f

/*ÍÈ³¤L0¿ØÖÆPID*/
#define LEG_L0_PID_KP             400//600.0f
#define LEG_L0_PID_KI             0  //0.0f
#define LEG_L0_PID_KD             0  //300
#define LEG_L0_PID_MAX_OUT        200.0f
#define LEG_L0_PID_MAX_IOUT       30.0f
#define LEG_L0_PID_BAND_I         0.04f
/*ÍÈ³¤L0¿ØÖÆPID*/
#define LEG_L1_PID_KP             400//600.0f
#define LEG_L1_PID_KI             0  //0.0f
#define LEG_L1_PID_KD             0  //300
#define LEG_L1_PID_MAX_OUT        200.0f
#define LEG_L1_PID_MAX_IOUT       30.0f
#define LEG_L1_PID_BAND_I         0.04f
/*ÍÈ³¤L0¿ØÖÆËÙ¶ÈPID*/
#define LEG_L0_SPEED_PID_KP         120.0f
#define LEG_L0_SPEED_PID_KI         0.0f
#define LEG_L0_SPEED_PID_KD         0.0f
#define LEG_L0_SPEED_PID_MAX_OUT  200.0f
#define LEG_L0_SPEED_PID_MAX_IOUT 120.0f
#define LEG_L0_SPEED_PID_BAND_I   100.0f



#ifndef useMecanum
#define useBalance
#endif
#ifdef useSteering
#undef useMecanum
#endif
#ifdef useBalance
#undef useSteering
#endif

#ifndef useInfantry
#define useInfantry
#endif
#ifdef useHero
#undef useInfantry
#endif

typedef enum
{
	FaultData = 0x00,
	CanData1,
	CanData2,
	CanData3,
	SerialData1,
	SerialData3,
	SerialData4,
	SerialData7,
	SerialData8,
	RCData,
	RefereeData,
	GyroData,
	MessageData,
	GimbalData,
	ChassisData,
	UIdrawData,
	CorrespondenceData,
	SupercapData,
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
