#include "sbus.h"

RC_Ctrl remote;
void RC_Ctrl::init(UART_HandleTypeDef * huart, bool use_dma){
	huart_ = huart;
	use_dma_ = use_dma;
}
void RC_Ctrl::request(){
  if (use_dma_) {
    // dismiss return
    HAL_UARTEx_ReceiveToIdle_DMA(huart_, sbus_buff_, SBUS_BUFF_SIZE);
    // ref: https://github.com/HNUYueLuRM/basic_framework/blob/master/bsp/usart/bsp_usart.c
    __HAL_DMA_DISABLE_IT(huart_->hdmarx, DMA_IT_HT);
  }else {
    // dismiss return
    HAL_UARTEx_ReceiveToIdle_IT(huart_, sbus_buff_, SBUS_BUFF_SIZE);
  }
}
void RC_Ctrl::RemoteClear(){
	rc.ch[0] = 0;
	rc.ch[1] = 0;
	rc.ch[2] = 0;
	rc.ch[3] = 0;
	rc.ch[4] = 0;
	rc.s[0] = 2;
	rc.s[1] = 2;
	key.S = 0;
	Lastkey.S = 0;
	mouse.x = 0;
	mouse.y = 0;
	mouse.z = 0;
	mouse.press_r = 0;
	mouse.press_l = 0;
	mouse.last_press_r = 0;
	mouse.last_press_l = 0;
}
void RC_Ctrl::sbus_to_rc(uint16_t size, uint32_t stamp_ms){
	has_read_ = true;
	rc.ch[0] = ( sbus_buff_[0] | (sbus_buff_[1] << 8)) & 0x07ff;        //!< Channel 0
	rc.ch[1] = ((sbus_buff_[1] >> 3) | (sbus_buff_[2] << 5)) & 0x07ff; //!< Channel 1
	rc.ch[2] = ((sbus_buff_[2] >> 6) | (sbus_buff_[3] << 2) | (sbus_buff_[4] << 10)) &0x07ff;//!< Channel 2
	rc.ch[3] = ((sbus_buff_[4] >> 1) | (sbus_buff_[5] << 7)) & 0x07ff; //!< Channel 3
	rc.ch[4] = 	sbus_buff_[16] | (sbus_buff_[17] << 8);                 //NULL

	rc.s[0] = ((sbus_buff_[5] >> 4) & 0x000C) >> 2; //!< Switch left
	rc.s[1] = ((sbus_buff_[5] >> 4) & 0x0003);			//!< Switch right
	
	mouse.x = sbus_buff_[6] | (sbus_buff_[7] << 8); //!< Mouse X axis
	mouse.y = sbus_buff_[8] | (sbus_buff_[9] << 8);  //!< Mouse Y axis
	mouse.z = sbus_buff_[10] | (sbus_buff_[11] << 8);//!< Mouse Z axis

	mouse.press_l = sbus_buff_[12];//!< Mouse Left Is Press ?
	mouse.press_r = sbus_buff_[13];//!< Mouse Right Is Press ?
  
	*(uint16_t * ) & (key) = sbus_buff_[14] | sbus_buff_[15] << 8;
	mouse.last_press_l = mouse.press_l;
	mouse.last_press_r = mouse.press_r;
	Lastkey = key;
	rc.ch[0] -= RC_CH_VALUE_OFFSET;
	rc.ch[1] -= RC_CH_VALUE_OFFSET;
	rc.ch[2] -= RC_CH_VALUE_OFFSET;
	rc.ch[3] -= RC_CH_VALUE_OFFSET;
	rc.ch[4] -= RC_CH_VALUE_OFFSET;
	switch(rc.s[1]){
		case 1:
				//遥控器控制模式
				RemoteControlProcess(&(rc));
				break;
		case 3:
				//键鼠控制模式
				MouseKeyControlProcess(&mouse, key, Lastkey);
				break;
		case 2:
				STOPControlProcess();
				break;		
	}
}
void RC_Ctrl::RemoteControlProcess(Remote *RC){
	RemoteMode = REMOTE_INPUT;
	Key_ch[0] =(float )(rc.ch[0])/660;
	Key_ch[1] =(float )(rc.ch[1])/660;
	Key_ch[2] =(float )(rc.ch[2])/660;
	Key_ch[3] =(float )(rc.ch[3])/660;
	
	rc_deadline_limit(Key_ch[0],0.1f);
	rc_deadline_limit(Key_ch[1],0.1f);
	rc_deadline_limit(Key_ch[2],0.1f);
	rc_deadline_limit(Key_ch[3],0.1f);	
}
void RC_Ctrl::MouseKeyControlProcess(Mouse *mouse, Key_t key, Key_t Lastkey) {
	RemoteMode=KEY_MOUSE_INPUT;
	
	limit (mouse ->x,200,-200);
	limit (mouse ->y,200,-200);
	limit (mouse ->z,200,-200);
	
	Mouse_ch[0]=(float)(mouse ->x)/200;
	Mouse_ch[1]=(float)(mouse ->y)/200;
	Mouse_ch[2]=(float)(mouse ->z)/200;
	
	rc_deadline_limit(Mouse_ch[0],0.01f);
	rc_deadline_limit(Mouse_ch[1],0.01f);
	rc_deadline_limit(Mouse_ch[2],0.01f);
}
void RC_Ctrl::STOPControlProcess(){
	RemoteMode=STOP;
}
