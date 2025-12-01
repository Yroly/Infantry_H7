#include "PS2_Task.h"
PS2_Class PS2;

extern "C" void PS2_Task(){
	PS2.PS2_SetInit();
	for(;;){
		if(PS2.Data[1] != 0x73) PS2.PS2_SetInit();
		
		PS2.PS2_Data_Read();
		PS2.PS2_Data_Process((float)PS2.PS2_TIME/1000.0f);
		osDelay(PS2.PS2_TIME);
	}
}
void PS2_Class::PS2_SetInit(void){
	PS2_ShortPoll();
	PS2_EnterConfing();
	PS2_TurnOnAnalogMode();
	PS2_ExitConfing();
}
void PS2_Class::PS2_Data_Read(void){
	psData.key = PS2_DataKey();
	psData.lx = PS2_AnologData(PSS_LX);
	psData.ly = PS2_AnologData(PSS_LY);
	psData.rx = PS2_AnologData(PSS_RX);
	psData.ry = PS2_AnologData(PSS_RY);
	if((psData.ry<=255&&psData.ry>192)||(psData.ry<64&&psData.ry>=0))
	{
	  psData.rx=127;
	}
	if((psData.rx<=255&&psData.rx>192)||(psData.rx<64&&psData.rx>=0))
	{
	  psData.ry=128;
	}
}
void PS2_Class::PS2_Data_Process(float dt){
	if(psData.last_key != 4&&psData.key == 4&&Chassis.start_flag == 0) 
	{
		Chassis.start_flag = 1;
		if(Chassis.recover_flag == 0
			&&((Chassis.myPithR<((-3.1415926f)/4.0f)&&Chassis.myPithR>((-3.1415926f)/2.0f))
		  || (Chassis.myPithR> ( 3.1415926f /4.0f)&&Chassis.myPithR< ( 3.1415926f /2.0f))))
		{
		  Chassis.recover_flag = 1;
		}
	}
	else if(psData.last_key != 4&&psData.key == 4&&Chassis.start_flag == 1) 
	{
		Chassis.start_flag = 0;
		Chassis.recover_flag = 0;
	}
	
	psData.last_key=psData.key;
  
	if(Chassis.start_flag == 1)
	{
		Chassis.v_target=((float)(psData.ry - 128))*(-0.008f);
		Chassis.slope_following(&Chassis.v_target,&Chassis.v_set,0.005f);

		Chassis.x_set=Chassis.x_set+Chassis.v_set*dt;
		Chassis.turn_set=Chassis.turn_set+(psData.lx - 127)*(-0.00025f);

		Chassis.leg_set=Chassis.leg_set+((float)(psData.ly - 128))*(-0.000015f);
		Chassis.roll_target= ((float)(psData.rx - 127))*(0.0025f);

		Chassis.slope_following(&Chassis.roll_target,&Chassis.roll_set,0.0075f);

		Jump();

		Limit_min_max(&Chassis.leg_set,0.065f,0.18f);

		if(fabsf(Chassis.last_leg_set-Chassis.leg_set)>0.0001f || fabsf(Chassis.last_leg_right_set-Chassis.leg_right_set)>0.0001f)
		{
			VMC.leg_flag  = 1;
		}
		Chassis.last_leg_set=Chassis.leg_set;
	}
	else if(Chassis.start_flag == 0)
	{
		Chassis.v_set = 0.0f;
		Chassis.x_set = Chassis.x_filter;
		Chassis.turn_set = Chassis.total_yaw;
		Chassis.leg_set = 0.08f;
	}	
}

void PS2_Class::Jump(void){
	if(psData.key == 12)
	{
		if(++Chassis.count_key>10)
		{
			if(Chassis.jump_flag == 0)
			{
				Chassis.jump_flag = 1;
				Chassis.jump_leg = Chassis.leg_set;
			}
		}
	}
	else
	{
		Chassis.count_key = 0;
	}	
}

void PS2_Class::PS2_ShortPoll(void){
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_9,GPIO_PIN_RESET);
	DWT_Delay(0.000016f);
	PS2_Cmd(0x01);  
	PS2_Cmd(0x42);  
	PS2_Cmd(0X00);
	PS2_Cmd(0x00);
	PS2_Cmd(0x00);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_SET);
	DWT_Delay(0.000016f);	
}

void PS2_Class::PS2_EnterConfing(void){
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);//CS_L
	DWT_Delay(0.000016f); 
	PS2_Cmd(0x01);  
	PS2_Cmd(0x43);  
	PS2_Cmd(0X00);
	PS2_Cmd(0x01);
	PS2_Cmd(0x00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_SET);//CS_H
	DWT_Delay(0.000016f); 	
}

void PS2_Class::PS2_VibrationMode(void){
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);//CS_L
	DWT_Delay(0.000016f); 
	PS2_Cmd(0x01);  
	PS2_Cmd(0x4D);  
	PS2_Cmd(0X00);
	PS2_Cmd(0x00);
	PS2_Cmd(0X01);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_SET);//CS_H
	DWT_Delay(0.000016f); 
}

void PS2_Class::PS2_ExitConfing(void){
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);//CS_L
	DWT_Delay(0.000016f); 
	PS2_Cmd(0x01);  
	PS2_Cmd(0x43);  
	PS2_Cmd(0X00);
	PS2_Cmd(0x00);
	PS2_Cmd(0x5A);
	PS2_Cmd(0x5A);
	PS2_Cmd(0x5A);
	PS2_Cmd(0x5A);
	PS2_Cmd(0x5A);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_SET);//CS_H
	DWT_Delay(0.000016f); 
}
void PS2_Class::PS2_TurnOnAnalogMode(void){
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);//CS_L
	PS2_Cmd(0x01);  
	PS2_Cmd(0x44);  
	PS2_Cmd(0X00);
	PS2_Cmd(0x01);
	PS2_Cmd(0x03);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_SET);//CS_H
	DWT_Delay(0.000016f); 	
}

void PS2_Class::PS2_Cmd(uint8_t CMD){
	volatile uint16_t ref=0x01;
	Data[1] = 0;
	for(ref=0x01;ref<0x0100;ref<<=1)
	{
		if(ref&CMD)
		{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
		}
		else HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);

		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);
		DWT_Delay(0.000005f);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_SET);
		if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0))	Data[1] = ref|Data[1];
	}
	DWT_Delay(0.000016f); 	
}

uint8_t PS2_Class::PS2_DataKey(void){
	uint8_t index;
	
	PS2_ClearData();
	PS2_ReadData();
	
	Handkey=(Data[4]<<8)|Data[3];
	for(index=0;index<16;index++)
	{	    
		if((Handkey&(1<<(MASK[index]-1)))==0)
		return index+1;
	}
	return 0;
}

void PS2_Class::PS2_ReadData(void){
	volatile uint8_t byte=0;
	volatile uint16_t ref=0x01;
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);
	PS2_Cmd(Comd[0]);
	PS2_Cmd(Comd[1]);
	for(byte = 2;byte < 9;byte ++){
		for(ref=0x01;ref<0x100;ref<<=1){
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_SET);
			DWT_Delay(0.000005f);
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);
			DWT_Delay(0.000005f);
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_SET);
		      if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0))
		      Data[byte] = ref|Data[byte];
		}
        DWT_Delay(0.000016f); 
	}
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_SET);
}

void PS2_Class::PS2_ClearData(void){
	uint8_t a;
	for(a=0;a<9;a++)
		Data[a]=0x00;	
}

uint8_t PS2_Class::PS2_AnologData(uint8_t button)
{
	return Data[button];
}
