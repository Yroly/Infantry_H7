#include "bsp_can.h"
#include "dm_motor_drv.h"
#include "Chassis_Task.h"

FDCAN_RxHeaderTypeDef RxHeader1;
uint8_t g_Can1RxData[64];
FDCAN_RxHeaderTypeDef RxHeader2;
uint8_t g_Can2RxData[64];

uint8_t canx_send_data(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint32_t len)
{
	FDCAN_TxHeaderTypeDef TxHeader;
	
	TxHeader.Identifier = id;
  TxHeader.IdType =  FDCAN_STANDARD_ID ;        
  TxHeader.TxFrameType = FDCAN_DATA_FRAME;
	TxHeader.DataLength = len;

	TxHeader.ErrorStateIndicator =  FDCAN_ESI_ACTIVE;
  TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
  TxHeader.FDFormat =  FDCAN_CLASSIC_CAN;
  TxHeader.TxEventFifoControl =  FDCAN_NO_TX_EVENTS;  
  TxHeader.MessageMarker = 0;

	 HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxHeader, data);
	 return 0;
}
extern "C" void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{ 
  if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
  {
    if(hfdcan->Instance == FDCAN1)
    {
      /* Retrieve Rx messages from RX FIFO0 */
			memset(g_Can1RxData, 0, sizeof(g_Can1RxData));	//½ӊՇ°ψǥ¿Պýש	
      HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader1, g_Can1RxData);
			
			switch(RxHeader1.Identifier)
			{
        case 3 :DM_Class.DM4310_fbdata(&Chassis.Joint_Motor[0], g_Can1RxData,RxHeader1.DataLength);break;
        case 4 :DM_Class.DM4310_fbdata(&Chassis.Joint_Motor[1], g_Can1RxData,RxHeader1.DataLength);break;	         	
				case 0 :DM_Class.DM6215_fbdata(&Chassis.Wheel_Motor[0], g_Can1RxData,RxHeader1.DataLength);break;
				default: break;
			}			
	  }
  }
}

extern "C" void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
	if((RxFifo1ITs&FDCAN_IT_RX_FIFO1_NEW_MESSAGE)!=RESET)
	{
		if(hfdcan->Instance == FDCAN2)
		{
			memset(g_Can2RxData, 0, sizeof(g_Can2RxData));
			HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &RxHeader2, g_Can2RxData);
			switch(RxHeader2.Identifier){
				case 3 :DM_Class.DM4310_fbdata(&Chassis.Joint_Motor[2], g_Can2RxData,RxHeader2.DataLength);break;
				case 4 :DM_Class.DM4310_fbdata(&Chassis.Joint_Motor[3], g_Can2RxData,RxHeader2.DataLength);break;	         	
				case 0 :DM_Class.DM6215_fbdata(&Chassis.Wheel_Motor[1], g_Can2RxData,RxHeader2.DataLength);break;
				default: break;
			}
		}
	}
}