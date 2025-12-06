#include "remote_task.h"

extern "C" void Remote_Task(){
	static portTickType currentTime;
	remote.init(&huart5,false);
	dog.init(&dog.Remote_Dog,20,0);
	remote.request();
	for(;;){
		currentTime = xTaskGetTickCount();
		if(dog.Remote_Dog.State != Device_Online){
		}else{
			remote.RemoteClear();
		}
		dog.polling();
		osDelay(15);
	}
}
extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef * huart, uint16_t Size){
	auto stamp_ms = xTaskGetTickCount();
  if (huart == &huart5) {
		remote.sbus_to_rc(Size,stamp_ms);
		dog.feed(&dog.Remote_Dog);
    remote.request();
  }
}
extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef * huart){
  if (huart == &huart5) {
    remote.request();
  }
}