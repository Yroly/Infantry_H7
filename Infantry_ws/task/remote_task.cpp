#include "cmsis_os.h"
#include "dbus.h"
#include "remote_task.h"

// 达妙
at::DBus remote(&huart5, false);

extern "C" void Remote_Task(){
  remote.request();

  while (true) {
    osDelay(1);
  }
}

extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef * huart, uint16_t Size){
  auto stamp_ms = osKernelSysTick();

  if (huart == &huart5) {
    remote.update(Size, stamp_ms);
    remote.request();
  }
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef * huart){
  if (huart == &huart5) {
    remote.request();
  }
}