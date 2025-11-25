#include "cmsis_os2.h"
#include "buzzer.h"

// 达妙
 at::Buzzer buzzer(&htim12, TIM_CHANNEL_2, 240e6);
 
extern "C" void Buzzer_Task()
{
  buzzer.set(5000, 0.1);

  for (int i = 0; i < 3; i++) {
    buzzer.start();
    osDelay(100);
    buzzer.stop();
    osDelay(100);
  }

  while (true) {
    osDelay(100);
  }
}