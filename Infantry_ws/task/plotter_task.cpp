#include "cmsis_os.h"
#include "plotter.h"
#include "INS_Task.h"

at::Plotter plotter(&huart10);

extern "C" void plotter_task(){
	while(true){
		plotter.plot(INS.Roll,INS.Pitch,INS.Yaw);
		osDelay(10);
	}
}