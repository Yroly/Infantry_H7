#include "cmsis_os.h"
#include "plotter.h"
#include "INS_Task.h"
#include "chassis_task.h"

at::Plotter plotter(&huart10);

extern "C" void plotter_task(){
	while(true){
		plotter.plot(leg.vmc[0].point.phi0,leg.stand.Stand_Angle);
		osDelay(1);
	}
}