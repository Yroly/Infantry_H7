#include "WatchDog.h"
#include "remote_task.h"
WatchDog dog;

static WatchDogP List[WatchDoglength];

osStatus_t REMOTE_IfDataError( void ){
if ((remote.rc.s[0] != 1 && remote.rc.s[0] != 3 && remote.rc.s[0] != 2)
 || (remote.rc.s[1] != 1 && remote.rc.s[1] != 3 && remote.rc.s[1] != 2)
 || (remote.rc.ch[0] > 660 || remote.rc.ch[0] < -660)
 || (remote.rc.ch[1] > 660 || remote.rc.ch[1] < -660)
 || (remote.rc.ch[2] > 660 || remote.rc.ch[2] < -660)
 || (remote.rc.ch[3] > 660 || remote.rc.ch[3] < -660) )
	return osError;
else
  return osOK;
}
/*!@brief 看门狗长度*/
static uint16_t Len = 0;

void WatchDog::polling(){
	for (uint8_t i = 0; i < Len; ++i) {
		List[i]->Life++;
		if (List[i]->Life > List[i]->Max) {
			WatchBack(List[i]);
		}
	}
	osDelay(15);
}
void WatchDog::init(WatchDogP handle, uint32_t max,uint16_t life) {
	if (Len >= WatchDoglength)
			return;
	handle->Max = max;
	handle->ID  = Len + 1;
	List[Len++] = handle;
}
void WatchDog::feed(WatchDogP handle) {
	handle->Life = 0;
	FeedBack(handle);
}
void WatchDog::FeedBack(WatchDogP handle){
  switch (handle->ID){
		case 1:
			if(REMOTE_IfDataError() == osError){
				Remote_Dog.State = Device_Error;
			} else {
				Remote_Dog.State = Device_Online;
			}
		break;
  }
}
void WatchDog::WatchBack(WatchDogP handle){
  switch (handle->ID){
		case 1:
      Remote_Dog.State = Device_Offline;
    break;
  }
}
