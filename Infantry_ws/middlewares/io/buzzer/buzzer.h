#ifndef __BUZZER_H__
#define __BUZZER_H__

#include "tim.h"

namespace at{
class Buzzer{
public:
	Buzzer(TIM_HandleTypeDef* htim, uint16_t channel,float clock_hz);
	void start();
	void stop();

	void set(float hz,float duty = 0.5);
private:
	TIM_HandleTypeDef* htim_;
	uint32_t channel_;
	float clock_hz_;
};
} //namespace at

#endif // __BUZZER_H__
