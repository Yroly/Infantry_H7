#include "app_preference.h"

ID_Data_t ID_Data[ID_e_count];

void Prefence_Init(void)
{
	for(int i = 0;i < ID_e_count;i++)
	{
		ID_Data[i].Data_ID = ID_e(i);
	}
}

#ifndef __APP_PREFERENCE_H
#endif
