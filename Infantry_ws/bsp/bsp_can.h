#ifndef __BSP_CAN_H
#define __BSP_CAN_H

#include "fdcan.h"
#include "stm32h7xx_hal.h"
#include "string.h"

typedef FDCAN_HandleTypeDef hcan_t;

uint8_t canx_send_data(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint32_t len);

#endif 
