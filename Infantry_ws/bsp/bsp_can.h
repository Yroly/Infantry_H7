#pragma once 

#include "main.h"

typedef FDCAN_HandleTypeDef hfdcan_t;

uint8_t canx_send_data(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint32_t len);

