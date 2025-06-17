#ifndef __BSP_CAN_H__
#define __BSP_CAN_H__
#include "stm32h7xx.h"
#include "fdcan.h"

void CAN_Init(FDCAN_HandleTypeDef *hfdcan);
void can_filter_init(FDCAN_HandleTypeDef *hfdcan);
uint8_t fdcan_send(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint8_t len);
uint8_t can_receive(FDCAN_HandleTypeDef *hfdcan, uint16_t *rec_id, uint8_t *buf);

#endif
