//
// Created by CaoKangqi on 2026/1/5.
//

#ifndef G4_FRAMEWORK_BSP_FDCAN_H
#define G4_FRAMEWORK_BSP_FDCAN_H

#include "fdcan.h"

typedef FDCAN_HandleTypeDef hcan_t;
void FDCAN_Config(FDCAN_HandleTypeDef *hfdcan, uint32_t fifo);
extern uint8_t FDCAN_Send_Msg(FDCAN_HandleTypeDef *hfdcan, uint32_t id, uint8_t *data, uint32_t len);

#endif //G4_FRAMEWORK_BSP_FDCAN_H