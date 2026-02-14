//
// Created by CaoKangqi on 2026/1/5.
//

#ifndef FDCAN_TEST_G4_BSP_FDCAN_H
#define FDCAN_TEST_G4_BSP_FDCAN_H

#include "fdcan.h"

typedef void (*CAN_Handler_Func)(void* instance, uint8_t* data);

typedef struct {
    uint32_t id;                // CAN ID
    void* instance;             // 对应电机或电源实例的地址
    CAN_Handler_Func handler;   // 解析回调函数
} CAN_Rx_Entry;

typedef FDCAN_HandleTypeDef hcan_t;
void FDCAN_Config(FDCAN_HandleTypeDef *hfdcan, uint32_t fifo);
extern uint8_t FDCAN_Send_Msg(FDCAN_HandleTypeDef *hfdcan, uint32_t id, uint8_t *data, uint32_t len);

#endif //FDCAN_TEST_G4_BSP_FDCAN_H