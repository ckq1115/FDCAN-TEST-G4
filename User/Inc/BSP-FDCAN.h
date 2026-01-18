//
// Created by CaoKangqi on 2026/1/5.
//

#ifndef FDCAN_TEST_G4_BSP_FDCAN_H
#define FDCAN_TEST_G4_BSP_FDCAN_H

#include "fdcan.h"

typedef FDCAN_HandleTypeDef hcan_t;
extern void FDCAN1_Config(void);
extern void FDCAN2_Config(void);
extern void FDCAN3_Config(void);
extern uint8_t canx_send_data(FDCAN_HandleTypeDef *hcan, uint16_t id, uint8_t *data, uint32_t len);
uint8_t GimbalRXResolve(uint8_t * buff,uint16_t CANID) ;

#endif //FDCAN_TEST_G4_BSP_FDCAN_H