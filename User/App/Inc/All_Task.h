//
// Created by CaoKangqi on 2026/1/19.
//

#ifndef FDCAN_TEST_G4_ALL_TASK_H
#define FDCAN_TEST_G4_ALL_TASK_H

#include "All_Init.h"

#define off_line 0
#define on_Line  1

typedef struct
{
    float Vx;
    float Vy;
    float omega;
    float LF;
    float RF;
    float LB;
    float RB;
}Speed_Solve;
extern Speed_Solve Omni;

extern uint16_t adc_dma_buffer[2];
void speed_solve(void);

void MY_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
#endif //FDCAN_TEST_G4_ALL_TASK_H