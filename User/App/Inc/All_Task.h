//
// Created by CaoKangqi on 2026/1/19.
//

#ifndef G4_FRAMEWORK_ALL_TASK_H
#define G4_FRAMEWORK_ALL_TASK_H

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

// CAN接收统计数据结构
typedef struct
{
    uint32_t rx_count;          // 总接收消息数
    uint32_t fifo_full_count;   // FIFO满次数
    uint32_t msg_lost_count;    // 消息丢失次数
    uint32_t error_count;       // 读取错误次数
} CAN_Stats_t;

extern CAN_Stats_t can1_stats;
extern CAN_Stats_t can2_stats;
extern CAN_Stats_t can3_stats;

void CAN_GetStats(FDCAN_HandleTypeDef *hfdcan, CAN_Stats_t *stats);
void CAN_ResetStats(void);

extern uint16_t adc_dma_buffer[2];
void speed_solve(void);

void MY_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
#endif //G4_FRAMEWORK_ALL_TASK_H