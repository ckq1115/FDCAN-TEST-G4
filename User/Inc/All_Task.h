//
// Created by CaoKangqi on 2026/1/19.
//

#ifndef FDCAN_TEST_G4_ALL_TASK_H
#define FDCAN_TEST_G4_ALL_TASK_H

#include "main.h"

#define off_line 0
#define on_Line  1

extern uint8_t DBUS_Time;

extern uint8_t DBUS[18];

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
void speed_solve(void);

#endif //FDCAN_TEST_G4_ALL_TASK_H