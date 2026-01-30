//
// Created by CaoKangqi on 2026/1/19.
//

#ifndef FDCAN_TEST_G4_ALL_TASK_H
#define FDCAN_TEST_G4_ALL_TASK_H

#include <string.h>
#include "main.h"
#include "DBUS.h"
#include "Motor.h"
#include "BSP_ICM42688P.h"
#include "tim.h"
#include "usart.h"
#include "WS2812.h"
#include "controller.h"
#include "IMU_Task.h"
#include "QuaternionEKF.h"
#include "cmsis_os2.h"
#include "VOFA.h"
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
void All_Init(void);
void MY_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
#endif //FDCAN_TEST_G4_ALL_TASK_H