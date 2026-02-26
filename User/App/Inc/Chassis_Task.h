//
// Created by CaoKangqi on 2026/2/18.
//

#ifndef G4_FRAMEWORK_CHASSIS_TASK_H
#define G4_FRAMEWORK_CHASSIS_TASK_H
#include "All_Motor.h"
#include "IMU_Task.h"

extern float m;
uint8_t Chassis_Control_Init(MOTOR_Typdef *MOTOR);
void Chassis_Control_Task(MOTOR_Typdef *MOTOR);
#endif //G4_FRAMEWORK_CHASSIS_TASK_H