//
// Created by CaoKangqi on 2026/2/14.
//

#ifndef FDCAN_TEST_G4_SYSTEM_STATUS_H
#define FDCAN_TEST_G4_SYSTEM_STATUS_H
#include <stdbool.h>
#include <stdint.h>

#include "All_Motor.h"
#include "DBUS.h"
#include "Power_CAP.h"


typedef struct
{
    /******************************遥控********************************/
    uint8_t RM_DBUS;
    /******************************运行模式********************************/
    uint8_t RM_MOD;
    /******************************头部电机********************************/
    uint8_t MOTOR_HEAD_Pitch;
    uint8_t MOTOR_HEAD_Yaw;
    /******************************发射电机********************************/
    uint8_t MOTOR_Shoot_L;
    uint8_t MOTOR_Shoot_R;
    uint8_t MOTOR_Shoot_M;
    /******************************底盘电机********************************/
    uint8_t MOTOR_Chassis_1;
    uint8_t MOTOR_Chassis_2;
    uint8_t MOTOR_Chassis_3;
    uint8_t MOTOR_Chassis_4;
    /******************************底盘电容********************************/
    uint8_t Cap;
    /******************************主控位置********************************/
    uint8_t MASTER_LOCATION;

}ROOT_STATUS_Typedef;

typedef struct {
    uint8_t r, g, b;     // 颜色 RGB
    float breathe;       // 呼吸频率
    bool buzzer;         // 是否需要蜂鸣器
} Status_t;

void System_Root(ROOT_STATUS_Typedef *Root, DBUS_Typedef *DBUS, MOTOR_Typdef *MOTOR, CAP_RXDATA *CAP_GET);
//总的状态监测
void All_Status(ROOT_STATUS_Typedef *Root, DBUS_Typedef *DBUS, MOTOR_Typdef *MOTOR, CAP_RXDATA *CAP_GET);
void LED_Show_Status(ROOT_STATUS_Typedef *Root);
void Buzzer_UpdateCycle(float activeTime, float period, uint16_t maxVolume);
void Buzzer_Start();
void Buzzer_Stop();
#endif //FDCAN_TEST_G4_SYSTEM_STATUS_H