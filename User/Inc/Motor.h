//
// Created by CaoKangqi on 2026/1/19.
//

#ifndef FDCAN_TEST_G4_MOTOR_H
#define FDCAN_TEST_G4_MOTOR_H

#include "BSP-FDCAN.h"

typedef struct
{
    int8_t ONLINE_JUDGE_TIME;
    int16_t Angle_last; // 上一个角度值
    int16_t Angle_now;  // 现在的角度值
    int32_t round;
    int32_t conEncode;     // 处理后的连续的编码器值
    int16_t Speed_last; // 上一个速度值
    int16_t Speed_now;  // 现在的速度值
    int16_t acceleration;//加速度
    int16_t current;
    int8_t temperature;
    int32_t Angle_Infinite;
    int64_t Stuck_Time;
    uint16_t Stuck_Flag[2];
    int16_t Laps;
    float Error;
    float Aim;
    float Aim_last;
    float dt;
}DJI_MOTOR_DATA_Typedef;

typedef struct
{
    uint8_t PID_INIT;
    DJI_MOTOR_DATA_Typedef DATA;
    /*Feedforward_t PID_F;
    PID_t PID_P;
    PID_t PID_S;*/
}DJI_MOTOR_Typedef;

typedef struct
{
    DJI_MOTOR_Typedef  GM6020_1;
    DJI_MOTOR_Typedef  M3508_1;
    DJI_MOTOR_Typedef  M3508_2;
    DJI_MOTOR_Typedef  M3508_3;
    DJI_MOTOR_Typedef  M3508_4;
} All_Motor_TypeDef;

typedef struct{
    float shunt_volt;
    float bus_volt;
    float current;
    float power;
}Power_Typedef;

typedef struct{
    Power_Typedef P1;
    Power_Typedef P2;
    Power_Typedef P3;
    Power_Typedef P4;
    Power_Typedef P5;
}ALL_POWER_RX;

extern ALL_POWER_RX All_Power;
extern All_Motor_TypeDef All_Motor;

void MOTOR_CAN_RX_3508RM(DJI_MOTOR_DATA_Typedef* DATA , uint8_t* can_data);
void MOTOR_CAN_RX_2006RM(DJI_MOTOR_DATA_Typedef* MOTOR , uint8_t* can_data);
void MOTOR_CAN_RX_6020RM(DJI_MOTOR_DATA_Typedef* DATA , uint8_t* can_data);
void HEAD_MOTOR_CLEAR(DJI_MOTOR_Typedef* MOTOR , uint8_t mode);
void HEAD_MOTOR2006_STUCK(DJI_MOTOR_Typedef* MOTOR , float ERROR_ANGLE , float ERROR_SPEED , uint16_t ERROR_TIME);
//void RUI_F_HEAD_MOTOR3508_STUCK(DJI_MOTOR_Typedef* MOTOR , float ERROR_CURRENT , float ERROR_SPEED);
void DJI_Current_Ctrl(hcan_t* hcan, uint16_t stdid, int16_t num1, int16_t num2, int16_t num3, int16_t num4);
void MotorRoundResolve(DJI_MOTOR_Typedef *motor);
void DJI_Motor_Init(void);
int16_t Motor_PID_Control(DJI_MOTOR_Typedef *motor, float target);

void CAN_POWER_Rx(Power_Typedef* pPower, uint8_t *rx_data);

#endif //FDCAN_TEST_G4_MOTOR_H