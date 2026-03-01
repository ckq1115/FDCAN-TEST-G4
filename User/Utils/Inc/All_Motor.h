//
// Created by CaoKangqi on 2026/2/14.
//

#ifndef G4_FRAMEWORK_ALL_MOTOR_H
#define G4_FRAMEWORK_ALL_MOTOR_H

#include "DJI_Motor.h"
#include "DM_Motor.h"

typedef struct
{
    DJI_MOTOR_Typedef DJI_6020_Pitch;
    DJI_MOTOR_Typedef DJI_6020_Yaw;

    DJI_MOTOR_Typedef DJI_3508_Shoot_L;
    DJI_MOTOR_Typedef DJI_3508_Shoot_R;
    DJI_MOTOR_Typedef DJI_3508_Shoot_M;

    DJI_MOTOR_Typedef DJI_3508_Chassis[4];

    DJI_MOTOR_Typedef DJI_3508_Chassis_1;
    DJI_MOTOR_Typedef DJI_3508_Chassis_2;
    DJI_MOTOR_Typedef DJI_3508_Chassis_3;
    DJI_MOTOR_Typedef DJI_3508_Chassis_4;
    DM_MOTOR_Typdef DM4310_Pitch;
    DM_MOTOR_Typdef DM4310_Yaw;
}MOTOR_Typdef;

typedef struct
{
    struct
    {
        float VX;
        float VY;
        float VW;
        float wheel1;
        float wheel2;
        float wheel3;
        float wheel4;
        uint8_t CAP;

    } BOTTOM;

    struct
    {
        float Pitch;
        float Pitch_MAX;//常量
        float Pitch_MIN;//常量
        float Yaw;

    } HEAD;

    struct
    {
        float SHOOT_L;
        float SHOOT_R;
        float SHOOT_M;
        float Shoot_Speed;//常量
        int64_t Single_Angle;//常量//单发角度
    } SHOOT;

    struct
    {
        int16_t YAW_INIT_ANGLE;//常量
        int16_t YAW_ANGLE;
        int16_t RELATIVE_ANGLE;
        int16_t YAW_SPEED;
        float TOP_ANGLE;
    } CG;

    struct
    {
        float Speed_err_L;
        float Speed_err_R;
        int32_t Angle;
        float Speed_Aim_L;
        float Speed_Aim_R;
        uint8_t JAM_Flag;
        uint32_t Shoot_Number;
        uint32_t Shoot_Number_Last;
    }SHOOT_Bask;

    uint8_t MOD[2];
    uint8_t ORE;
}CONTAL_Typedef;

#endif //G4_FRAMEWORK_ALL_MOTOR_H