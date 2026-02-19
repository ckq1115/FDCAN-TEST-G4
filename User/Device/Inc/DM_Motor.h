//
// Created by CaoKangqi on 2026/2/14.
//
#ifndef G4_FRAMEWORK_DM_MOTOR_H
#define G4_FRAMEWORK_DM_MOTOR_H

#include "BSP-FDCAN.h"
#include "CKQ_MATH.h"
#include "controller.h"

// --- 模式偏移地址 ---
#define MIT_MODE      0x000
#define POS_MODE      0x100
#define SPEED_MODE    0x200

// --- 映射范围参数 ---
#define P_MIN   -12.5f
#define P_MAX    12.5f
#define V_MIN   -30.0f
#define V_MAX    30.0f
#define KP_MIN   0.0f
#define KP_MAX   500.0f
#define KD_MIN   0.0f
#define KD_MAX   5.0f
#define T_MIN   -10.0f
#define T_MAX    10.0f

// --- 原始结构体完全保留 ---
typedef struct {
    int id;
    int state;
    int p_int;
    int v_int;
    int t_int;
    int kp_int;
    int kd_int;
    float pos;
    float vel;
    float tor;
    float Kp;
    float Kd;
    float Tmos;
    float Tcoil;

    int16_t Angle_last;
    int16_t Angle_now;
    int16_t Speed_last;
    int16_t Speed_now;
    int16_t acceleration;
    float current;
    int8_t temperature;
    int32_t Angle_Infinite;
    int64_t Stuck_Time;
    uint16_t Stuck_Flag[2];
    int16_t Laps;
    float Error;
    float Aim;
    float Aim_last;
    float dt;

    int8_t ONLINE_JUDGE_TIME;
    uint16_t initialAngle;
    float ralativeAngle;
    int16_t round;
    int32_t reality;
} DM_MOTOR_DATA_Typdef;

typedef struct {
    uint8_t PID_INIT;
    DM_MOTOR_DATA_Typdef DATA;
    Feedforward_t PID_F;
    PID_t PID_P;
    PID_t PID_S;
} DM_MOTOR_Typdef;

typedef enum {
    DM_CMD_MOTOR_MODE    = 0xfc,//电机使能
    DM_CMD_RESET_MODE    = 0xfd,//电机失能
    DM_CMD_ZERO_POSITION = 0xfe,//将当前编码器位置设置为零位
    DM_CMD_CLEAR_ERROR   = 0xfb,//清除错误状态
} DMMotor_Mode_e;

// 保持其他辅助结构体
typedef struct {
    uint16_t id;
    uint16_t state;
    int p_int, v_int, t_int, kp_int, kd_int;
    float tor, Kp, Kd, Tmos, Tcoil, aim, cur;
    int16_t pos[2], vel[2], round;
    int32_t reality;
    uint16_t initialAngle;
    float ralativeAngle;
} motor_fbpara_t;

typedef struct {
    float kp, ki, kd, epsilon;
    int32_t iLt, alLt;
    float error[2], p_out, i_out;
    int32_t d_out;
    float all_out;
} PID_typedef;

typedef struct {
    float A, B, rin, lastRin, perrRin;
} FFC_typedef;

typedef struct {
    uint16_t mode[2];
    motor_fbpara_t para;
    FFC_typedef FFC;
    PID_typedef PID_P;
    PID_typedef PID_S;
} Joint_Motor_t;

// 模式解析
void DM_Standard_Resolve(DM_MOTOR_Typdef *motor, uint8_t *rx_data);
void DM_1to4_Resolve(DM_MOTOR_Typdef *motor, uint8_t *rx_data);

// 控制发送
void Motor_Mode(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id, DMMotor_Mode_e what);
void MIT_Ctrl(FDCAN_HandleTypeDef* hcan, uint16_t motor_id, float pos, float vel, float kp, float kd, float torq);
void Pos_Speed_Ctrl(FDCAN_HandleTypeDef* hcan, uint16_t motor_id, float pos, float vel);
void Speed_Ctrl(FDCAN_HandleTypeDef* hcan, uint16_t motor_id, float vel);
void DM_Motor_Send(FDCAN_HandleTypeDef* hcan, uint16_t master_id, float m1_cur, float m2_cur, float m3_cur, float m4_cur);
int16_t OneFilter1(int16_t now, int16_t last, float thresholdValue);
#endif //G4_FRAMEWORK_DM_MOTOR_H