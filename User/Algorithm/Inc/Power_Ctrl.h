//
// Created by CaoKangqi on 2026/2/23.
//

#ifndef G4_FRAMEWORK_POWER_CTRL_H
#define G4_FRAMEWORK_POWER_CTRL_H

#include "All_Motor.h"
#include "main.h"
#include "user_lib.h"
#include "Power_CAP.h"
#include "Referee.h"

/* 宏定义 */
#define CHASSIS_MOTOR_NUM 4
#define MIN_DISCRIMINANT 1e-6f
#define MAX_CURRENT_OUTPUT 16000

/* 采样相关配置 */
#define SAMPLE_TIMEOUT_CNT 10      // 采样超时计数（500Hz时，10个周期即20ms无更新视为掉线）
#define POWER_SAMPLE_ALPHA 0.1f    // 低通滤波系数（0~1，越小越平滑）
#define MODEL_CALIB_GAIN 0.005f    // 模型校准增益（非常小，慢环校正）

/* 功率控制状态枚举 */
typedef enum {
    POWER_CTRL_MODE_MODEL_ONLY,     // 纯模型模式（兜底）
    POWER_CTRL_MODE_FUSION,         // 融合模式（采样正常）
} Power_Ctrl_Mode_e;

/* 母线采样功率管理结构体 */
typedef struct {
    float raw_power;                // 原始采样功率
    float filtered_power;           // 滤波后的功率
    float model_total_power;        // 模型预测的总功率
    float power_error;              // 采样与模型的误差

    uint32_t last_update_tick;      // 上次更新的时间戳（或计数器）
    uint8_t is_offline;             // 离线标志位
    Power_Ctrl_Mode_e mode;         // 当前控制模式
} Power_Sample_Manager_t;

/* 缓冲能量PID结构体 */
typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float ILt;
    float AlLt;
    float Error[2];
    float P_out;
    float I_out;
    float D_out;
    float All_out;
} PID_buffer_t;

/* 功率模型结构体 */
typedef struct {
    PID_buffer_t PID_Buffer;
    float scaled_give_power[CHASSIS_MOTOR_NUM];

    float k1;
    float k2;
    float k3;
    float k4; // 这个系数会在融合模式下被动态校准

    /* 采样管理器 */
    Power_Sample_Manager_t sample_mgr;
} model_t;

/* 你的采样结构体（保持原样） */
typedef struct {
    float shunt_volt;
    float bus_volt;
    float current;
    float power;
} Power_Typedef;

typedef struct {
    Power_Typedef P1, P2, P3, P4, P5;
} ALL_POWER_RX;

/* 外部接口声明 */
extern ALL_POWER_RX All_Power;
extern float initial_give_power[CHASSIS_MOTOR_NUM];
/* 函数声明 */
void Power_control_init(model_t *model);
void Power_Sample_Update(model_t *model, float raw_sample_power); // 采样更新函数（需在500Hz回调中调用）
uint8_t chassis_power_control(CONTAL_Typedef *RUI_V_CONTAL_V,
                           User_Data_T *usr_data,
                           model_t *model,
                           CAP_RXDATA *CAP_GET,
                           MOTOR_Typdef *MOTOR);

void CAN_POWER_Rx(Power_Typedef* pPower, uint8_t *rx_data);

#endif //G4_FRAMEWORK_POWER_CTRL_H