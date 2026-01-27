//
// Created by CaoKangqi on 2026/1/27.
//

#ifndef FDCAN_TEST_G4_IMU_TASK_H
#define FDCAN_TEST_G4_IMU_TASK_H

#include "main.h"
#include "BSP_ICM42688P.h"
#include "TIM.h"
#include "controller.h"

typedef enum
{
    TEMP_INIT = 0,   // 温控状态初始化，初始化变量、清零 PID、启动加热相关外设
    TEMP_PREHEAT,    // 预热阶段,以固定功率加热，使温度快速接近目标值
    TEMP_PID_CTRL,   // PID 控制加热阶段
    TEMP_STABLE,     // 温度稳定状态
    GYRO_CALIB,      // 陀螺仪校准阶段
    FUSION_RUN,      // 姿态融合正常运行状态
} IMU_CTRL_STATE_e;

typedef struct
{
    uint8_t temp_reached;      // 到达目标温度
    uint8_t temp_stable;       // 温度稳定
    uint8_t gyro_calib_done;   // 陀螺仪零漂完成
    uint8_t fusion_enabled;    // 融合算法使能
} IMU_CTRL_FLAG_t;

typedef struct
{
    float gyro_correct[3];
    float gyro[3];
    float accel[3];
    float temp;

    float q[4];

    float pitch;
    float roll;
    float yaw;
    float YawTotalAngle;

    uint8_t attitude_flag;
    uint32_t correct_times;
}IMU_Data_t;
extern IMU_Data_t IMU_Data;
void Set_Heat_Power(float pwm);
void IMU_Temp_PID_Init(void);
void IMU_Temp_Control_Task(void);
void IMU_Gyro_Zero_Calibration_Task(void);
void IMU_Gyro_Calib_Initiate(void);
float Heater_PWM_Limit(float target_pwm);
#endif //FDCAN_TEST_G4_IMU_TASK_H