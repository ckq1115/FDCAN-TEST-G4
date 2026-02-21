/**
 * @file mahony_filter.h
 * @brief Mahony姿态滤波算法头文件
 * @author CaoKangqi
 * @date 2026/2/11
 */

#ifndef G4_FRAMEWORK_MAHONY_FILTER_H
#define G4_FRAMEWORK_MAHONY_FILTER_H

#include <math.h>
#include "All_define.h"
#include "All_Init.h"

typedef struct {
    float x;  // X轴
    float y;  // Y轴
    float z;  // Z轴
} Axis3f;

/**
 * @brief 快速逆平方根
 * @param x 输入值
 * @return 逆平方根近似值
 */
float invSqrt(float x);

// 前置声明
struct MAHONY_FILTER_t;

/**
 * @brief Mahony滤波器结构体
 */
struct MAHONY_FILTER_t
{
    // 输入参数
    float Kp, Ki;          // 比例/积分增益
    float dt;              // 采样间隔
    Axis3f  gyro, acc;     // 陀螺仪/加速度计数据
    Axis3f gyro_bias;      // 陀螺仪零偏
    float acc_norm;

    // 过程参数
    float exInt, eyInt, ezInt;  // 积分误差
    float q0, q1, q2, q3;       // 四元数
    float rMat[3][3];           // 旋转矩阵

    // 输出参数
    float pitch, roll, yaw;     // 俯仰/横滚/偏航角
    float last_yaw,YawTotalAngle;          // 累积偏航角
    // 函数指针
    void (*mahony_update)(struct MAHONY_FILTER_t *mahony_filter,
                          float gx, float gy, float gz,
                          float ax, float ay, float az,float dt);
    void (*mahony_output)(struct MAHONY_FILTER_t *mahony_filter);
};

extern struct MAHONY_FILTER_t mahony_filter;

void mahony_init(struct MAHONY_FILTER_t *mahony_filter, float Kp, float Ki, float dt);
void mahony_input(struct MAHONY_FILTER_t *mahony_filter, Axis3f gyro, Axis3f acc);
void mahony_update(struct MAHONY_FILTER_t *mahony_filter,
                   float gx, float gy, float gz,
                   float ax, float ay, float az,float dt);
void mahony_output(struct MAHONY_FILTER_t *mahony_filter);
void RotationMatrix_update(struct MAHONY_FILTER_t *mahony_filter);

#endif //G4_FRAMEWORK_MAHONY_FILTER_H