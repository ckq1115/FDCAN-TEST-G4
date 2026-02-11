/**
 * @file       mahony_filter.c
 * @author     CaoKangqi
 * @brief      Mahony姿态滤波算法实现
 * @date       2026/2/11
 * @version    V1.0
 * @note       核心特性：
 *             1. 基于四元数的互补滤波，避免欧拉角万向锁问题
 *             2. 陀螺仪零偏自学习（静态时自动校准）
 *             3. 积分限幅防止姿态发散，偏航角低通平滑
 *             4. 快速逆平方根优化计算效率
 */
#include "mahony_filter.h"

struct MAHONY_FILTER_t mahony_filter;

/**
 * @brief 快速逆平方根函数（牛顿迭代法优化）
 * @param x 输入的浮点数
 * @return float x的逆平方根近似值
 */
float invSqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long *)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float *)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

/**
 * @brief 根据四元数更新旋转矩阵
 * @param f 指向MAHONY_FILTER_t结构体的指针，包含四元数和旋转矩阵存储区域
 * @return void
 */
void RotationMatrix_update(struct MAHONY_FILTER_t *f)
{
    float q0 = f->q0, q1 = f->q1, q2 = f->q2, q3 = f->q3;

    f->rMat[0][0] = 1 - 2*q2*q2 - 2*q3*q3;
    f->rMat[0][1] = 2*(q1*q2 - q0*q3);
    f->rMat[0][2] = 2*(q1*q3 + q0*q2);

    f->rMat[1][0] = 2*(q1*q2 + q0*q3);
    f->rMat[1][1] = 1 - 2*q1*q1 - 2*q3*q3;
    f->rMat[1][2] = 2*(q2*q3 - q0*q1);

    f->rMat[2][0] = 2*(q1*q3 - q0*q2);
    f->rMat[2][1] = 2*(q2*q3 + q0*q1);
    f->rMat[2][2] = 1 - 2*q1*q1 - 2*q2*q2;
}

/**
 * @brief Mahony滤波算法核心更新函数
 * @param f 指向MAHONY_FILTER_t结构体的指针，存储算法状态和参数
 * @param gx 陀螺仪X轴角速度（rad/s）
 * @param gy 陀螺仪Y轴角速度（rad/s）
 * @param gz 陀螺仪Z轴角速度（rad/s）
 * @param ax 加速度计X轴加速度（m/s²）
 * @param ay 加速度计Y轴加速度（m/s²）
 * @param az 加速度计Z轴加速度（m/s²）
 * @return void
 */
void mahony_update(struct MAHONY_FILTER_t *f,
                   float gx, float gy, float gz,
                   float ax, float ay, float az)
{
    float dt = f->dt;
    float halfT = 0.5f * dt;

    float acc_norm = sqrtf(ax*ax + ay*ay + az*az);
    float gyro_norm = sqrtf(gx*gx + gy*gy + gz*gz);

    int is_static = (fabsf(acc_norm - 9.81f) < 0.08f) && (gyro_norm < 0.03f);// 静态判定：加速度接近重力，陀螺仪接近静止

    if (is_static)
    {
        const float learn_rate = 0.001f;   // 零偏学习率：调大收敛更快，调小更稳定
        f->gyro_bias.z = (1 - learn_rate) * f->gyro_bias.z + learn_rate * gz;
    }
    // 去除陀螺仪零偏
    gx -= f->gyro_bias.x;
    gy -= f->gyro_bias.y;
    gz -= f->gyro_bias.z;
    // 加速度计有效性判定：模长偏离重力超过1.5m/s²或陀螺仪模长超过1.0rad/s时，认为动态过大，不校正
    int high_dynamic = (fabsf(acc_norm - 9.81f) > 1.5f) || (gyro_norm > 1.0f);

    float ex = 0, ey = 0;

    if (!high_dynamic)
    {
        // 加速度计数据归一化，转为单位向量
        float norm = invSqrt(ax*ax + ay*ay + az*az);
        ax *= norm; ay *= norm; az *= norm;

        // 计算重力方向误差（加速度计测量值与四元数推导的重力方向的叉乘）
        ex = ay * f->rMat[2][2] - az * f->rMat[2][1];
        ey = az * f->rMat[2][0] - ax * f->rMat[2][2];

        // 积分项限幅，防止积分饱和导致姿态发散
        if (gyro_norm < 0.5f)
        {
            f->exInt += f->Ki * ex * dt;
            f->eyInt += f->Ki * ey * dt;

            gx += f->Kp * ex + f->exInt;
            gy += f->Kp * ey + f->eyInt;
        }
    }

    // 四元数积分更新
    float q0 = f->q0, q1 = f->q1, q2 = f->q2, q3 = f->q3;

    f->q0 += (-q1*gx - q2*gy - q3*gz) * halfT;
    f->q1 += ( q0*gx + q2*gz - q3*gy) * halfT;
    f->q2 += ( q0*gy - q1*gz + q3*gx) * halfT;
    f->q3 += ( q0*gz + q1*gy - q2*gx) * halfT;

    float norm = invSqrt(f->q0*f->q0 + f->q1*f->q1 + f->q2*f->q2 + f->q3*f->q3);
    f->q0 *= norm; f->q1 *= norm; f->q2 *= norm; f->q3 *= norm;

    RotationMatrix_update(f);
}

/**
 * @brief 从旋转矩阵解算并输出姿态角（俯仰/横滚/偏航）
 * @param f 指向MAHONY_FILTER_t结构体的指针，存储旋转矩阵和姿态角结果
 * @return void
 */
void mahony_output(struct MAHONY_FILTER_t *f)
{
    f->pitch = -asinf(f->rMat[2][0]) * RAD2DEG;
    f->roll  = atan2f(f->rMat[2][1], f->rMat[2][2]) * RAD2DEG;
    f->yaw = atan2f(f->rMat[1][0], f->rMat[0][0]) * RAD2DEG;
}

/**
 * @brief Mahony滤波算法初始化函数
 * @param f 指向MAHONY_FILTER_t结构体的指针，待初始化的算法结构体
 * @param Kp 比例增益（姿态校正参数）
 * @param Ki 积分增益（姿态校正参数）
 * @param dt 算法更新周期（单位：秒）
 * @return void
 */
void mahony_init(struct MAHONY_FILTER_t *f, float Kp, float Ki, float dt)
{
    f->Kp = Kp;
    f->Ki = Ki;
    f->dt = dt;

    f->q0 = 1; f->q1 = 0; f->q2 = 0; f->q3 = 0;

    f->gyro_bias.x = 0;
    f->gyro_bias.y = 0;
    f->gyro_bias.z = 0;

    f->exInt = f->eyInt = 0;

    RotationMatrix_update(f);

    f->mahony_update = mahony_update;
    f->mahony_output = mahony_output;
}