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

#include "arm_math.h"

CCM_DATA struct MAHONY_FILTER_t mahony_filter;

/**
 * @brief 优化的逆平方根：在支持FPU的ARM上，硬件除法和开方极快
 */
static CCM_FUNC float32_t arm_invSqrt(float32_t x) {
    float32_t out;
    arm_status status = arm_sqrt_f32(x, &out);
    if (status != ARM_MATH_SUCCESS || out == 0.0f) return 0.0f;
    return 1.0f / out;
}

/**
 * @brief 根据四元数更新旋转矩阵
 * @param f 指向MAHONY_FILTER_t结构体的指针，包含四元数和旋转矩阵存储区域
 * @return void
 */
CCM_FUNC void RotationMatrix_update(struct MAHONY_FILTER_t *f)
{
    float32_t q0 = f->q0, q1 = f->q1, q2 = f->q2, q3 = f->q3;

    // 提前计算平方项，减少重复乘法
    float32_t q1q1 = q1 * q1;
    float32_t q2q2 = q2 * q2;
    float32_t q3q3 = q3 * q3;
    float32_t q0q1 = q0 * q1;
    float32_t q0q2 = q0 * q2;
    float32_t q0q3 = q0 * q3;
    float32_t q1q2 = q1 * q2;
    float32_t q1q3 = q1 * q3;
    float32_t q2q3 = q2 * q3;

    f->rMat[0][0] = 1.0f - 2.0f * (q2q2 + q3q3);
    f->rMat[0][1] = 2.0f * (q1q2 - q0q3);
    f->rMat[0][2] = 2.0f * (q1q3 + q0q2);

    f->rMat[1][0] = 2.0f * (q1q2 + q0q3);
    f->rMat[1][1] = 1.0f - 2.0f * (q1q1 + q3q3);
    f->rMat[1][2] = 2.0f * (q2q3 - q0q1);

    f->rMat[2][0] = 2.0f * (q1q3 - q0q2);
    f->rMat[2][1] = 2.0f * (q2q3 + q0q1);
    f->rMat[2][2] = 1.0f - 2.0f * (q1q1 + q2q2);
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
CCM_FUNC void mahony_update(struct MAHONY_FILTER_t *f,
                   float gx, float gy, float gz,
                   float ax, float ay, float az)
{
    gx *= DEG2RAD;
    gy *= DEG2RAD;
    gz *= DEG2RAD;
    float dt = f->dt;
    float halfT = 0.5f * dt;

    float32_t sum_sq;
    sum_sq = ax*ax + ay*ay + az*az;
    arm_sqrt_f32(sum_sq, &f->acc_norm);

    sum_sq = gx*gx + gy*gy + gz*gz; // 注意这里gx等已经是rad/s
    float32_t gyro_norm;
    arm_sqrt_f32(sum_sq, &gyro_norm);
    // 静态判定：加速度接近重力，陀螺仪接近静止，两个数都不建议大于0.1
    int is_static = (fabsf(f->acc_norm - 9.81f) < 0.09f) && (gyro_norm < 0.015f);

    if (is_static)
    {
        const float learn_rate = 0.005f;   // 零偏学习率：调大收敛更快，调小更稳定
        f->gyro_bias.x = (1 - learn_rate) * f->gyro_bias.x + learn_rate * gx;
        f->gyro_bias.y = (1 - learn_rate) * f->gyro_bias.y + learn_rate * gy;
        f->gyro_bias.z = (1 - learn_rate) * f->gyro_bias.z + learn_rate * gz;
    }
    // 去除陀螺仪零偏
    gx -= f->gyro_bias.x;
    gy -= f->gyro_bias.y;
    gz -= f->gyro_bias.z;

    // 加速度计有效性判定：模长偏离重力超过1.5m/s²或陀螺仪模长超过1.0rad/s时，认为动态过大，不校正
    int high_dynamic = (fabsf(f->acc_norm - 9.81f) > 1.5f) || (gyro_norm > 1.0f);

    float ex = 0, ey = 0;

    if (!high_dynamic)
    {
        // 加速度计数据归一化，转为单位向量
        float32_t norm = arm_invSqrt(ax*ax + ay*ay + az*az);
        ax *= norm; ay *= norm; az *= norm;

        // 叉乘计算误差 (计算重力向量在机体系的投影误差)
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

    float norm = arm_invSqrt(f->q0*f->q0 + f->q1*f->q1 + f->q2*f->q2 + f->q3*f->q3);
    f->q0 *= norm; f->q1 *= norm; f->q2 *= norm; f->q3 *= norm;

    RotationMatrix_update(f);
}

CCM_FUNC void mahony_output(struct MAHONY_FILTER_t *f) {
    float r20 = f->rMat[2][0];
    if (r20 > 1.0f) r20 = 1.0f;
    if (r20 < -1.0f) r20 = -1.0f;

    float sqrt_val;
    arm_sqrt_f32(1.0f - r20 * r20, &sqrt_val);
    // 1. Pitch: 注意符号
    // 原公式 pitch = -asin(r20) = -atan2(r20, sqrt(1-r20^2))
    f->pitch = -CORDIC_Atan2_Fast(r20, sqrt_val);
    // 2. Roll: atan2(r21, r22)
    f->roll  = CORDIC_Atan2_Fast(f->rMat[2][1], f->rMat[2][2]);
    // 3. Yaw: atan2(r10, r00)
    f->yaw   = CORDIC_Atan2_Fast(f->rMat[1][0], f->rMat[0][0]);
    // --- 偏航角累加逻辑 ---
    float yaw_diff = f->yaw - f->last_yaw;
    if (yaw_diff > 180.0f)  yaw_diff -= 360.0f;
    else if (yaw_diff < -180.0f) yaw_diff += 360.0f;

    f->YawTotalAngle += yaw_diff;
    f->last_yaw = f->yaw;
}
/*/**
 * @brief 从旋转矩阵解算并输出姿态角（俯仰/横滚/偏航）
 * @param f 指向MAHONY_FILTER_t结构体的指针，存储旋转矩阵和姿态角结果
 * @return void
 #1#
void mahony_output(struct MAHONY_FILTER_t *f) {
    f->pitch = -asinf(f->rMat[2][0]) * RAD2DEG;
    f->roll  = atan2f(f->rMat[2][1], f->rMat[2][2]) * RAD2DEG;
    f->yaw   = atan2f(f->rMat[1][0], f->rMat[0][0]) * RAD2DEG;
    // 累积偏航角计算：当偏航角变化超过180度时，认为发生了跳变，进行累积修正
    float yaw_diff = f->yaw - f->last_yaw;
    if (yaw_diff > 180.0f) {
        yaw_diff -= 360.0f;
    } else if (yaw_diff < -180.0f) {
        yaw_diff += 360.0f;
    }
    // 累加到总偏航角
    f->YawTotalAngle += yaw_diff;
    // 更新上一次的偏航角，用于下一次计算
    f->last_yaw = f->yaw;
}*/
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

    f->pitch = 0;
    f->roll = 0;
    f->yaw = 0;
    f->YawTotalAngle = 0;

    f->exInt = f->eyInt = 0;

    RotationMatrix_update(f);
    osDelay(1);
    f->mahony_update = mahony_update;
    f->mahony_output = mahony_output;
}