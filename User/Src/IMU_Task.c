//
// Created by CaoKangqi on 2026/1/27.
//
#include "IMU_Task.h"

#define IMU_TARGET_TEMP      40.0f
#define TEMP_STABLE_ERR      0.4f    // ±0.1℃
#define TEMP_STABLE_TIME_MS  2000    // 稳定 2 秒

uint32_t ccr=0;
void Set_Heater_PWM(float pwm)
{
    /* 限幅 */
    if (pwm < 0.0f)
        pwm = 0.0f;
    else if (pwm > 1000.0f)
        pwm = 1000.0f;

    /* 计算 CCR */
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim3);
    ccr = (uint32_t)(pwm);

    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, ccr);
}

PID_t imu_temp;

IMU_CTRL_STATE_e imu_ctrl_state = TEMP_INIT;
IMU_CTRL_FLAG_t imu_ctrl_flag = {0};

static uint32_t temp_stable_tick = 0;
float pid_temp[3] = {90.0f, 0.2f, 180.0f};
//float pid_temp[3] = {10.0f, 0.25f, 0.0f};
void IMU_Temp_PID_Init(void){
    PID_Init(&imu_temp,
             1000.0f,     // MaxOut
             350.0f,      // IntegralLimit
             pid_temp,
             4.0f,        // CoefA
             0.8f,        // CoefB
             0.5f,       // Output LPF
             0.15f,       // D LPF
             0,
             Trapezoid_Intergral |
             ChangingIntegrationRate |
             Derivative_On_Measurement |
             DerivativeFilter |
             Integral_Limit |
             OutputFilter);
}

static uint16_t imu_pid_cnt = 0;

void IMU_Temp_Control_Task(void)
{
    float temp = IMU_Data.temp;
    float pwm;

    if (imu_ctrl_state != TEMP_INIT)
    {
        imu_pid_cnt++;
        if (imu_pid_cnt >= 10) // 10ms 计算一次 PID
        {
            pwm = PID_Calculate(&imu_temp, temp, IMU_TARGET_TEMP);
            pwm= Heater_PWM_Limit(pwm);
            Set_Heater_PWM((uint16_t)pwm);
            imu_pid_cnt = 0;
            }
    }

    switch (imu_ctrl_state)
    {
        case TEMP_INIT:
            IMU_Temp_PID_Init();
            imu_ctrl_state = TEMP_PREHEAT;
            //imu_ctrl_state = GYRO_CALIB;
            break;

        case TEMP_PREHEAT:

                imu_ctrl_state = TEMP_PID_CTRL;

            break;

        case TEMP_PID_CTRL:
            if (fabsf(temp - IMU_TARGET_TEMP) < TEMP_STABLE_ERR)
            {
                imu_ctrl_flag.temp_reached = 1;
                temp_stable_tick = HAL_GetTick();
                imu_ctrl_state = TEMP_STABLE;
            }
            break;

        case TEMP_STABLE:
            if (fabsf(temp - IMU_TARGET_TEMP) < TEMP_STABLE_ERR)
            {
                if (HAL_GetTick() - temp_stable_tick > TEMP_STABLE_TIME_MS)
                {
                    imu_ctrl_flag.temp_stable = 1;
                    imu_ctrl_state = GYRO_CALIB;
                }
            }
            else
            {
                imu_ctrl_state = TEMP_PID_CTRL;
            }
            break;

        case GYRO_CALIB:
            IMU_Gyro_Zero_Calibration_Task();
            if (imu_ctrl_flag.gyro_calib_done)
            {
                imu_ctrl_state = FUSION_RUN;
            }
            break;

        case FUSION_RUN:
            IMU_Data.gyro[0] -= IMU_Data.gyro_correct[0];
            IMU_Data.gyro[1] -= IMU_Data.gyro_correct[1];
            IMU_Data.gyro[2] -= IMU_Data.gyro_correct[2];
            imu_ctrl_flag.fusion_enabled = 1;
            break;

        default:
            break;
    }
}


#define GYRO_CALIB_SAMPLES  1500

static uint16_t gyro_calib_cnt = 0;
IMU_Data_t IMU_Data;

// 陀螺仪零漂校准任务
void IMU_Gyro_Zero_Calibration_Task(void)
{
    // 【修正1】校准完成后直接返回，终止后续无效操作
    if (imu_ctrl_flag.gyro_calib_done) {
        return;
    }

    // 【修正2】仅当数据就绪时，才执行采集、累加和计数（保证数据有效性）
    if (ICM42688_IsDataReady()) {
        // 读取最新的陀螺仪、加速度计、温度数据
        //ICM42688_read(IMU_Data.gyro, IMU_Data.accel, &IMU_Data.temp);

        // 累加陀螺仪数据，用于后续求平均值
        IMU_Data.gyro_correct[0] += IMU_Data.gyro[0];
        IMU_Data.gyro_correct[1] += IMU_Data.gyro[1];
        IMU_Data.gyro_correct[2] += IMU_Data.gyro[2];

        // 校准计数自增
        gyro_calib_cnt++;
    }

    // 判断是否完成指定次数的采样
    if (gyro_calib_cnt >= GYRO_CALIB_SAMPLES)
    {
        // 【核心逻辑】求平均值，得到陀螺仪零漂补偿值
        IMU_Data.gyro_correct[0] /= GYRO_CALIB_SAMPLES;
        IMU_Data.gyro_correct[1] /= GYRO_CALIB_SAMPLES;
        IMU_Data.gyro_correct[2] /= GYRO_CALIB_SAMPLES;

        // 【修正3】重置计数，为后续可能的重新校准做准备
        gyro_calib_cnt = 0;

        // 标记陀螺仪校准完成
        imu_ctrl_flag.gyro_calib_done = 1;

        // 注：日常数据的零漂补偿，应放在「正常读取IMU数据」的函数中，而非此处
        // 示例：后续读取 gyro 后，执行 gyro[0] -= gyro_correct[0]
    }
}
// 触发陀螺仪校准时的初始化操作（放在调用 IMU_Gyro_Zero_Calibration_Task 之前）
void IMU_Gyro_Calib_Initiate(void)
{
    // 重置校准完成标记
    imu_ctrl_flag.gyro_calib_done = 0;

    // 重置累加缓冲区为0
    IMU_Data.gyro_correct[0] = 0.0f;
    IMU_Data.gyro_correct[1] = 0.0f;
    IMU_Data.gyro_correct[2] = 0.0f;

    // 重置采样计数为0
    gyro_calib_cnt = 0;
}


static float heater_pwm_limited = 0;
#define HEATER_PWM_RAMP_UP   30.0f   // 每 10ms 最多 +20（=0.2%）
#define HEATER_PWM_RAMP_DN   30.0f   // 降可以快一点
float Heater_PWM_Limit(float target_pwm)
{
    if (target_pwm > heater_pwm_limited)
    {
        heater_pwm_limited += HEATER_PWM_RAMP_UP;
        if (heater_pwm_limited > target_pwm)
            heater_pwm_limited = target_pwm;
    }
    else
    {
        heater_pwm_limited -= HEATER_PWM_RAMP_DN;
        if (heater_pwm_limited < target_pwm)
            heater_pwm_limited = target_pwm;
    }

    return heater_pwm_limited;
}