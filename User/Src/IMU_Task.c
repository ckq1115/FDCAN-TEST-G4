//
// Created by CaoKangqi on 2026/1/27.
//
#include "IMU_Task.h"

/*==================== 温控参数 ====================*/
#define IMU_TARGET_TEMP        40.0f     // 目标温度
#define TEMP_STABLE_ERR        0.3f      // ±0.3℃
#define TEMP_STABLE_TIME_MS    2000      // 稳定 2s

/*==================== PID参数 ====================*/
static float pid_temp[3] = {90.0f, 0.155f, 180.0f};

/*==================== 加热PWM限制 ====================*/
#define HEATER_PWM_MAX         1000.0f
#define HEATER_PWM_RAMP_UP     20.0f
#define HEATER_PWM_RAMP_DN     30.0f

/*==================== 陀螺仪校准 ====================*/
#define GYRO_CALIB_SAMPLES     2000

/*==================== 控制对象 ====================*/
PID_t imu_temp;
IMU_Data_t IMU_Data;

/*==================== 状态与标志 ====================*/
IMU_CTRL_STATE_e imu_ctrl_state = TEMP_INIT;
IMU_CTRL_FLAG_t  imu_ctrl_flag  = {0};

/*==================== 内部计数 ====================*/
static uint32_t temp_stable_tick = 0;
static uint16_t imu_pid_cnt      = 0;
static uint16_t gyro_calib_cnt   = 0;

/*==================== PWM控制 ====================*/
static float heater_pwm_limited = 0;
static uint32_t heater_ccr = 0;

void Set_Heater_PWM(float pwm)
{
    /* 限幅 */
    if (pwm < 0.0f) pwm = 0.0f;
    if (pwm > HEATER_PWM_MAX) pwm = HEATER_PWM_MAX;

    heater_ccr = (uint32_t)pwm;
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, heater_ccr);
}

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

void IMU_Temp_PID_Init(void)
{
    PID_Init(&imu_temp,
             1000.0f,     // MaxOut
             600.0f,      // IntegralLimit
             pid_temp,
             6.0f,       // CoefA
             0.5f,        // CoefB
             0.5f,        // Output LPF
             0.15f,       // D LPF
             0,
             Trapezoid_Intergral |
             ChangingIntegrationRate |
             Derivative_On_Measurement |
             DerivativeFilter |
             Integral_Limit |
             OutputFilter);
}

void IMU_Temp_Control_Task(void)
{
    float temp = IMU_Data.temp;
    float pwm;

    /*==================== PID周期计算 ====================*/
    if (imu_ctrl_state != TEMP_INIT)
    {
        if (++imu_pid_cnt >= 10)   // 10ms
        {
            pwm = PID_Calculate(&imu_temp, temp, IMU_TARGET_TEMP);
            pwm = Heater_PWM_Limit(pwm);
            Set_Heater_PWM(pwm);
            imu_pid_cnt = 0;
        }
    }

    /*==================== 状态机 ====================*/
    switch (imu_ctrl_state)
    {
        case TEMP_INIT:

            IMU_Temp_PID_Init();
            imu_ctrl_state = TEMP_PID_CTRL;
            break;

        case TEMP_PID_CTRL:
            WS2812_SetPixel(0, 200, 40, 0);  // 橙色表示预热阶段
            if (fabsf(temp - IMU_TARGET_TEMP) < TEMP_STABLE_ERR)
            {
                imu_ctrl_flag.temp_reached = 1;
                temp_stable_tick = HAL_GetTick();
                imu_ctrl_state = TEMP_STABLE;
            }
            break;

        case TEMP_STABLE:
            WS2812_SetPixel(0, 0, 0, 200); // 蓝色表示温度稳定阶段
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
            WS2812_SetPixel(0, 200, 0, 200); // 紫色表示陀螺仪校准阶段
            IMU_Gyro_Zero_Calibration_Task();
            if (imu_ctrl_flag.gyro_calib_done)
            {
                imu_ctrl_state = FUSION_RUN;
            }
            break;

        case FUSION_RUN:
            WS2812_SetPixel(0, 0, 60, 0); // 绿色表示融合算法正常运行阶段
            IMU_Data.gyro[0] -= IMU_Data.gyro_correct[0];
            IMU_Data.gyro[1] -= IMU_Data.gyro_correct[1];
            IMU_Data.gyro[2] -= IMU_Data.gyro_correct[2];
            imu_ctrl_flag.fusion_enabled = 1;
            break;

        default:
            break;
    }
    WS2812_UpdateBreathing(0, 2.0f);
    WS2812_Submit();
}


void IMU_Gyro_Zero_Calibration_Task(void)
{
    if (imu_ctrl_flag.gyro_calib_done)
        return;

    if (ICM42688_IsDataReady())
    {
        IMU_Data.gyro_correct[0] += IMU_Data.gyro[0];
        IMU_Data.gyro_correct[1] += IMU_Data.gyro[1];
        IMU_Data.gyro_correct[2] += IMU_Data.gyro[2];
        gyro_calib_cnt++;
    }

    if (gyro_calib_cnt >= GYRO_CALIB_SAMPLES)
    {
        IMU_Data.gyro_correct[0] /= GYRO_CALIB_SAMPLES;
        IMU_Data.gyro_correct[1] /= GYRO_CALIB_SAMPLES;
        IMU_Data.gyro_correct[2] /= GYRO_CALIB_SAMPLES;

        gyro_calib_cnt = 0;
        imu_ctrl_flag.gyro_calib_done = 1;
    }
}

void IMU_Gyro_Calib_Initiate(void)
{
    imu_ctrl_flag.gyro_calib_done = 0;
    gyro_calib_cnt = 0;

    IMU_Data.gyro_correct[0] = 0.0f;
    IMU_Data.gyro_correct[1] = 0.0f;
    IMU_Data.gyro_correct[2] = 0.0f;
}