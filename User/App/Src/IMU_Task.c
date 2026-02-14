/**
 * @file    IMU_Task.c
 * @author  CaoKangqi (Optimized version)
 * @date    2026/01/27
 * @brief   IMU温控与校准任务，采用模糊PID控制与状态机管理
 */

#include "IMU_Task.h"
#include <math.h>

/*==================== 温控与校准常量 ====================*/
#define IMU_TARGET_TEMP        40.0f     // 目标温度 (℃)
#define TEMP_STABLE_ERR        0.35f     // 稳定判据误差
#define TEMP_STABLE_TIME_MS    1500      // 稳定持续时间 (ms)
#define GYRO_CALIB_SAMPLES     1000      // 陀螺仪采样样本数

typedef struct {
    float kp;
    float ki;
    float kd;
} PID_Params_t;

static const PID_Params_t base_pid = {70.0f, 0.12f, 100.0f};

CCM_DATA static PID_Params_t current_pid;

#define HEATER_PWM_MAX         1000.0f

CCM_DATA IMU_CTRL_STATE_e imu_ctrl_state = TEMP_INIT;// 当前控制状态
CCM_DATA IMU_CTRL_FLAG_t  imu_ctrl_flag  = {0};// 控制状态标志
CCM_DATA PID_t imu_temp;
CCM_DATA FuzzyRule_t fuzzy_rule_temp;
CCM_DATA IMU_Data_t IMU_Data = {
    .accel_bias = {-0.0018742225f, -0.0085052567f, -0.3006388713f},
    .accel_scale = {0.9991735142f, 1.0005724099f, 0.9983936339f}
};

static CCM_DATA uint32_t temp_stable_tick = 0;// 温度稳定计时起点
static CCM_DATA uint16_t imu_pid_cnt      = 0;//PID控制计数器，用于10ms分频执行PID计算
static CCM_DATA uint16_t gyro_calib_cnt   = 0;//陀螺仪校准计数
static CCM_DATA float heater_pwm_out   = 0;// 当前加热片PWM输出值

/**
 * @brief 设置加热片PWM占空比
 */
void Set_Heater_PWM(float pwm)
{
    // 限幅保护
    pwm = (pwm < 0.0f) ? 0.0f : (pwm > HEATER_PWM_MAX) ? HEATER_PWM_MAX : pwm;
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, (uint32_t)pwm);
}

/**
 * @brief 初始化PID结构体与模糊规则
 * @note  仅在系统启动或状态机复位时调用一次
 */
void IMU_Temp_Control_Init(void)
{
    // 1. 初始化PID控制器基础配置
    PID_Init(&imu_temp,
             1000.0f,               // MaxOut
             600.0f,                // IntegralLimit
             (float*)&base_pid,     // 指向初始参数
             7.5f,                  // CoefA
             1.0f,                  // CoefB
             0.0f,                  // Output LPF
             0.0f,                 // D LPF
             0,
             Trapezoid_Intergral |
             ChangingIntegrationRate |
             Derivative_On_Measurement |
             DerivativeFilter |
             Integral_Limit |
             OutputFilter);

    // 2. 初始化模糊规则参数
    Fuzzy_Rule_Init(&fuzzy_rule_temp, NULL, NULL, NULL,
        6.0f, 0.015f, 10.0f, // Kp, Ki, Kd Ratios
        3.5f, // eStep
        0.85f // ecStep
);

    current_pid = base_pid;
}

/*==================== 核心任务逻辑 ====================*/
/**
 * @brief IMU温度控制与状态管理主任务
 * @note  建议调用频率：1ms (内部自带10ms分频)
 */
void IMU_Update_Task(void)
{
    float now_temp = IMU_Data.temp;
    IMU_Status_Check();// 监测IMU数据，若不正常则进入错误状态
    if (imu_ctrl_state != TEMP_INIT)
    {
        if (++imu_pid_cnt >= 10)
        {
            heater_pwm_out = (now_temp <= IMU_TARGET_TEMP-10.0f)
                              ? HEATER_PWM_MAX
                              : PID_Calculate(&imu_temp, now_temp, IMU_TARGET_TEMP);

            if (now_temp > IMU_TARGET_TEMP-10.0f)
            {
                // 更新模糊推理
                Fuzzy_Rule_Implementation(&fuzzy_rule_temp, now_temp, IMU_TARGET_TEMP);
                // 在基准参数上叠加模糊修正量
                current_pid.kp = base_pid.kp + (fuzzy_rule_temp.KpFuzzy * fuzzy_rule_temp.KpRatio);
                current_pid.ki = base_pid.ki + (fuzzy_rule_temp.KiFuzzy * fuzzy_rule_temp.KiRatio);
                current_pid.kd = base_pid.kd + (fuzzy_rule_temp.KdFuzzy * fuzzy_rule_temp.KdRatio);
                PID_set(&imu_temp, (float*)&current_pid);
            }
            Set_Heater_PWM(heater_pwm_out);
            imu_pid_cnt = 0;
        }
    }


    switch (imu_ctrl_state)
    {
        case TEMP_INIT:
            IMU_Temp_Control_Init();
            //IMU_QuaternionEKF_Init(10, 0.01f, 10000000, 1, 0.001f,0);
            mahony_init(&mahony_filter, 5.0f, 0.01f, 0.001f);
            imu_ctrl_state = TEMP_PID_CTRL;
            break;

        case TEMP_PID_CTRL:
            WS2812_SetPixel(0, 200, 40, 0);  // 橙色：加热中
            if (fabsf(now_temp - IMU_TARGET_TEMP) < TEMP_STABLE_ERR)
            {
                imu_ctrl_flag.temp_reached = 1;
                temp_stable_tick = HAL_GetTick();
                imu_ctrl_state = TEMP_STABLE;
            }
            break;

        case TEMP_STABLE:
            WS2812_SetPixel(0, 200, 0, 200); // 紫色：恒温稳定判断
            if (fabsf(now_temp - IMU_TARGET_TEMP) < TEMP_STABLE_ERR)
            {
                if (HAL_GetTick() - temp_stable_tick > TEMP_STABLE_TIME_MS)
                {
                    HAL_TIM_PWM_Stop(&htim20, TIM_CHANNEL_2);
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
            WS2812_SetPixel(0, 0, 0, 200);   // 蓝色：校准中
            IMU_Gyro_Zero_Calibration_Task();
            if (imu_ctrl_flag.gyro_calib_done)
            {
                imu_ctrl_flag.gyro_calib_done = 0;
                gyro_calib_cnt = 0;
                VOFA_justfloat(
            IMU_Data.accel_correct[0],
            IMU_Data.accel_correct[1],
            IMU_Data.accel_correct[2],0,0,0,0,0,0,0);//用于加速度计椭球拟合零偏及尺度因子
                IMU_Data.accel_correct[0]=0;
                IMU_Data.accel_correct[1]=0;
                IMU_Data.accel_correct[2]=0;
                imu_ctrl_flag.gyro_calib_done = 0;
                gyro_calib_cnt = 0;
                imu_ctrl_state = FUSION_RUN;
            }
            break;

        case FUSION_RUN:
            WS2812_SetPixel(0, 0, 60, 0);    // 绿色：正常运行
            //HAL_TIM_PWM_Start(&htim20, TIM_CHANNEL_2);
            // 减去静态零偏
            IMU_Data.gyro[0] -= IMU_Data.gyro_correct[0];
            IMU_Data.gyro[1] -= IMU_Data.gyro_correct[1];
            IMU_Data.gyro[2] -= IMU_Data.gyro_correct[2];
            IMU_Data.gyro[1] = -IMU_Data.gyro[1];
            IMU_Data.gyro[2] = -IMU_Data.gyro[2];


            IMU_Data.accel[0] = (IMU_Data.accel[0] - IMU_Data.accel_bias[0]) * IMU_Data.accel_scale[0];
            IMU_Data.accel[1] = (IMU_Data.accel[1] - IMU_Data.accel_bias[1]) * IMU_Data.accel_scale[1];
            IMU_Data.accel[2] = (IMU_Data.accel[2] - IMU_Data.accel_bias[2]) * IMU_Data.accel_scale[2];
            /*VOFA_justfloat(
            IMU_Data.gyro[0],
            IMU_Data.gyro[1],
            IMU_Data.gyro[2],
            IMU_Data.accel[0],
            IMU_Data.accel[1],
            IMU_Data.accel[2],0,0,0,0);//用于FFT分析采样*/
            IMU_Data.accel[1] = -IMU_Data.accel[1];
            IMU_Data.accel[2] = -IMU_Data.accel[2];
            /*IMU_QuaternionEKF_Update(
                IMU_Data.gyro[0],IMU_Data.gyro[1],IMU_Data.gyro[2],
                IMU_Data.accel[0],IMU_Data.accel[1],IMU_Data.accel[2]);
            IMU_Data.pitch=Get_Pitch();//获得pitch
            IMU_Data.roll=Get_Roll();//获得roll
            IMU_Data.yaw=Get_Yaw();//获得yaw
            IMU_Data.YawTotalAngle=Get_YawTotalAngle();
            memcpy(IMU_Data.q, QEKF_INS.q, 16);//EKF更新*/
            mahony_update(&mahony_filter,
            IMU_Data.gyro[0], IMU_Data.gyro[1], IMU_Data.gyro[2],
            IMU_Data.accel[0], IMU_Data.accel[1], IMU_Data.accel[2]);
            mahony_output(&mahony_filter);
            IMU_Data.pitch = mahony_filter.pitch;
            IMU_Data.roll = mahony_filter.roll;
            IMU_Data.yaw = mahony_filter.yaw;
            IMU_Data.YawTotalAngle = mahony_filter.YawTotalAngle;
            imu_ctrl_flag.fusion_enabled = 1;
            break;
        case ERROR_STATE:
            Set_Heater_PWM(0); // 关闭加热片
            WS2812_SetPixel(0, 255, 0, 0); // 红色表示错误
            break;
        default:
            break;
    }

}

/**
 * @brief 陀螺仪静态零偏校准任务
 */
void IMU_Gyro_Zero_Calibration_Task(void)
{
    if (imu_ctrl_flag.gyro_calib_done) return;


    IMU_Data.gyro_correct[0] += IMU_Data.gyro[0];
    IMU_Data.gyro_correct[1] += IMU_Data.gyro[1];
    IMU_Data.gyro_correct[2] += IMU_Data.gyro[2];

    IMU_Data.accel_correct[0] += IMU_Data.accel[0];
    IMU_Data.accel_correct[1] += IMU_Data.accel[1];
    IMU_Data.accel_correct[2] += IMU_Data.accel[2]; // 重力补偿
    gyro_calib_cnt++;

    if (gyro_calib_cnt >= GYRO_CALIB_SAMPLES)
    {
        const float div = 1.0f / (float)GYRO_CALIB_SAMPLES;
        IMU_Data.gyro_correct[0] *= div;
        IMU_Data.gyro_correct[1] *= div;
        IMU_Data.gyro_correct[2] *= div;

        IMU_Data.accel_correct[0] *= div;
        IMU_Data.accel_correct[1] *= div;
        IMU_Data.accel_correct[2] *= div;

        gyro_calib_cnt = 0;
        imu_ctrl_flag.gyro_calib_done = 1;
    }
}
/**
 * @brief 外部触发重新校准
 */
void IMU_Gyro_Calib_Initiate(void)
{

    imu_ctrl_flag.gyro_calib_done = 0;// 重置校准完成标志
    gyro_calib_cnt = 0;// 重置计数器
    IMU_Data.gyro_correct[0] = IMU_Data.gyro_correct[1] = IMU_Data.gyro_correct[2] = 0.0f;
}

/**
 * @brief IMU数据状态检查，包含静态零值检测、数据卡死检测和温度边界保护
 * @note  该函数在每次IMU数据更新后调用，若检测到异常则将状态机切换到ERROR_STATE
 */
void IMU_Status_Check(void) {
        // 静态变量用于卡死检测
    static float last_sum = 0;
    static uint16_t stuck_cnt = 0;

    if ((fabsf(IMU_Data.accel[0]) < 1e-6f && fabsf(IMU_Data.accel[1]) < 1e-6f && fabsf(IMU_Data.accel[2]) < 1e-6f)
    ||(fabsf(IMU_Data.gyro[0]) < 1e-6f && fabsf(IMU_Data.gyro[1]) < 1e-6f && fabsf(IMU_Data.gyro[2]) < 1e-6f))
    {
        imu_ctrl_state = ERROR_STATE;
    }
    // 数据卡死检测，将七个数据求和，若连续100次采样完全一致，判定为传感器内部逻辑死锁
    float sum = 0;
    for(int i=0; i<3; i++) {
        sum += IMU_Data.accel[i] + IMU_Data.gyro[i];
    }
    if (fabsf(sum - last_sum) < 1e-7f) {
        if (++stuck_cnt > 100) {
            imu_ctrl_state = ERROR_STATE;
        }
    } else {
        stuck_cnt = 0;
        last_sum = sum;
    }
        // 5. 温度边界保护
    if (IMU_Data.temp > 50.0f || IMU_Data.temp < 0.0f) {
        imu_ctrl_state = ERROR_STATE;
    }
}