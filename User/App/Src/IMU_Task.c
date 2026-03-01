/**
 * @file    IMU_Task.c
 * @author  CaoKangqi
 * @date    2026/01/27
 * @brief   IMU温控与校准任务，采用模糊PID控制与状态机管理
 */

#include "IMU_Task.h"
#include <math.h>

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
    /*.accel_bias = {-0.0018742225f, -0.0085052567f, -0.3006388713f},
    .accel_scale = {0.9930995110f, 0.9944899028f, 0.9923243716f}*/
    .accel_bias = {-0.0039012455f, -0.0100767006f, -0.2877107718f},
    .accel_scale = {0.9982235869f, 1.0002515018f, 0.9962459264f}
};

static CCM_DATA uint32_t temp_stable_tick = 0;// 温度稳定计时起点
static CCM_DATA uint16_t imu_pid_cnt      = 0;//PID控制计数器，用于10ms分频执行PID计算
static CCM_DATA uint16_t gyro_calib_cnt   = 0;//陀螺仪校准计数
static CCM_DATA float heater_pwm_out   = 0;// 当前加热片PWM输出值

/**
 * @brief 设置加热片PWM输出
 * @param pwm 目标PWM值 (0.0f - HEATER_PWM_MAX)
 * @note 该函数会自动进行限幅保护，确保PWM值在安全范围内
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
/**
 * @brief IMU数据更新与控制状态机执行函数
 * @note 该函数在每次IMU数据更新后调用，负责执行温控PID计算、状态转换和陀螺仪校准等核心逻辑
 */
CCM_FUNC void IMU_Update_Task(float dt_s)
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
            IMU_QuaternionEKF_Init(10, 0.001f, 1000000, 0.9996f, 0.001f,0);
            mahony_init(&mahony_filter, 5.0f, 0.01f, 0.001f);
#ifdef DEBUG_MODE
            imu_ctrl_state = TEMP_PID_CTRL;
#endif
#ifdef RELEASE_MODE
            IMU_Data.gyro_correct[0] = 0.00413000258f;
            IMU_Data.gyro_correct[1] = -0.0189137105f;
            IMU_Data.gyro_correct[2] = -0.000697744836f;
            imu_ctrl_state = FUSION_RUN;
#endif
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
                    HAL_TIM_PWM_Stop(&htim20, TIM_CHANNEL_2);// 陀螺仪零漂收集开始前关闭蜂鸣器
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
                /*VOFA_justfloat(IMU_Data.accel_correct[0],IMU_Data.accel_correct[1],
                    IMU_Data.accel_correct[2],0,0,0,0,0,0,0);//用于加速度计椭球拟合零偏及尺度因子*/
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
            //HAL_TIM_PWM_Start(&htim20, TIM_CHANNEL_2);// 陀螺仪零漂收集结束后开启蜂鸣器
            const float AXIS_DIR[3] = {1.0f, -1.0f, -1.0f};// 根据安装方向调整轴向，确保输出符合右手坐标系
            for (int i = 0; i < 3; i++) {
                IMU_Data.gyro[i] = (IMU_Data.gyro[i] - IMU_Data.gyro_correct[i]) * AXIS_DIR[i];
                IMU_Data.accel[i] = (IMU_Data.accel[i] - IMU_Data.accel_bias[i]) * IMU_Data.accel_scale[i] * AXIS_DIR[i];
            }
            /*VOFA_justfloat(IMU_Data.gyro[0],IMU_Data.gyro[1],IMU_Data.gyro[2],
            IMU_Data.accel[0],IMU_Data.accel[1],IMU_Data.accel[2],0,0,0,0);//用于FFT分析采样*/

            //mahony姿态融合更新，实测效果还不错，QuaternionEKF有想法的自己整吧
            mahony_update(&mahony_filter,
            IMU_Data.gyro[0], IMU_Data.gyro[1], IMU_Data.gyro[2],
            IMU_Data.accel[0], IMU_Data.accel[1], IMU_Data.accel[2],dt_s);
            mahony_output(&mahony_filter);
            IMU_Data.pitch = mahony_filter.pitch;
            IMU_Data.roll = mahony_filter.roll;
            IMU_Data.yaw = mahony_filter.yaw;
            IMU_Data.YawTotalAngle = mahony_filter.YawTotalAngle;
            imu_ctrl_flag.fusion_enabled = 1;
            break;
        case ERROR_STATE:
            if (ICM42688_Init() == 1) // 尝试重新初始化IMU，成功则认为错误已恢复
            {
                imu_ctrl_state = TEMP_INIT; // 成功则回到初始状态
                break;
            }
            Set_Heater_PWM(0); // 关闭加热片
            WS2812_SetPixel(0, 255, 0, 0); // 红色表示错误
            break;
        default:
            break;
    }

}

/**
 * @brief 陀螺仪零偏校准任务
 * @note  该函数在GYRO_CALIB状态下被周期调用，累计采样数据进行平均，完成后设置校准完成标志
 */
void IMU_Gyro_Zero_Calibration_Task(void)
{
    static float gyro_sq_sum[3] = {0};
    static float accel_sq_sum[3] = {0};

    if (imu_ctrl_flag.gyro_calib_done) return;
    // 累加数据与平方和
    for (int i = 0; i < 3; i++)
    {
        IMU_Data.gyro_correct[i]  += IMU_Data.gyro[i];
        IMU_Data.accel_correct[i] += IMU_Data.accel[i];

        gyro_sq_sum[i]  += IMU_Data.gyro[i]  * IMU_Data.gyro[i];
        accel_sq_sum[i] += IMU_Data.accel[i] * IMU_Data.accel[i];
    }
    gyro_calib_cnt++;
    // 采样未完成，直接返回等待下次调用
    if (gyro_calib_cnt < GYRO_CALIB_SAMPLES) return;
    // 到达采样数量，开始计算方差
    const float div = 1.0f / (float)GYRO_CALIB_SAMPLES;
    uint8_t is_stable = 1;
    for (int i = 0; i < 3; i++)
    {
        // 计算当前均值（临时变量，防止直接修改原始数据导致重试逻辑失效）
        float mean_g = IMU_Data.gyro_correct[i] * div;
        float mean_a = IMU_Data.accel_correct[i] * div;
        // 方差公式：Var = E(x²) - (E(x))²
        float gyro_var  = (gyro_sq_sum[i] * div) - (mean_g * mean_g);
        float accel_var = (accel_sq_sum[i] * div) - (mean_a * mean_a);
        // 判定阈值，如果任一轴的方差超过0.005f，认为数据不稳定，需重新采集
        if (gyro_var > 0.003f || accel_var > 0.002f)
        {
            is_stable = 0;
            break;
        }
    }
    if (is_stable)
    {
        // 判定稳定：计算最终均值并结束校准
        for (int i = 0; i < 3; i++)
        {
            IMU_Data.gyro_correct[i] *= div;
            IMU_Data.accel_correct[i] *= div;
        }
        imu_ctrl_flag.gyro_calib_done = 1;
    }
    else
    {
        // 判定不稳定：清零累加器，下一周期自动重新开始
        for (int i = 0; i < 3; i++)
        {
            IMU_Data.gyro_correct[i] = 0.0f;
            IMU_Data.accel_correct[i] = 0.0f;
            gyro_sq_sum[i] = 0.0f;
            accel_sq_sum[i] = 0.0f;
        }
        // 可以在这里加一个串口打印提示：Calibration failed, retrying...
    }
    gyro_calib_cnt = 0; // 重置计数器
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
    static float last_sum = 0;
    static uint16_t stuck_cnt = 0;
    static uint16_t zero_cnt = 0; // 新增：全零检测计数器

    static uint16_t nan_cnt = 0;  // NaN检测计数器

    // NaN检测
    uint8_t is_nan = 0;
    // 一次性检测加速度、陀螺仪所有轴 + 温度是否存在NaN
    is_nan = (isnanf(IMU_Data.accel[0]) || isnanf(IMU_Data.accel[1]) || isnanf(IMU_Data.accel[2]) ||
              isnanf(IMU_Data.gyro[0])  || isnanf(IMU_Data.gyro[1])  || isnanf(IMU_Data.gyro[2])  ||
              isnanf(IMU_Data.temp));
    // 连续2个周期检测到NaN才判定异常
    if (is_nan) {
        nan_cnt = (nan_cnt >= 2) ? 2 : nan_cnt + 1;  // 防止溢出
        if (nan_cnt >= 2) imu_ctrl_state = ERROR_STATE;
    } else {
        nan_cnt = 0;
    }

    // 静态零值检测，判断加速度或陀螺仪是否全为0
    if ((fabsf(IMU_Data.accel[0]) < 1e-6f && fabsf(IMU_Data.accel[1]) < 1e-6f && fabsf(IMU_Data.accel[2]) < 1e-6f)
     || (fabsf(IMU_Data.gyro[0]) < 1e-6f && fabsf(IMU_Data.gyro[1]) < 1e-6f && fabsf(IMU_Data.gyro[2]) < 1e-6f))
    {
        if (++zero_cnt >= 2) { // 连续2个周期全为0才判定为异常
            imu_ctrl_state = ERROR_STATE;
        }
    } else {
        zero_cnt = 0; // 只要有数据不为 0，立即重置计数器
    }

    // 数据卡死检测
    // 将六轴数据求和，若连续 100 次采样完全一致，判定为传感器内部逻辑死锁（SPI/I2C 还在传，但数据不更新）
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
    // 温度边界保护
    if (IMU_Data.temp > 50.0f || IMU_Data.temp < 0.0f) {
        imu_ctrl_state = ERROR_STATE;
    }
}