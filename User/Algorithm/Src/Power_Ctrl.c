//
// Created by CaoKangqi on 2026/2/23.
//

#include "Power_Ctrl.h"
#include <stdint.h>
#include "DJI_Motor.h"
#include "All_Define.h"

CCM_DATA ALL_POWER_RX All_Power;

static float SafeSectionLimit(float max, float min, float data);
static float SafeSqrt(float x);
static void Power_Mode_Transition(Power_Sample_Manager_t *mgr);

// 底盘总功率的采样通道（请根据实际硬件修改宏定义）
#define CHASSIS_POWER_SENSOR All_Power.P5
#define RPM_TO_RAD 0.104719755f
// 全局静态变量（用于模拟心跳计数器，也可以用HAL_GetTick()）
static uint32_t s_tick_counter = 0;

/**
  * @brief  采样更新函数（必须在500Hz的定时中断或回调中调用！）
  * @param  raw_sample_power: 采样得到的底盘总功率
  */
void Power_Sample_Update(model_t *model, float raw_sample_power)
{
    Power_Sample_Manager_t *mgr = &model->sample_mgr;
    s_tick_counter++; // 模拟时间戳递增

    /* 1. 心跳检测（数据是否更新） */
    // 这里假设power值不变超过一定时间视为掉线（也可以通过通信标志位）
    if (fabsf(raw_sample_power - mgr->raw_power) < 0.1f)
    {
        // 数据未变化，增加离线计数
        if (s_tick_counter - mgr->last_update_tick > SAMPLE_TIMEOUT_CNT)
        {
            mgr->is_offline = 1;
        }
    }
    else
    {
        // 数据有更新，复位状态
        mgr->is_offline = 0;
        mgr->last_update_tick = s_tick_counter;
        mgr->raw_power = raw_sample_power;
    }

    /* 2. 合理性检测（物理边界检查） */
    if (raw_sample_power < 0.0f || raw_sample_power > 500.0f)
    {
        mgr->is_offline = 1; // 数据不合理，强制离线
    }

    /* 3. 模式切换逻辑 */
    Power_Mode_Transition(mgr);

    /* 4. 如果在线，进行低通滤波 */
    if (mgr->mode == POWER_CTRL_MODE_FUSION)
    {
        mgr->filtered_power = mgr->filtered_power * (1.0f - POWER_SAMPLE_ALPHA) +
                              raw_sample_power * POWER_SAMPLE_ALPHA;
    }
}

/**
  * @brief  模式平滑切换，避免硬切换导致的电流跳变
  */
static void Power_Mode_Transition(Power_Sample_Manager_t *mgr)
{
    if (mgr->is_offline)
    {
        mgr->mode = POWER_CTRL_MODE_MODEL_ONLY;
    }
    else
    {
        mgr->mode = POWER_CTRL_MODE_FUSION;
    }
}

/**
  * @brief  功率控制初始化函数
  */
void Power_control_init(model_t *model)
{
    /* PID参数初始化（略，保持不变） */
    model->PID_Buffer.Kp = 2.0f;
    model->PID_Buffer.Ki = 0.0f;
    model->PID_Buffer.Kd = 0.0f;
    model->PID_Buffer.ILt = 0.0f;
    model->PID_Buffer.AlLt = 100.0f;
    model->PID_Buffer.Error[0] = 0.0f;
    model->PID_Buffer.Error[1] = 0.0f;

    //模型参数初始化
    model->k1 = 0.0218f;
    model->k2 = 0.0458f;
    model->k3 = 0.00810f;
    model->k4 = 2.60f;   // 总固定损耗（四个轮子总和）

    /* 采样管理器初始化 */
    model->sample_mgr.raw_power = 0.0f;
    model->sample_mgr.filtered_power = 0.0f;
    model->sample_mgr.model_total_power = 0.0f;
    model->sample_mgr.power_error = 0.0f;
    model->sample_mgr.last_update_tick = 0;
    model->sample_mgr.is_offline = 1; // 初始化为离线，等采样正常后自动切融合
    model->sample_mgr.mode = POWER_CTRL_MODE_MODEL_ONLY;
}

/**
  * @brief  内部静态函数：电机功率计算
  */
/*static float get_initial_power_internal(DJI_MOTOR_Typedef *MOTOR, model_t *model)
{
    float speed_rpm = (float)MOTOR->DATA.Speed_now;
    float omega = speed_rpm * RPM_TO_RAD;   // 必须定义 RPM_TO_RAD
    float Iq = (float)MOTOR->DATA.current;  // ⚠ 必须用真实反馈电流

    return model->k1 * fabsf(omega * Iq) +
           model->k2 * Iq * Iq +
           model->k3 * fabsf(omega);
}*/
static float get_initial_power_internal(DJI_MOTOR_Typedef *MOTOR, model_t *model)
{
    float speed_rpm = (float)MOTOR->DATA.Speed_now;
    float omega = speed_rpm * RPM_TO_RAD;      // RPM → rad/s
    float Iq = (float)MOTOR->PID_S.Output*20000/16384;    // mA 或 A，根据你的单位
    float motor_power = 0.0f;

    motor_power = model->k1 * omega * Iq       // 可以正负
            + model->k2 * Iq * Iq          // 始终正
            + model->k3 * omega            // 可以正负
            + model->k4;                   // 常数损耗

    return motor_power;
}

/**
  * @brief  内部静态函数：功率限制
  */
static void chassis_power_limit_internal(DJI_MOTOR_Typedef *MOTOR,
                                         uint8_t motor_idx,
                                         model_t *model)
{
    if (motor_idx >= CHASSIS_MOTOR_NUM) return;

    float speed_rpm = (float)MOTOR->DATA.Speed_now;
    float omega = speed_rpm * RPM_TO_RAD;
    float Iq = (float)MOTOR->DATA.current;

    float abs_omega = fabsf(omega);

    float a_eq = model->k2;
    float b_eq = model->k1 * abs_omega;
    float c_eq = model->k3 * abs_omega - model->scaled_give_power[motor_idx];

    float discriminant = b_eq * b_eq - 4.0f * a_eq * c_eq;

    if (discriminant < MIN_DISCRIMINANT)
    {
        MOTOR->PID_S.Output = 0.0f;
        return;
    }

    float sqrt_disc = SafeSqrt(discriminant);

    float new_I;

    if (Iq >= 0)
        new_I = (-b_eq + sqrt_disc) / (2.0f * a_eq);
    else
        new_I = (-b_eq - sqrt_disc) / (2.0f * a_eq);

    MOTOR->PID_S.Output =
        SafeSectionLimit(MAX_CURRENT_OUTPUT,
                         -MAX_CURRENT_OUTPUT,
                         new_I);
}

/**
  * @brief  缓冲能量PID
  */
static void PID_buffer_internal(PID_buffer_t *PID_buffer, float power_buffer, float target_buffer)
{
    PID_buffer->Error[0] = target_buffer - power_buffer;
    PID_buffer->P_out = PID_buffer->Error[0] * PID_buffer->Kp;
    PID_buffer->I_out += PID_buffer->Error[0] * PID_buffer->Ki;
    PID_buffer->I_out = SafeSectionLimit(PID_buffer->ILt, -PID_buffer->ILt, PID_buffer->I_out);
    PID_buffer->D_out = -(PID_buffer->Error[0] - PID_buffer->Error[1]) * PID_buffer->Kd;
    PID_buffer->Error[1] = PID_buffer->Error[0];
    PID_buffer->All_out = PID_buffer->P_out + PID_buffer->I_out + PID_buffer->D_out;
    PID_buffer->All_out = SafeSectionLimit(PID_buffer->AlLt, -PID_buffer->AlLt, PID_buffer->All_out);
}

/**
  * @brief  功率控制总函数
  */
float initial_give_power[CHASSIS_MOTOR_NUM];
uint8_t chassis_power_control(CONTAL_Typedef *RUI_V_CONTAL_V,
                           User_Data_T *usr_data,
                           model_t *model,
                           CAP_RXDATA *CAP_GET,
                           MOTOR_Typdef *MOTOR)
{
    /* 可编辑参数 */
    const float TARGET_BUFFER = 25.0f;
    const int16_t NORMAL_POWER_COMP = 15;
    const int16_t CAP_POWER_BOOST = 200;
    const uint16_t CAP_LOW_VOLT_THRESH = 7;
    const uint16_t DEFAULT_POWER_LIMIT = 80;

    uint16_t max_power_limit = DEFAULT_POWER_LIMIT;
    float input_power = 0.0f;
    float chassis_max_power = 0.0f;

    float initial_total_power = 0.0f;

    /* 1. 获取裁判系统数据 */
    if (usr_data->robot_status.chassis_power_limit != 0)
    {
        max_power_limit = usr_data->robot_status.chassis_power_limit;
    }
    float chassis_power_buffer = usr_data->power_heat_data.buffer_energy;

    /* 2. 缓冲能量PID */
    PID_buffer_internal(&model->PID_Buffer, chassis_power_buffer, TARGET_BUFFER);
    input_power = (float)max_power_limit - model->PID_Buffer.All_out;

    /* 3. 超级电容管理 */
    if (CAP_GET->CAP_VOLT > (float)CAP_LOW_VOLT_THRESH)
    {
        chassis_max_power = (model->sample_mgr.mode == POWER_CTRL_MODE_FUSION && RUI_V_CONTAL_V->BOTTOM.CAP == 1) ?
                            (input_power + CAP_POWER_BOOST) : (input_power + NORMAL_POWER_COMP);
    }
    else
    {
        chassis_max_power = input_power;
    }

    /* 4. 计算模型预测总功率 */
    initial_give_power[0] = get_initial_power_internal(&MOTOR->DJI_3508_Chassis[0], model);
    initial_give_power[1] = get_initial_power_internal(&MOTOR->DJI_3508_Chassis[1], model);
    initial_give_power[2] = get_initial_power_internal(&MOTOR->DJI_3508_Chassis[2], model);
    initial_give_power[3] = get_initial_power_internal(&MOTOR->DJI_3508_Chassis[3], model);

    for (uint8_t i = 0; i < CHASSIS_MOTOR_NUM; i++)
    {
        initial_total_power += initial_give_power[i];
    }
    initial_total_power += model->k4;
    model->sample_mgr.model_total_power = initial_total_power;
    /* 核心融合逻辑：如果采样在线，动态校准模型系数 k4 (固定损耗) */
    if (model->sample_mgr.mode == POWER_CTRL_MODE_FUSION)
    {
        // 计算误差：采样功率 - 模型功率
        model->sample_mgr.power_error = model->sample_mgr.filtered_power - initial_total_power;

        // 慢环校正：利用误差微调 k4 (固定损耗/偏移量)
        // 这是一个积分过程，慢慢消除模型的静态误差
        if (fabsf(model->sample_mgr.power_error) < 50.0f)
        {
            model->k4 += model->sample_mgr.power_error * MODEL_CALIB_GAIN;
        }

        // 限制 k4 的范围，防止校正过头
        model->k4 = SafeSectionLimit(10.0f, 0.0f, model->k4);
    }

    /* 5. 功率限制与分配（无论什么模式，都用当前的模型系数进行分配） */
    /* 注意：这里我们不直接用采样功率做分配，因为采样是总功率，无法知道每个轮子的情况 */
    /* 我们通过校准模型系数，让模型变得精准，然后依然用模型进行分配 */
    /* 计算动态功率（不含 k4） */
    float dynamic_power = 0.0f;

    for (uint8_t i = 0; i < CHASSIS_MOTOR_NUM; i++)
    {
        dynamic_power += initial_give_power[i];
    }

    /* 允许的动态功率 = 总限制 - 固定损耗 */
    float allowed_dynamic_power = chassis_max_power - model->k4;

    if (allowed_dynamic_power < 0.0f)
        allowed_dynamic_power = 0.0f;

    /* 只有动态功率超限才缩放 */
    if (dynamic_power > allowed_dynamic_power)
    {
        float power_scale = allowed_dynamic_power / dynamic_power;

        for (uint8_t i = 0; i < CHASSIS_MOTOR_NUM; i++)
        {
            model->scaled_give_power[i] =
                initial_give_power[i] * power_scale;
        }

        chassis_power_limit_internal(&MOTOR->DJI_3508_Chassis[0], 0, model);
        chassis_power_limit_internal(&MOTOR->DJI_3508_Chassis[1], 1, model);
        chassis_power_limit_internal(&MOTOR->DJI_3508_Chassis[2], 2, model);
        chassis_power_limit_internal(&MOTOR->DJI_3508_Chassis[3], 3, model);
    }
    else
    {
        for (uint8_t i = 0; i < CHASSIS_MOTOR_NUM; i++)
        {
            model->scaled_give_power[i] = initial_give_power[i];
        }
    }

    return DF_READY;
}

/* 辅助函数实现 */
static float SafeSectionLimit(float max, float min, float data)
{
    if (max >= min)
    {
        if (data >= max) return max;
        else if (data <= min) return min;
        else return data;
    }
    else
    {
        float temp = min; min = max; max = temp;
        if (data >= max) return max;
        else if (data <= min) return min;
        else return data;
    }
}

static float SafeSqrt(float x)
{
    return (x < MIN_DISCRIMINANT) ? 0.0f : sqrtf(x);
}

void CAN_POWER_Rx(Power_Typedef* Power, uint8_t *rx_data)
{
    /*int16_t raw_shunt = (int16_t)((rx_data[0] << 8) | rx_data[1]);
    int16_t raw_bus = (int16_t)((rx_data[2] << 8) | rx_data[3]);
    int16_t raw_curr = (int16_t)((rx_data[4] << 8) | rx_data[5]);
    int16_t raw_pwr = (int16_t)((rx_data[6] << 8) | rx_data[7]);*/
    int16_t raw_shunt = (int16_t)((int16_t)rx_data[0] << 8 | rx_data[1]);
    int16_t raw_bus   = (int16_t)((int16_t)rx_data[2] << 8 | rx_data[3]);
    int16_t raw_curr  = (int16_t)((int16_t)rx_data[4] << 8 | rx_data[5]);
    int16_t raw_pwr   = (int16_t)((int16_t)rx_data[6] << 8 | rx_data[7]);

    Power->shunt_volt = (float)raw_shunt / 1000.0f;
    Power->bus_volt   = (float)raw_bus   / 1000.0f;
    Power->current    = (float)raw_curr  / 1000.0f;
    //Power->power      = (float)raw_pwr   / 100.0f;
    Power->power      = Power->bus_volt * Power->current;
}
