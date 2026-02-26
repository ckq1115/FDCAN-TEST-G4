//
// Created by CaoKangqi on 2026/2/18.
//
#include "Chassis_Task.h"
#include <stdint.h>
#include "All_Motor.h"
#include "All_define.h"
#include "All_Init.h"
#include "Chassis_Calc.h"

OmniInit_typdef OmniInit_t;
PID_t PID_Chassis_Angle;

float Ramp_Control_Asym(float target, float current,
                        float accel_up, float accel_down,
                        float dt)
{
    float diff = target - current;

    float step;

    // 判断是否在减速（目标绝对值小于当前）
    if (fabs(target) < fabs(current))
    {
        step = accel_down * dt;   // 减速
    }
    else
    {
        step = accel_up * dt;     // 加速
    }

    if (diff > step)
        diff = step;
    else if (diff < -step)
        diff = -step;

    return current + diff;
}


uint8_t Chassis_Control_Init(MOTOR_Typdef *MOTOR)
{
    OmniInit(&OmniInit_t);
    float PID_F_0[3] = {0,0,0};
    float PID_S_0[3] = {4.0f,0.05f,0};
    float PID_F_1[3] = {0,0,0};
    float PID_S_1[3] = {4.0f,0.05f,0};
    float PID_F_2[3] = {0,0,0};
    float PID_S_2[3] = {4.0f,0.05f,0};
    float PID_F_3[3] = {0,0,0};
    float PID_S_3[3] = {4.0f,0.05f,0};
    float PID_Angle[3] = {5.0f,0,0};

    PID_Init(&MOTOR->DJI_3508_Chassis[0].PID_S, 100000.0f, 1000.0f,
             PID_S_0, 0, 0,
             0, 0, 0,
             Integral_Limit|ErrorHandle//积分限幅,输出滤波,堵转监测
             //梯形积分,变速积分
             );//微分先行,微分滤波器
    PID_Init(&MOTOR->DJI_3508_Chassis[1].PID_S, 100000.0f, 1000.0f,
             PID_S_1, 0, 0,
             0, 0, 0,
             Integral_Limit|ErrorHandle//积分限幅,输出滤波,堵转监测
             //梯形积分,变速积分
             );//微分先行,微分滤波器
    PID_Init(&MOTOR->DJI_3508_Chassis[2].PID_S, 100000.0f, 1000.0f,
             PID_S_2, 0, 0,
             0, 0, 0,
             Integral_Limit|ErrorHandle//积分限幅,输出滤波,堵转监测
             //梯形积分,变速积分
             );//微分先行,微分滤波器
    PID_Init(&MOTOR->DJI_3508_Chassis[3].PID_S, 100000.0f, 1000.0f,
             PID_S_3, 0, 0,
             0, 0, 0,
             Integral_Limit|ErrorHandle//积分限幅,输出滤波,堵转监测
             //梯形积分,变速积分
             );//微分先行,微分滤波器
    PID_Init(&PID_Chassis_Angle, 100000.0f, 1000.0f,
             PID_Angle, 0, 0,
             0, 0, 0,
             Integral_Limit|ErrorHandle//积分限幅,输出滤波,堵转监测
             //梯形积分,变速积分
             );//微分先行,微分滤波器
    return DF_READY;
}
float Chassis_Power_Model(MOTOR_Typdef *MOTOR)
{
    const float k1 = 2.2642576729e-02f;
    const float k2 = 5.2592404252e-02f;
    const float k3 = 6.3284599588e-03f;
    const float k4 = 5.1979615262e+00f;

    float sum_wi = 0.0f;
    float sum_i2 = 0.0f;
    float sum_w  = 0.0f;

    for (int i = 0; i < 4; i++)
    {
        float w = MOTOR->DJI_3508_Chassis[i].DATA.Speed_now;   // rpm
        float I = MOTOR->DJI_3508_Chassis[i].DATA.current*0.001f;     // ⚠ 用反馈电流！

        // 如果 current 单位是 0.01A，需要：
        // I *= 0.01f;

        // rpm → rad/s
        w = w * 2.0f * 3.1415926f / 60.0f;

        sum_wi += fabsf(w * I);
        sum_i2 += I * I;
        sum_w  += fabsf(w);
    }

    float power_model =
        k1 * sum_wi +
        k2 * sum_i2 +
        k3 * sum_w +
        k4;

    return power_model;
}
float m = 0;
void Chassis_Control_Task(MOTOR_Typdef *MOTOR)
{
    float wheel_rpm[4];
    float dt = 0.001f;
    float vx_target = C_DBUS.Remote.CH0_int16 * 3;
    float vy_target = -C_DBUS.Remote.CH1_int16 * 3;
    float yaw_in = -(C_DBUS.Remote.CH2_int16 - C_DBUS.Remote.Dial_int16)/10;
    m = Chassis_Power_Model(MOTOR);
    /* ========= 角度目标 ========= */
    static float Chassis_Target_Yaw = 0;
    // 角度积分（角速度控制）
    float target_w_input = yaw_in*6;
    Chassis_Target_Yaw += target_w_input * dt;
    PID_Calculate(&PID_Chassis_Angle,
                  IMU_Data.YawTotalAngle,
                  Chassis_Target_Yaw);
    float vw_target = -PID_Chassis_Angle.Output;

    /* ========= 非对称缓启动 ========= */

    static float vx_set = 0;
    static float vy_set = 0;
    static float vw_set = 0;

    // 平移：起步慢，刹车快
    vx_set = Ramp_Control_Asym(vx_target, vx_set,3000,3000,dt);
    vy_set = Ramp_Control_Asym(vy_target, vy_set,3000,3000,dt);
    // 旋转：单独控制
    vw_set = Ramp_Control_Asym(vw_target, vw_set,800,4000,dt);
    /* ========= 底盘解算 ========= */
    Omni_calc(wheel_rpm, vx_set, vy_set, vw_set, &OmniInit_t);
    /* ========= 电机速度环 ========= */
    for (uint8_t i = 0; i < 4; i++)
    {
        PID_Calculate(&MOTOR->DJI_3508_Chassis[i].PID_S,
                      MOTOR->DJI_3508_Chassis[i].DATA.Speed_now,
                      wheel_rpm[i]);
    }
    DJI_Motor_Send(&hfdcan1,0x200,All_Motor.DJI_3508_Chassis[0].PID_S.Output,
        All_Motor.DJI_3508_Chassis[1].PID_S.Output,
        All_Motor.DJI_3508_Chassis[2].PID_S.Output,
        All_Motor.DJI_3508_Chassis[3].PID_S.Output);
}
