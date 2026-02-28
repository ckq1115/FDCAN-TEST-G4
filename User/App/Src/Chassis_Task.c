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

float Ramp_Control(float target,
                          float current,
                          float accel_up,     // 加速度
                          float accel_down,   // 减速度
                          float dt)
{
    float diff = target - current;
    float step;
    if (target * current < 0.0f)
    {
        step = accel_down * dt;
    }
    else if (fabs(target) < fabs(current))
    {
        step = accel_down * dt;
    }
    else
    {
        step = accel_up * dt;
    }
    if (diff > step)
        diff = step;
    else if (diff < -step)
        diff = -step;
    float next = current + diff;
    if ((current > 0 && next < 0) ||
        (current < 0 && next > 0))
    {
        next = 0;
    }
    return next;
}


uint8_t Chassis_Control_Init(MOTOR_Typdef *MOTOR)
{
    OmniInit(&OmniInit_t);
    float PID_F_0[3] = {0,0,0};
    float PID_S_0[3] = {4.0f,0.08f,0};
    float PID_F_1[3] = {0,0,0};
    float PID_S_1[3] = {4.0f,0.08f,0};
    float PID_F_2[3] = {0,0,0};
    float PID_S_2[3] = {4.0f,0.08f,0};
    float PID_F_3[3] = {0,0,0};
    float PID_S_3[3] = {4.0f,0.08f,0};
    float PID_Angle[3] = {8.0f,0,0};

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
    const float k1 = 1.9236442551e-02f;
    const float k2 = 1.7306169486e-01f;
    const float k3 = -1.0825006906e-03f;
    const float k4 = 1.9140042001e-05f;
    const float k5 = 8.6935008901e+00f;
    const float rpm_to_rad = 2.0f * 3.1415926f / 60.0f;

    float sum_wi = 0.0f;
    float sum_i2 = 0.0f;
    float sum_w  = 0.0f;
    float sum_w2 = 0.0f;

    for (int i = 0; i < 4; i++)
    {
        float w = MOTOR->DJI_3508_Chassis[i].DATA.Speed_now;
        float I = MOTOR->DJI_3508_Chassis[i].DATA.current * 0.001f;

        w = w * rpm_to_rad;

        sum_wi += w * I;
        sum_i2 += I * I;
        sum_w  += w;
        sum_w2 += w * w;
    }

    float p_dyn = k1 * sum_wi
                + k2 * sum_i2
                + k3 * sum_w
                + k4 * sum_w2
                + k5;
    return p_dyn;
}
static void Chassis_Power_Distribute(
    MOTOR_Typdef *MOTOR,
    float I_cmd[4],
    float P_limit)
{
    const float k1 = 1.9236442551e-02f;
    const float k2 = 1.7306169486e-01f;
    const float k3 = -1.0825006906e-03f;
    const float k4 = 1.9140042001e-05f;
    const float k5 = 8.6935008901e+00f;

    const float rpm_to_rad = 2.0f * 3.1415926f / 60.0f;

    float P_i[4];
    float P_sum = 0.0f;
    for(int i = 0; i < 4; i++)
    {
        float w = MOTOR->DJI_3508_Chassis[i].DATA.Speed_now * rpm_to_rad;
        float I = I_cmd[i] * 0.001f;

        float P = k1*w*I
                + k2*I*I
                + k3*w
                + k4*w*w
                + k5 * 0.25f;

        if(P < 0) P = 0;

        P_i[i] = P;
        P_sum += P;
    }

    if(P_sum <= P_limit)
        return;

    float scale = P_limit / P_sum;

    for(int i = 0; i < 4; i++)
    {
        float w = MOTOR->DJI_3508_Chassis[i].DATA.Speed_now * rpm_to_rad;
        float P_new = P_i[i] * scale;

        float C = k3*w + k4*w*w + k5*0.25f - P_new;
        float B = k1*w;
        float A = k2;

        float discriminant = B*B - 4*A*C;

        if(discriminant <= 0.0f)
        {
            I_cmd[i] = 0;
            continue;
        }

        float sqrt_d = sqrtf(discriminant);

        float I1 = (-B + sqrt_d) / (2*A);
        float I2 = (-B - sqrt_d) / (2*A);

        float I_old = I_cmd[i] * 0.001f;

        /* 选与原电流同号的解 */
        float I_final = (I_old >= 0) ?
                        (I1 >= 0 ? I1 : I2) :
                        (I1 < 0 ? I1 : I2);

        I_cmd[i] = I_final * 1000.0f;
    }
}

float m = 0;
float power_radio = 0;
void Chassis_Control_Task(MOTOR_Typdef *MOTOR)
{
    float wheel_rpm[4];
    float dt = 0.001f;

    float vx_target =  C_DBUS.Remote.CH0_int16 * 4.0f;
    float vy_target = -C_DBUS.Remote.CH1_int16 * 4.0f;
    float yaw_in = -(C_DBUS.Remote.CH2_int16 - C_DBUS.Remote.Dial_int16) / 10.0f;

    static float Chassis_Target_Yaw = 0.0f;
#define YAW_DEADZONE 5.0f

    float target_w_input = yaw_in * 6.0f;

    if (fabsf(yaw_in) < YAW_DEADZONE)
    {
        // ① 直接锁当前角度
        Chassis_Target_Yaw = IMU_Data.YawTotalAngle;

        target_w_input = 0;
    }
    else
    {
        Chassis_Target_Yaw += target_w_input * dt;
    }

    PID_Calculate(&PID_Chassis_Angle,
                  IMU_Data.YawTotalAngle,
                  Chassis_Target_Yaw);

    float vw_target = -PID_Chassis_Angle.Output;

    static float vx_set = 0;
    static float vy_set = 0;
    static float vw_set = 0;
    vx_set = Ramp_Control(vx_target, vx_set, 4000, 4000, dt);
    vy_set = Ramp_Control(vy_target, vy_set, 4000, 4000, dt);
    vw_set = Ramp_Control(vw_target, vw_set, 1000, 10000, dt);

    Omni_calc(wheel_rpm, vx_set, vy_set, vw_set, &OmniInit_t);

    for (uint8_t i = 0; i < 4; i++)
    {
        PID_Calculate(&MOTOR->DJI_3508_Chassis[i].PID_S,
                      MOTOR->DJI_3508_Chassis[i].DATA.Speed_now,
                      wheel_rpm[i]);
    }

    m = Chassis_Power_Model(MOTOR);

    float I_cmd[4];
    for(uint8_t i = 0; i < 4; i++)
    {
        I_cmd[i] = MOTOR->DJI_3508_Chassis[i].PID_S.Output;
    }
    float P_limit = 50.0f;
    Chassis_Power_Distribute(MOTOR, I_cmd, P_limit);
    DJI_Motor_Send(&hfdcan1,0x200,
                   I_cmd[0],
                   I_cmd[1],
                   I_cmd[2],
                   I_cmd[3]);
}