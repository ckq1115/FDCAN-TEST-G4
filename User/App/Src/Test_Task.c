//
// Created by CaoKangqi on 2026/2/27.
//
#include "Test_Task.h"

#include "All_Init.h"

TD_t TD_Pitch;
LDOB_t LDOB_Pitch;

/**
 * @brief 位置环 PID 钩子函数：集成 TD 平滑与速度前馈
 * @note  挂载在 PID_P 的 User_Func1_f 指针上
 */
void Pitch_Pos_TD_Hook(PID_t *pid)
{
    TD_Calculate(&TD_Pitch, pid->Ref);
    pid->Ref = TD_Pitch.x;

    pid->Err = pid->Ref - pid->Measure;

    float k_ff = 0.8f;
    pid->Pout += TD_Pitch.dx * k_ff;
}
void Test_Init(void)
{
    float PID_P_0[3] = {1.0f,0.005f,0};
    float PID_S_0[3] = {12.0f,0.005f,0};
    PID_Init(&All_Motor.DJI_6020_Pitch.PID_P, 300.0f, 100.0f,
             PID_P_0, 0, 0,
             0, 0, 0,
             Integral_Limit|ErrorHandle//积分限幅,输出滤波,堵转监测
             //梯形积分,变速积分
             );//微分先行,微分滤波器
    All_Motor.DJI_6020_Pitch.PID_P.User_Func1_f = Pitch_Pos_TD_Hook;
    PID_Init(&All_Motor.DJI_6020_Pitch.PID_S, 16384.0f, 1000.0f,
             PID_S_0, 0, 0,
             0, 0, 0,
             Integral_Limit|ErrorHandle//积分限幅,输出滤波,堵转监测
             //梯形积分,变速积分
             );//微分先行,微分滤波器
    TD_Init(&TD_Pitch, 38000, 0.005f);
    float LDOB_c[3] = {0.0f, 0.01f, 0.001f}; // c0, c1, c2 (此处 c2 对应惯量补偿)

    LDOB_Init(&LDOB_Pitch,
              5000.0f,   // Max_Disturbance: 最大补偿电流限幅
              0.0f,     // DeadBand: 5% 的输出死区
              LDOB_c,    // 模型参数
              0.01f,     // LPF_RC: 低通滤波时间常数
              5,         // 一阶微分 OLS 阶数
              5);        // 二阶微分 OLS 阶数
}
float Target = 0;

float Sine_Amplitude = 2000.0f;  // 振幅 (根据你的单位调整，例如45度)
float Sine_Frequency = 2.0f;   // 频率 (单位：Hz，即每秒钟往返一次)
float Sine_Offset = 0.0f;      // 中心偏移量
float Sine_Time = 0.0f;        // 时间累积变量
float Control_Freq = 1000.0f;  // 任务执行频率 (如果是1ms运行一次，就是1000Hz)
float ldob_comp = 0;
void Ctrl_Test_Task(void)
{
    float Pitch_raw = C_DBUS.Remote.CH2_int16;
    static float Pitch_Target = 0;
    Pitch_Target += Pitch_raw * 0.01f;

    /*Target = Sine_Amplitude * sinf(2.0f * 3.1415926f * Sine_Frequency * Sine_Time) + Sine_Offset;
    Sine_Time += (1.0f / Control_Freq);*/
    if(Sine_Time > 100.0f) Sine_Time = 0.0f;
    PID_Calculate(&All_Motor.DJI_6020_Pitch.PID_P,
                  All_Motor.DJI_6020_Pitch.DATA.Angle_Infinite,
                  Target);
    PID_Calculate(&All_Motor.DJI_6020_Pitch.PID_S,
                  All_Motor.DJI_6020_Pitch.DATA.Speed_now,
                  All_Motor.DJI_6020_Pitch.PID_P.Output);

    ldob_comp = LDOB_Calculate(&LDOB_Pitch,
                                     All_Motor.DJI_6020_Pitch.DATA.Angle_Infinite,
                                     All_Motor.DJI_6020_Pitch.PID_S.Output);

    /* 4. 最终输出 = PID 输出 + LDOB 补偿输出 */
    // 注意：补偿方向取决于你的系统建模，通常是相加来抵消干扰
    float Final_Output = All_Motor.DJI_6020_Pitch.PID_S.Output - ldob_comp;

    // 限幅并发送
    Final_Output = float_constrain(Final_Output, -16384.0f, 16384.0f);
        DJI_Motor_Send(&hfdcan3,0x1FE,All_Motor.DJI_6020_Pitch.PID_S.Output,
                        0,All_Motor.DJI_6020_Pitch.PID_S.Output, 0);

}
