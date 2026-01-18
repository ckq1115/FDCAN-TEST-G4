//
// Created by CaoKangqi on 2026/1/19.
//
#include "pid.h"

void PID_Init(PID_t *pid, float kp, float ki, float kd, float integral_limit, float output_limit)
{
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;

    pid->Integral_Limit = integral_limit;
    pid->Output_Limit = output_limit;

    PID_Clear(pid);
}

float PID_Calculate(PID_t *pid, float target, float feedback)
{
    pid->Target = target;
    pid->Feedback = feedback;

    pid->Error = pid->Target - pid->Feedback;
    pid->Pout = pid->Kp * pid->Error;

    pid->Integral += pid->Error;
    if(pid->Integral > pid->Integral_Limit)
        pid->Integral = pid->Integral_Limit;
    else if(pid->Integral < -pid->Integral_Limit)
        pid->Integral = -pid->Integral_Limit;
    pid->Iout = pid->Ki * pid->Integral;

    pid->Derivative = pid->Error - pid->Error_Last;
    pid->Error_Last = pid->Error;
    pid->Dout = pid->Kd * pid->Derivative;

    pid->Output = pid->Pout + pid->Iout + pid->Dout;
    if(pid->Output > pid->Output_Limit)
        pid->Output = pid->Output_Limit;
    else if(pid->Output < -pid->Output_Limit)
        pid->Output = -pid->Output_Limit;

    return pid->Output;
}

void PID_Clear(PID_t *pid)
{
    pid->Target = 0.0f;
    pid->Feedback = 0.0f;

    pid->Error = 0.0f;
    pid->Error_Last = 0.0f;
    pid->Integral = 0.0f;
    pid->Derivative = 0.0f;

    pid->Output = 0.0f;
}