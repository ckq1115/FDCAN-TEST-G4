//
// Created by CaoKangqi on 2026/1/19.
//

#ifndef FDCAN_TEST_G4_PID_H
#define FDCAN_TEST_G4_PID_H

typedef struct
{
    float Kp;
    float Ki;
    float Kd;

    float Target;
    float Feedback;

    float Error;
    float Error_Last;
    float Integral;
    float Derivative;

    float Integral_Limit;
    float Output_Limit;

    float Pout;
    float Iout;
    float Dout;
    float Output;
} PID_t;

void PID_Init(PID_t *pid, float kp, float ki, float kd, float integral_limit, float output_limit);
float PID_Calculate(PID_t *pid, float target, float feedback);
void PID_Clear(PID_t *pid);

#endif //FDCAN_TEST_G4_PID_H