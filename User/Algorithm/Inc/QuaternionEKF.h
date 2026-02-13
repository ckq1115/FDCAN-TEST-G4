//
// Created by CaoKangqi on 2026/1/30.
// Fixed & Enhanced by Gemini
//

#ifndef FDCAN_TEST_G4_QUATERNIONEKF_H
#define FDCAN_TEST_G4_QUATERNIONEKF_H

#include "kalman_filter.h"

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

typedef struct
{
    uint8_t Initialized;
    KalmanFilter_t IMU_QuaternionEKF;

    uint8_t ConvergeFlag;
    uint8_t StableFlag;      // ZUPT trigger
    uint64_t ErrorCount;
    uint64_t UpdateCount;

    float q[4];        // [w, x, y, z]
    float GyroBias[3]; // [x, y, z]

    float Gyro[3];     // Unbiased gyro
    float Accel[3];    // Filtered accel

    float accLPFcoef;
    float gyro_norm;
    float accl_norm;
    float AdaptiveGainScale;

    // Euler Angles
    float Roll;
    float Pitch;
    float Yaw;
    float YawTotalAngle;
    int16_t YawRoundCount;
    float YawAngleLast;

    // Noise parameters
    float Q1; // Quaternion Process Noise
    float Q2; // Bias Process Noise
    float R;  // Measure Noise

    // ZUPT Parameters
    float ZuptAccelThresh;
    float ZuptGyroThresh;

    float dt;

    // Chi-Square
    float ChiSquare_Data[1]; // Storage for result
    float ChiSquareTestThreshold;
    float lambda; // Fading factor

} QEKF_INS_t;

extern QEKF_INS_t QEKF_INS;

void IMU_QuaternionEKF_Init(float process_noise1, float process_noise2, float measure_noise, float lambda, float dt, float lpf);
void IMU_QuaternionEKF_Update(float gx, float gy, float gz, float ax, float ay, float az);
void IMU_QuaternionEKF_Reset(void);

float Get_Pitch(void);
float Get_Roll(void);
float Get_Yaw(void);
float Get_YawTotalAngle(void);

#endif //FDCAN_TEST_G4_QUATERNIONEKF_H