//
// Created by CaoKangqi on 2026/1/30.
// Fixed & Enhanced by Gemini
//
#include "QuaternionEKF.h"
#include "math.h"
#include <string.h>

#ifndef Sqrt
#define Sqrt sqrtf
#endif

QEKF_INS_t QEKF_INS = {0};

// State: [q0, q1, q2, q3, bias_x, bias_y]
const float IMU_QuaternionEKF_F[36] = {
    1, 0, 0, 0, 0, 0,
    0, 1, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0,
    0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 1
};

const float IMU_QuaternionEKF_P_Const[36] = {
    1000, 0, 0, 0, 0, 0,
    0, 1000, 0, 0, 0, 0,
    0, 0, 1000, 0, 0, 0,
    0, 0, 0, 1000, 0, 0,
    0, 0, 0, 0, 100, 0,
    0, 0, 0, 0, 0, 100
};

static float invSqrt(float x);
static void IMU_QuaternionEKF_Observe(KalmanFilter_t *kf);
static void IMU_QuaternionEKF_F_Linearization_P_Fading(KalmanFilter_t *kf);
static void IMU_QuaternionEKF_SetH(KalmanFilter_t *kf);
static void IMU_QuaternionEKF_CalculateGainAndResidual(KalmanFilter_t *kf);

// ---------------- Implementation ----------------

void IMU_QuaternionEKF_Init(float process_noise1, float process_noise2, float measure_noise, float lambda, float dt, float lpf)
{
    QEKF_INS.Initialized = 1;
    QEKF_INS.Q1 = process_noise1;
    QEKF_INS.Q2 = process_noise2;
    QEKF_INS.R = measure_noise;

    QEKF_INS.ZuptAccelThresh = 0.5f;
    QEKF_INS.ZuptGyroThresh = 0.1f;

    QEKF_INS.ChiSquareTestThreshold = 9.81f;
    QEKF_INS.ConvergeFlag = 0;
    QEKF_INS.ErrorCount = 0;
    QEKF_INS.UpdateCount = 0;
    QEKF_INS.dt = dt;
    QEKF_INS.accLPFcoef = lpf;

    if (lambda > 1) lambda = 1;
    QEKF_INS.lambda = lambda;

    // Init Kalman Filter: 6 States, 0 Controls, 3 Measurements
    Kalman_Filter_Init(&QEKF_INS.IMU_QuaternionEKF, 6, 0, 3);

    // Initial State: q=[1,0,0,0], bias=[0,0]
    QEKF_INS.IMU_QuaternionEKF.xhat_data[0] = 1;

    // Register Hooks
    // Func0: Debug or pre-calc
    QEKF_INS.IMU_QuaternionEKF.User_Func0_f = IMU_QuaternionEKF_Observe;
    // Func1: Post-State-Prediction. We do P fading here.
    QEKF_INS.IMU_QuaternionEKF.User_Func1_f = IMU_QuaternionEKF_F_Linearization_P_Fading;
    // Func2: Post-Covariance-Prediction. We calc H here (using x_pred).
    QEKF_INS.IMU_QuaternionEKF.User_Func2_f = IMU_QuaternionEKF_SetH;
    // Func3: Replace Gain Calculation. We calculate K, Innovation, and State Update here.
    QEKF_INS.IMU_QuaternionEKF.User_Func3_f = IMU_QuaternionEKF_CalculateGainAndResidual;

    // We skip standard Eq3 (K) and Eq4 (x update) because Func3 handles both for EKF non-linear residual
    QEKF_INS.IMU_QuaternionEKF.SkipEq3 = TRUE;
    QEKF_INS.IMU_QuaternionEKF.SkipEq4 = TRUE;

    memcpy(QEKF_INS.IMU_QuaternionEKF.F_data, IMU_QuaternionEKF_F, sizeof(IMU_QuaternionEKF_F));
    memcpy(QEKF_INS.IMU_QuaternionEKF.P_data, IMU_QuaternionEKF_P_Const, sizeof(IMU_QuaternionEKF_P_Const));
}

void IMU_QuaternionEKF_Reset(void)
{
    Kalman_Filter_Reset(&QEKF_INS.IMU_QuaternionEKF, 6, 0, 3);

    QEKF_INS.IMU_QuaternionEKF.xhat_data[0] = 1;

    QEKF_INS.IMU_QuaternionEKF.SkipEq3 = TRUE;
    QEKF_INS.IMU_QuaternionEKF.SkipEq4 = TRUE;

    memcpy(QEKF_INS.IMU_QuaternionEKF.F_data, IMU_QuaternionEKF_F, sizeof(IMU_QuaternionEKF_F));
    memcpy(QEKF_INS.IMU_QuaternionEKF.P_data, IMU_QuaternionEKF_P_Const, sizeof(IMU_QuaternionEKF_P_Const));

    QEKF_INS.YawRoundCount = 0;
    QEKF_INS.YawAngleLast = 0;
}

void IMU_QuaternionEKF_Update(float gx, float gy, float gz, float ax, float ay, float az)
{
    float halfgxdt, halfgydt, halfgzdt;

    // 1. Get current bias estimates
    QEKF_INS.GyroBias[0] = QEKF_INS.IMU_QuaternionEKF.xhat_data[4];
    QEKF_INS.GyroBias[1] = QEKF_INS.IMU_QuaternionEKF.xhat_data[5];
    QEKF_INS.GyroBias[2] = 0;

    // 2. Unbias Gyro
    QEKF_INS.Gyro[0] = gx - QEKF_INS.GyroBias[0];
    QEKF_INS.Gyro[1] = gy - QEKF_INS.GyroBias[1];
    QEKF_INS.Gyro[2] = gz - QEKF_INS.GyroBias[2];

    QEKF_INS.gyro_norm = Sqrt(QEKF_INS.Gyro[0]*QEKF_INS.Gyro[0] +
                              QEKF_INS.Gyro[1]*QEKF_INS.Gyro[1] +
                              QEKF_INS.Gyro[2]*QEKF_INS.Gyro[2]);

    // 3. LPF Accel
    if (QEKF_INS.UpdateCount == 0) {
        QEKF_INS.Accel[0] = ax; QEKF_INS.Accel[1] = ay; QEKF_INS.Accel[2] = az;
        QEKF_INS.UpdateCount++;
    } else {
        float k = QEKF_INS.dt / (QEKF_INS.dt + QEKF_INS.accLPFcoef);
        QEKF_INS.Accel[0] += k * (ax - QEKF_INS.Accel[0]);
        QEKF_INS.Accel[1] += k * (ay - QEKF_INS.Accel[1]);
        QEKF_INS.Accel[2] += k * (az - QEKF_INS.Accel[2]);
    }

    QEKF_INS.accl_norm = Sqrt(QEKF_INS.Accel[0]*QEKF_INS.Accel[0] +
                              QEKF_INS.Accel[1]*QEKF_INS.Accel[1] +
                              QEKF_INS.Accel[2]*QEKF_INS.Accel[2]);

    // 4. Update F Matrix (Linearization based on Gyro)
    // We update F *before* the filter step so the Prediction step (Eq1) uses the new F
    halfgxdt = 0.5f * QEKF_INS.Gyro[0] * QEKF_INS.dt;
    halfgydt = 0.5f * QEKF_INS.Gyro[1] * QEKF_INS.dt;
    halfgzdt = 0.5f * QEKF_INS.Gyro[2] * QEKF_INS.dt;

    memcpy(QEKF_INS.IMU_QuaternionEKF.F_data, IMU_QuaternionEKF_F, sizeof(IMU_QuaternionEKF_F));

    QEKF_INS.IMU_QuaternionEKF.F_data[1] = -halfgxdt;
    QEKF_INS.IMU_QuaternionEKF.F_data[2] = -halfgydt;
    QEKF_INS.IMU_QuaternionEKF.F_data[3] = -halfgzdt;

    QEKF_INS.IMU_QuaternionEKF.F_data[6] = halfgxdt;
    QEKF_INS.IMU_QuaternionEKF.F_data[8] = halfgzdt;
    QEKF_INS.IMU_QuaternionEKF.F_data[9] = -halfgydt;

    QEKF_INS.IMU_QuaternionEKF.F_data[12] = halfgydt;
    QEKF_INS.IMU_QuaternionEKF.F_data[13] = -halfgzdt;
    QEKF_INS.IMU_QuaternionEKF.F_data[15] = halfgxdt;

    QEKF_INS.IMU_QuaternionEKF.F_data[18] = halfgzdt;
    QEKF_INS.IMU_QuaternionEKF.F_data[19] = halfgydt;
    QEKF_INS.IMU_QuaternionEKF.F_data[20] = -halfgxdt;

    // 5. ZUPT / Motion Logic
    if (QEKF_INS.gyro_norm < QEKF_INS.ZuptGyroThresh &&
        fabsf(QEKF_INS.accl_norm - 9.81f) < QEKF_INS.ZuptAccelThresh)
    {
        QEKF_INS.StableFlag = 1;
    } else {
        QEKF_INS.StableFlag = 0;
    }

    // 6. Set Measurement Vector
    // Crucial: Only update if gravity is valid (not freefall or extreme G)
    int valid_measure = 0;
    if (QEKF_INS.accl_norm > 2.0f && QEKF_INS.accl_norm < 18.0f) // Tolerance around 9.81
    {
        float accelInvNorm = 1.0f / QEKF_INS.accl_norm;
        QEKF_INS.IMU_QuaternionEKF.MeasuredVector[0] = QEKF_INS.Accel[0] * accelInvNorm;
        QEKF_INS.IMU_QuaternionEKF.MeasuredVector[1] = QEKF_INS.Accel[1] * accelInvNorm;
        QEKF_INS.IMU_QuaternionEKF.MeasuredVector[2] = QEKF_INS.Accel[2] * accelInvNorm;

        // Since we are not using AutoAdjustment, we set MeasurementValidNum manually to force update
        // However, the generic driver logic uses MeasurementValidNum for resizing only if AutoAdjust is on.
        // If AutoAdjust is OFF, it assumes all Z are valid.
        valid_measure = 1;
    }
    else
    {
        // Skip measurement update if invalid gravity
         valid_measure = 0;
    }

    // 7. Dynamic Noise Adjustment
    float current_Q2 = QEKF_INS.Q2;
    float current_R = QEKF_INS.R;

    if (QEKF_INS.StableFlag) {
        current_R = QEKF_INS.R * 0.5f;
        current_Q2 = QEKF_INS.Q2 * 100.0f; // Allow bias to converge
    } else {
        // Increase R significantly if acceleration is not 1G
        float acc_err = fabsf(QEKF_INS.accl_norm - 9.81f);
        current_R = QEKF_INS.R + acc_err * acc_err * 2000.0f;
        current_Q2 = QEKF_INS.Q2;
    }

    // Set Q
    QEKF_INS.IMU_QuaternionEKF.Q_data[0] = QEKF_INS.Q1 * QEKF_INS.dt;
    QEKF_INS.IMU_QuaternionEKF.Q_data[7] = QEKF_INS.Q1 * QEKF_INS.dt;
    QEKF_INS.IMU_QuaternionEKF.Q_data[14] = QEKF_INS.Q1 * QEKF_INS.dt;
    QEKF_INS.IMU_QuaternionEKF.Q_data[21] = QEKF_INS.Q1 * QEKF_INS.dt;
    QEKF_INS.IMU_QuaternionEKF.Q_data[28] = current_Q2 * QEKF_INS.dt;
    QEKF_INS.IMU_QuaternionEKF.Q_data[35] = current_Q2 * QEKF_INS.dt;

    // Set R
    QEKF_INS.IMU_QuaternionEKF.R_data[0] = current_R;
    QEKF_INS.IMU_QuaternionEKF.R_data[4] = current_R;
    QEKF_INS.IMU_QuaternionEKF.R_data[8] = current_R;

    // 8. Run Filter
    if(valid_measure) {
        // Standard Update
        Kalman_Filter_Update(&QEKF_INS.IMU_QuaternionEKF);
    } else {
        // Only Predict steps
        Kalman_Filter_Measure(&QEKF_INS.IMU_QuaternionEKF); // Clear input
        Kalman_Filter_xhatMinusUpdate(&QEKF_INS.IMU_QuaternionEKF);
        if (QEKF_INS.IMU_QuaternionEKF.User_Func1_f) QEKF_INS.IMU_QuaternionEKF.User_Func1_f(&QEKF_INS.IMU_QuaternionEKF);
        Kalman_Filter_PminusUpdate(&QEKF_INS.IMU_QuaternionEKF);
        // Skip Update steps, just copy to Output
        memcpy(QEKF_INS.IMU_QuaternionEKF.xhat_data, QEKF_INS.IMU_QuaternionEKF.xhatminus_data, sizeof_float * 6);
        memcpy(QEKF_INS.IMU_QuaternionEKF.P_data, QEKF_INS.IMU_QuaternionEKF.Pminus_data, sizeof_float * 36);
        memcpy(QEKF_INS.IMU_QuaternionEKF.FilteredValue, QEKF_INS.IMU_QuaternionEKF.xhat_data, sizeof_float * 6);
    }

    // 9. Output Extraction
    QEKF_INS.q[0] = QEKF_INS.IMU_QuaternionEKF.FilteredValue[0];
    QEKF_INS.q[1] = QEKF_INS.IMU_QuaternionEKF.FilteredValue[1];
    QEKF_INS.q[2] = QEKF_INS.IMU_QuaternionEKF.FilteredValue[2];
    QEKF_INS.q[3] = QEKF_INS.IMU_QuaternionEKF.FilteredValue[3];

    // Normalize
    float qNorm = invSqrt(QEKF_INS.q[0]*QEKF_INS.q[0] + QEKF_INS.q[1]*QEKF_INS.q[1] +
                          QEKF_INS.q[2]*QEKF_INS.q[2] + QEKF_INS.q[3]*QEKF_INS.q[3]);
    if(isinf(qNorm) || isnan(qNorm)) qNorm = 1.0f; // Safety

    QEKF_INS.q[0] *= qNorm;
    QEKF_INS.q[1] *= qNorm;
    QEKF_INS.q[2] *= qNorm;
    QEKF_INS.q[3] *= qNorm;

    // Feedback normalized q to state
    QEKF_INS.IMU_QuaternionEKF.xhat_data[0] = QEKF_INS.q[0];
    QEKF_INS.IMU_QuaternionEKF.xhat_data[1] = QEKF_INS.q[1];
    QEKF_INS.IMU_QuaternionEKF.xhat_data[2] = QEKF_INS.q[2];
    QEKF_INS.IMU_QuaternionEKF.xhat_data[3] = QEKF_INS.q[3];

    // Euler Calculation
    QEKF_INS.Roll = atan2f(2.0f*(QEKF_INS.q[0]*QEKF_INS.q[1] + QEKF_INS.q[2]*QEKF_INS.q[3]),
                           1.0f - 2.0f*(QEKF_INS.q[1]*QEKF_INS.q[1] + QEKF_INS.q[2]*QEKF_INS.q[2])) * 57.29578f;

    float sinp = -2.0f * (QEKF_INS.q[1]*QEKF_INS.q[3] - QEKF_INS.q[0]*QEKF_INS.q[2]);
    if (fabsf(sinp) >= 1.0f) sinp = copysignf(1.0f, sinp);
    QEKF_INS.Pitch = asinf(sinp) * 57.29578f;

    QEKF_INS.Yaw = atan2f(2.0f*(QEKF_INS.q[1]*QEKF_INS.q[2] + QEKF_INS.q[0]*QEKF_INS.q[3]),
                          1.0f - 2.0f*(QEKF_INS.q[2]*QEKF_INS.q[2] + QEKF_INS.q[3]*QEKF_INS.q[3])) * 57.29578f;

    if (QEKF_INS.Yaw - QEKF_INS.YawAngleLast > 180.0f) {
        QEKF_INS.YawRoundCount--;
    } else if (QEKF_INS.Yaw - QEKF_INS.YawAngleLast < -180.0f) {
        QEKF_INS.YawRoundCount++;
    }
    QEKF_INS.YawTotalAngle = 360.0f * QEKF_INS.YawRoundCount + QEKF_INS.Yaw;
    QEKF_INS.YawAngleLast = QEKF_INS.Yaw;
}

static void IMU_QuaternionEKF_F_Linearization_P_Fading(KalmanFilter_t *kf)
{
    float q0 = kf->xhatminus_data[0];
    float q1 = kf->xhatminus_data[1];
    float q2 = kf->xhatminus_data[2];
    float q3 = kf->xhatminus_data[3];

    // Note: F was largely set in Update() based on Gyro.
    // Here we add the coupling terms for bias (Partial deriv of q wrt bias)
    // dq/dt = ... - 0.5 * q * bias

    kf->F_data[4]  = q1 * QEKF_INS.dt * 0.5f;
    kf->F_data[5]  = q2 * QEKF_INS.dt * 0.5f;

    kf->F_data[10] = -q0 * QEKF_INS.dt * 0.5f;
    kf->F_data[11] = q3 * QEKF_INS.dt * 0.5f;

    kf->F_data[16] = -q3 * QEKF_INS.dt * 0.5f;
    kf->F_data[17] = -q0 * QEKF_INS.dt * 0.5f;

    kf->F_data[22] = q2 * QEKF_INS.dt * 0.5f;
    kf->F_data[23] = -q1 * QEKF_INS.dt * 0.5f;

    // Fading Memory
    kf->P_data[28] /= QEKF_INS.lambda;
    kf->P_data[35] /= QEKF_INS.lambda;

    if (kf->P_data[28] > 10000.0f) kf->P_data[28] = 10000.0f;
    if (kf->P_data[35] > 10000.0f) kf->P_data[35] = 10000.0f;
}

static void IMU_QuaternionEKF_SetH(KalmanFilter_t *kf)
{
    // H = dh/dx
    // h(x) is gravity vector in body frame based on predicted q

    float q0 = kf->xhatminus_data[0];
    float q1 = kf->xhatminus_data[1];
    float q2 = kf->xhatminus_data[2];
    float q3 = kf->xhatminus_data[3];

    float doubleq0 = 2.0f * q0;
    float doubleq1 = 2.0f * q1;
    float doubleq2 = 2.0f * q2;
    float doubleq3 = 2.0f * q3;

    memset(kf->H_data, 0, sizeof_float * kf->zSize * kf->xhatSize);

    // X
    kf->H_data[0] = -doubleq2* 9.81f;
    kf->H_data[1] = doubleq3* 9.81f;
    kf->H_data[2] = -doubleq0* 9.81f;
    kf->H_data[3] = doubleq1* 9.81f;

    // Y
    kf->H_data[6] = doubleq1* 9.81f;
    kf->H_data[7] = doubleq0* 9.81f;
    kf->H_data[8] = doubleq3* 9.81f;
    kf->H_data[9] = doubleq2* 9.81f;

    // Z
    kf->H_data[12] = doubleq0* 9.81f;
    kf->H_data[13] = -doubleq1* 9.81f;
    kf->H_data[14] = -doubleq2* 9.81f;
    kf->H_data[15] = doubleq3* 9.81f;
}

// Replaces Eq3 and Eq4 (Calculation of K, Residual, and State Update)
static void IMU_QuaternionEKF_CalculateGainAndResidual(KalmanFilter_t *kf)
{
    float q0, q1, q2, q3;

    // 1. Calculate K = P * H^T * inv(H * P * H^T + R)
    kf->MatStatus = Matrix_Transpose(&kf->H, &kf->HT);
    kf->MatStatus = Matrix_Multiply(&kf->H, &kf->Pminus, &kf->temp_matrix); // H * P-
    kf->MatStatus = Matrix_Multiply(&kf->temp_matrix, &kf->HT, &kf->temp_matrix1); // H * P- * HT
    kf->MatStatus = Matrix_Add(&kf->temp_matrix1, &kf->R, &kf->S); // S

    // Invert S
    kf->MatStatus = Matrix_Inverse(&kf->S, &kf->temp_matrix1); // temp1 = inv(S)

    if (kf->MatStatus != ARM_MATH_SUCCESS) {
        // Singular matrix, skip update to avoid NaN
        kf->SkipEq5 = TRUE; // Skip P update too
        return;
    }

    kf->MatStatus = Matrix_Multiply(&kf->Pminus, &kf->HT, &kf->temp_matrix); // P- * HT
    kf->MatStatus = Matrix_Multiply(&kf->temp_matrix, &kf->temp_matrix1, &kf->K); // K

    // 2. Innovation (Residual) y = z - h(x-)
    q0 = kf->xhatminus_data[0];
    q1 = kf->xhatminus_data[1];
    q2 = kf->xhatminus_data[2];
    q3 = kf->xhatminus_data[3];

    // Predicted Gravity
    kf->temp_vector.pData[0] = 2.0f * (q1 * q3 - q0 * q2) * 9.81f;
    kf->temp_vector.pData[1] = 2.0f * (q0 * q1 + q2 * q3) * 9.81f;
    kf->temp_vector.pData[2] = (q0*q0 - q1*q1 - q2*q2 + q3*q3) * 9.81f;
    // Bias parts of temp_vector are 0 since H for bias is 0

    // y = z - h(x)
    // Note: z is already populated in kf->z by Kalman_Filter_Measure
    kf->temp_vector1.numRows = kf->z.numRows;
    kf->temp_vector1.numCols = 1;
    kf->MatStatus = Matrix_Subtract(&kf->z, &kf->temp_vector, &kf->temp_vector1);

    // 3. Chi-Square Test & State Update
    // y^T * inv(S) * y
    // Reuse temp_matrix (P*HT) for scratchpad isn't safe, use separate buffer logic or loops

    float chi_val = 0.0f;
    // Compute invS * y -> temp_matrix (reuse: it's 6x6, we use 3x1)
    kf->temp_matrix.numRows = 3;
    kf->temp_matrix.numCols = 1;
    // temp_matrix1 is inv(S)
    kf->MatStatus = Matrix_Multiply(&kf->temp_matrix1, &kf->temp_vector1, &kf->temp_matrix);

    for(int i=0; i<3; i++) {
        chi_val += kf->temp_vector1.pData[i] * kf->temp_matrix.pData[i];
    }

    QEKF_INS.ChiSquare_Data[0] = chi_val;

    // Divergence Check
    if (chi_val > QEKF_INS.ChiSquareTestThreshold && QEKF_INS.ConvergeFlag)
    {
        QEKF_INS.ErrorCount++;
        if (QEKF_INS.ErrorCount > 50) {
            QEKF_INS.ConvergeFlag = 0; // Reset flag, accept measurement next time
            kf->SkipEq5 = FALSE;
        } else {
            // Ignore this measurement
            kf->SkipEq5 = TRUE; // Skip P update
            return;
        }
    }
    else
    {
        QEKF_INS.ConvergeFlag = 1;
        QEKF_INS.ErrorCount = 0;
        kf->SkipEq5 = FALSE;

        // Adaptive gain scaling
        if (chi_val > 0.1f * QEKF_INS.ChiSquareTestThreshold) {
             QEKF_INS.AdaptiveGainScale = (QEKF_INS.ChiSquareTestThreshold - chi_val) / (0.9f * QEKF_INS.ChiSquareTestThreshold);
             if(QEKF_INS.AdaptiveGainScale < 0) QEKF_INS.AdaptiveGainScale = 0;
        } else {
             QEKF_INS.AdaptiveGainScale = 1.0f;
        }

        int k_size = kf->K.numRows * kf->K.numCols;
        for (int i = 0; i < k_size; i++) {
            kf->K_data[i] *= QEKF_INS.AdaptiveGainScale;
        }
    }

    // 4. Update State x = x- + K * y
    kf->temp_vector.numRows = kf->K.numRows;
    kf->temp_vector.numCols = 1;
    kf->MatStatus = Matrix_Multiply(&kf->K, &kf->temp_vector1, &kf->temp_vector); // delta_x

    kf->MatStatus = Matrix_Add(&kf->xhatminus, &kf->temp_vector, &kf->xhat);
}

static void IMU_QuaternionEKF_Observe(KalmanFilter_t *kf)
{
    // Optional: Copy internal state to external debug structures if needed
}

float Get_Pitch() { return QEKF_INS.Pitch; }
float Get_Roll() { return QEKF_INS.Roll; }
float Get_Yaw() { return QEKF_INS.Yaw; }
float Get_YawTotalAngle() { return QEKF_INS.YawTotalAngle; }

static float invSqrt(float x)
{
    if (x <= 0.0f) return 0.0f;
    return 1.0f / sqrtf(x);
}