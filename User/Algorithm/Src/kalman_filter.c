#include "kalman_filter.h"
#include "FreeRTOS.h"
#include "task.h"

uint16_t sizeof_float, sizeof_double;

static void H_K_R_Adjustment(KalmanFilter_t *kf);

// Robust wrapper for malloc
void* user_malloc(size_t size)
{
    void* tmp = pvPortMalloc(size);
    if(tmp == NULL) {
        // Handle allocation failure (e.g., assert or infinite loop in debug)
        // configASSERT(0);
    }
    return tmp;
}

void* user_calloc(size_t nmemb, size_t size)
{
    void* tmp = user_malloc(nmemb * size);
    if (tmp != NULL) {
        memset(tmp, 0, nmemb * size);
    }
    return tmp;
}

void Kalman_Filter_Init(KalmanFilter_t *kf, uint8_t xhatSize, uint8_t uSize, uint8_t zSize)
{
    sizeof_float = sizeof(float);
    sizeof_double = sizeof(double);

    kf->xhatSize = xhatSize;
    kf->uSize = uSize;
    kf->zSize = zSize;

    kf->MeasurementValidNum = 0;

    // Use calloc to ensure zero initialization
    kf->MeasurementMap = (uint8_t *)user_calloc(zSize, sizeof(uint8_t));
    kf->MeasurementDegree = (float *)user_calloc(zSize, sizeof_float);
    kf->MatR_DiagonalElements = (float *)user_calloc(zSize, sizeof_float);
    kf->StateMinVariance = (float *)user_calloc(xhatSize, sizeof_float);
    kf->temp = (uint8_t *)user_calloc(zSize, sizeof(uint8_t));

    kf->FilteredValue = (float *)user_calloc(xhatSize, sizeof_float);
    kf->MeasuredVector = (float *)user_calloc(zSize, sizeof_float);
    kf->ControlVector = (float *)user_calloc(uSize, sizeof_float);

    // Matrix Data Allocation
    kf->xhat_data = (float *)user_calloc(xhatSize, sizeof_float);
    Matrix_Init(&kf->xhat, kf->xhatSize, 1, kf->xhat_data);

    kf->xhatminus_data = (float *)user_calloc(xhatSize, sizeof_float);
    Matrix_Init(&kf->xhatminus, kf->xhatSize, 1, kf->xhatminus_data);

    if (uSize > 0) {
        kf->u_data = (float *)user_calloc(uSize, sizeof_float);
        Matrix_Init(&kf->u, kf->uSize, 1, kf->u_data);

        kf->B_data = (float *)user_calloc(xhatSize * uSize, sizeof_float);
        Matrix_Init(&kf->B, kf->xhatSize, kf->uSize, kf->B_data);
    }

    kf->z_data = (float *)user_calloc(zSize, sizeof_float);
    Matrix_Init(&kf->z, kf->zSize, 1, kf->z_data);

    kf->P_data = (float *)user_calloc(xhatSize * xhatSize, sizeof_float);
    Matrix_Init(&kf->P, kf->xhatSize, kf->xhatSize, kf->P_data);

    kf->Pminus_data = (float *)user_calloc(xhatSize * xhatSize, sizeof_float);
    Matrix_Init(&kf->Pminus, kf->xhatSize, kf->xhatSize, kf->Pminus_data);

    kf->F_data = (float *)user_calloc(xhatSize * xhatSize, sizeof_float);
    kf->FT_data = (float *)user_calloc(xhatSize * xhatSize, sizeof_float);
    Matrix_Init(&kf->F, kf->xhatSize, kf->xhatSize, kf->F_data);
    Matrix_Init(&kf->FT, kf->xhatSize, kf->xhatSize, kf->FT_data);

    kf->H_data = (float *)user_calloc(zSize * xhatSize, sizeof_float);
    kf->HT_data = (float *)user_calloc(xhatSize * zSize, sizeof_float);
    Matrix_Init(&kf->H, kf->zSize, kf->xhatSize, kf->H_data);
    Matrix_Init(&kf->HT, kf->xhatSize, kf->zSize, kf->HT_data);

    kf->Q_data = (float *)user_calloc(xhatSize * xhatSize, sizeof_float);
    Matrix_Init(&kf->Q, kf->xhatSize, kf->xhatSize, kf->Q_data);

    kf->R_data = (float *)user_calloc(zSize * zSize, sizeof_float);
    Matrix_Init(&kf->R, kf->zSize, kf->zSize, kf->R_data);

    kf->K_data = (float *)user_calloc(xhatSize * zSize, sizeof_float);
    Matrix_Init(&kf->K, kf->xhatSize, kf->zSize, kf->K_data);

    // Temp matrices
    kf->S_data = (float *)user_calloc(kf->xhatSize * kf->xhatSize, sizeof_float);
    kf->temp_matrix_data = (float *)user_calloc(kf->xhatSize * kf->xhatSize, sizeof_float);
    kf->temp_matrix_data1 = (float *)user_calloc(kf->xhatSize * kf->xhatSize, sizeof_float);
    kf->temp_vector_data = (float *)user_calloc(kf->xhatSize, sizeof_float);
    kf->temp_vector_data1 = (float *)user_calloc(kf->xhatSize, sizeof_float);

    Matrix_Init(&kf->S, kf->xhatSize, kf->xhatSize, kf->S_data);
    Matrix_Init(&kf->temp_matrix, kf->xhatSize, kf->xhatSize, kf->temp_matrix_data);
    Matrix_Init(&kf->temp_matrix1, kf->xhatSize, kf->xhatSize, kf->temp_matrix_data1);
    Matrix_Init(&kf->temp_vector, kf->xhatSize, 1, kf->temp_vector_data);
    Matrix_Init(&kf->temp_vector1, kf->xhatSize, 1, kf->temp_vector_data1);

    kf->SkipEq1 = 0;
    kf->SkipEq2 = 0;
    kf->SkipEq3 = 0;
    kf->SkipEq4 = 0;
    kf->SkipEq5 = 0;
}

void Kalman_Filter_DeInit(KalmanFilter_t *kf)
{
    vPortFree(kf->MeasurementMap);
    vPortFree(kf->MeasurementDegree);
    vPortFree(kf->MatR_DiagonalElements);
    vPortFree(kf->StateMinVariance);
    vPortFree(kf->temp);
    vPortFree(kf->FilteredValue);
    vPortFree(kf->MeasuredVector);
    vPortFree(kf->ControlVector);
    vPortFree(kf->xhat_data);
    vPortFree(kf->xhatminus_data);
    if(kf->uSize > 0) {
        vPortFree(kf->u_data);
        vPortFree(kf->B_data);
    }
    vPortFree(kf->z_data);
    vPortFree(kf->P_data);
    vPortFree(kf->Pminus_data);
    vPortFree(kf->F_data);
    vPortFree(kf->FT_data);
    vPortFree(kf->H_data);
    vPortFree(kf->HT_data);
    vPortFree(kf->Q_data);
    vPortFree(kf->R_data);
    vPortFree(kf->K_data);
    vPortFree(kf->S_data);
    vPortFree(kf->temp_matrix_data);
    vPortFree(kf->temp_matrix_data1);
    vPortFree(kf->temp_vector_data);
    vPortFree(kf->temp_vector_data1);
}

void Kalman_Filter_Reset(KalmanFilter_t *kf, uint8_t xhatSize, uint8_t uSize, uint8_t zSize)
{
    // Clear all data buffers but keep allocations
    memset(kf->xhat_data, 0, sizeof_float * xhatSize);
    memset(kf->xhatminus_data, 0, sizeof_float * xhatSize);
    if(uSize > 0) memset(kf->u_data, 0, sizeof_float * uSize);
    memset(kf->z_data, 0, sizeof_float * zSize);
    memset(kf->P_data, 0, sizeof_float * xhatSize * xhatSize);
    memset(kf->Pminus_data, 0, sizeof_float * xhatSize * xhatSize);
    memset(kf->F_data, 0, sizeof_float * xhatSize * xhatSize);
    memset(kf->FT_data, 0, sizeof_float * xhatSize * xhatSize);
    if(uSize > 0) memset(kf->B_data, 0, sizeof_float * xhatSize * uSize);
    memset(kf->H_data, 0, sizeof_float * zSize * xhatSize);
    memset(kf->HT_data, 0, sizeof_float * xhatSize * zSize);
    memset(kf->Q_data, 0, sizeof_float * xhatSize * xhatSize);
    memset(kf->R_data, 0, sizeof_float * zSize * zSize);
    memset(kf->K_data, 0, sizeof_float * xhatSize * zSize);

    kf->SkipEq1 = 0; kf->SkipEq2 = 0; kf->SkipEq3 = 0; kf->SkipEq4 = 0; kf->SkipEq5 = 0;
}


void Kalman_Filter_Measure(KalmanFilter_t *kf)
{
    if (kf->UseAutoAdjustment != 0)
    {
        H_K_R_Adjustment(kf);
    }
    else
    {
        memcpy(kf->z_data, kf->MeasuredVector, sizeof_float * kf->zSize);
        memset(kf->MeasuredVector, 0, sizeof_float * kf->zSize);
    }

    if(kf->uSize > 0)
        memcpy(kf->u_data, kf->ControlVector, sizeof_float * kf->uSize);
}

void Kalman_Filter_xhatMinusUpdate(KalmanFilter_t *kf)
{
    if (!kf->SkipEq1)
    {
        if (kf->uSize > 0)
        {
            kf->temp_vector.numRows = kf->xhatSize;
            kf->temp_vector.numCols = 1;
            kf->MatStatus = Matrix_Multiply(&kf->F, &kf->xhat, &kf->temp_vector);

            kf->temp_vector1.numRows = kf->xhatSize;
            kf->temp_vector1.numCols = 1;
            kf->MatStatus = Matrix_Multiply(&kf->B, &kf->u, &kf->temp_vector1);

            kf->MatStatus = Matrix_Add(&kf->temp_vector, &kf->temp_vector1, &kf->xhatminus);
        }
        else
        {
            kf->MatStatus = Matrix_Multiply(&kf->F, &kf->xhat, &kf->xhatminus);
        }
    }
}

void Kalman_Filter_PminusUpdate(KalmanFilter_t *kf)
{
    if (!kf->SkipEq2)
    {
        kf->MatStatus = Matrix_Transpose(&kf->F, &kf->FT);
        kf->MatStatus = Matrix_Multiply(&kf->F, &kf->P, &kf->Pminus);

        kf->temp_matrix.numRows = kf->Pminus.numRows;
        kf->temp_matrix.numCols = kf->FT.numCols;
        kf->MatStatus = Matrix_Multiply(&kf->Pminus, &kf->FT, &kf->temp_matrix);

        kf->MatStatus = Matrix_Add(&kf->temp_matrix, &kf->Q, &kf->Pminus);
    }
}

void Kalman_Filter_SetK(KalmanFilter_t *kf)
{
    if (!kf->SkipEq3)
    {
        kf->MatStatus = Matrix_Transpose(&kf->H, &kf->HT);

        kf->temp_matrix.numRows = kf->H.numRows;
        kf->temp_matrix.numCols = kf->Pminus.numCols;
        kf->MatStatus = Matrix_Multiply(&kf->H, &kf->Pminus, &kf->temp_matrix); // H*P-

        kf->temp_matrix1.numRows = kf->temp_matrix.numRows;
        kf->temp_matrix1.numCols = kf->HT.numCols;
        kf->MatStatus = Matrix_Multiply(&kf->temp_matrix, &kf->HT, &kf->temp_matrix1); // H*P-*HT

        kf->S.numRows = kf->R.numRows;
        kf->S.numCols = kf->R.numCols;
        kf->MatStatus = Matrix_Add(&kf->temp_matrix1, &kf->R, &kf->S); // S = H*P-*HT + R

        kf->MatStatus = Matrix_Inverse(&kf->S, &kf->temp_matrix1); // inv(S)

        // If inverse fails (singular), K should probably be 0 or handle error
        if(kf->MatStatus == ARM_MATH_SUCCESS) {
            kf->temp_matrix.numRows = kf->Pminus.numRows;
            kf->temp_matrix.numCols = kf->HT.numCols;
            kf->MatStatus = Matrix_Multiply(&kf->Pminus, &kf->HT, &kf->temp_matrix); // P-*HT
            kf->MatStatus = Matrix_Multiply(&kf->temp_matrix, &kf->temp_matrix1, &kf->K);
        } else {
            memset(kf->K_data, 0, kf->xhatSize * kf->zSize * sizeof(float));
        }
    }
}

void Kalman_Filter_xhatUpdate(KalmanFilter_t *kf)
{
    if (!kf->SkipEq4)
    {
        kf->temp_vector.numRows = kf->H.numRows;
        kf->temp_vector.numCols = 1;
        kf->MatStatus = Matrix_Multiply(&kf->H, &kf->xhatminus, &kf->temp_vector); // H*x-

        kf->temp_vector1.numRows = kf->z.numRows;
        kf->temp_vector1.numCols = 1;
        kf->MatStatus = Matrix_Subtract(&kf->z, &kf->temp_vector, &kf->temp_vector1); // y = z - H*x-

        kf->temp_vector.numRows = kf->K.numRows;
        kf->temp_vector.numCols = 1;
        kf->MatStatus = Matrix_Multiply(&kf->K, &kf->temp_vector1, &kf->temp_vector); // K*y

        kf->MatStatus = Matrix_Add(&kf->xhatminus, &kf->temp_vector, &kf->xhat);
    }
}

void Kalman_Filter_P_Update(KalmanFilter_t *kf)
{
    if (!kf->SkipEq5)
    {
        kf->temp_matrix.numRows = kf->K.numRows;
        kf->temp_matrix.numCols = kf->H.numCols;
        kf->temp_matrix1.numRows = kf->temp_matrix.numRows;
        kf->temp_matrix1.numCols = kf->Pminus.numCols;

        kf->MatStatus = Matrix_Multiply(&kf->K, &kf->H, &kf->temp_matrix); // K*H
        kf->MatStatus = Matrix_Multiply(&kf->temp_matrix, &kf->Pminus, &kf->temp_matrix1); // K*H*P-
        kf->MatStatus = Matrix_Subtract(&kf->Pminus, &kf->temp_matrix1, &kf->P); // P = P- - K*H*P-
    }
}

float *Kalman_Filter_Update(KalmanFilter_t *kf)
{
    // 0. Prepare Measurement
    Kalman_Filter_Measure(kf);
    if (kf->User_Func0_f != NULL) kf->User_Func0_f(kf);

    // 1. Prediction (State)
    Kalman_Filter_xhatMinusUpdate(kf);
    if (kf->User_Func1_f != NULL) kf->User_Func1_f(kf);

    // 2. Prediction (Covariance)
    Kalman_Filter_PminusUpdate(kf);
    if (kf->User_Func2_f != NULL) kf->User_Func2_f(kf);

    if (kf->MeasurementValidNum != 0 || kf->UseAutoAdjustment == 0)
    {
        // 3. Calculate Gain
        Kalman_Filter_SetK(kf);
        if (kf->User_Func3_f != NULL) kf->User_Func3_f(kf);

        // 4. Update State
        Kalman_Filter_xhatUpdate(kf);
        if (kf->User_Func4_f != NULL) kf->User_Func4_f(kf);

        // 5. Update Covariance
        Kalman_Filter_P_Update(kf);
    }
    else
    {
        // No valid measurement: x = x-, P = P-
        memcpy(kf->xhat_data, kf->xhatminus_data, sizeof_float * kf->xhatSize);
        memcpy(kf->P_data, kf->Pminus_data, sizeof_float * kf->xhatSize * kf->xhatSize);
    }

    if (kf->User_Func5_f != NULL) kf->User_Func5_f(kf);

    // Min Variance Constraint
    for (uint8_t i = 0; i < kf->xhatSize; i++)
    {
        if (kf->P_data[i * kf->xhatSize + i] < kf->StateMinVariance[i])
            kf->P_data[i * kf->xhatSize + i] = kf->StateMinVariance[i];
    }

    memcpy(kf->FilteredValue, kf->xhat_data, sizeof_float * kf->xhatSize);

    if (kf->User_Func6_f != NULL) kf->User_Func6_f(kf);

    return kf->FilteredValue;
}

static void H_K_R_Adjustment(KalmanFilter_t *kf)
{
    kf->MeasurementValidNum = 0;
    memcpy(kf->z_data, kf->MeasuredVector, sizeof_float * kf->zSize);
    memset(kf->MeasuredVector, 0, sizeof_float * kf->zSize);

    memset(kf->R_data, 0, sizeof_float * kf->zSize * kf->zSize);
    memset(kf->H_data, 0, sizeof_float * kf->xhatSize * kf->zSize);

    for (uint8_t i = 0; i < kf->zSize; i++)
    {
        // Logic to determine if measurement is valid (user can customize this condition)
        if (1)
        {
            kf->z_data[kf->MeasurementValidNum] = kf->z_data[i];
            kf->temp[kf->MeasurementValidNum] = i;

            // Rebuild H row
            // Note: Be careful with indexing here to avoid overflow
            uint8_t col_idx = kf->MeasurementMap[i];
            if(col_idx > 0 && col_idx <= kf->xhatSize) {
                kf->H_data[kf->xhatSize * kf->MeasurementValidNum + (col_idx - 1)] = kf->MeasurementDegree[i];
            }

            kf->MeasurementValidNum++;
        }
    }

    for (uint8_t i = 0; i < kf->MeasurementValidNum; i++)
    {
        kf->R_data[i * kf->MeasurementValidNum + i] = kf->MatR_DiagonalElements[kf->temp[i]];
    }

    kf->H.numRows = kf->MeasurementValidNum;
    kf->H.numCols = kf->xhatSize;
    kf->HT.numRows = kf->xhatSize;
    kf->HT.numCols = kf->MeasurementValidNum;
    kf->R.numRows = kf->MeasurementValidNum;
    kf->R.numCols = kf->MeasurementValidNum;
    kf->K.numRows = kf->xhatSize;
    kf->K.numCols = kf->MeasurementValidNum;
    kf->z.numRows = kf->MeasurementValidNum;
}