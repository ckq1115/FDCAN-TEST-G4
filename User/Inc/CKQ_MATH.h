//
// Created by CaoKangqi on 2026/1/25.
//

#ifndef FDCAN_TEST_G4_CKQ_MATH_H
#define FDCAN_TEST_G4_CKQ_MATH_H

#include "stdint.h"

// 已移除所有 RUI_F_ 前缀，仅保留后续的 MATH_ 相关标识
int16_t MATH_ABS_int16_t(int16_t DATA);
int32_t MATH_ABS_int32_t(int32_t DATA);
int64_t MATH_ABS_int64_t(int64_t DATA);
float MATH_ABS_float(float DATA);
float MATH_Limit_float(float MAX , float MIN , float DATA);
int16_t MATH_Limit_int16(int16_t MAX , int16_t MIN , int16_t DATA);
float MATH_INV_SQRT_float(float DATA);

//float uint_to_float(int16_t x_int, float span, int16_t value);
float Hex_To_Float(uint32_t *Byte,int num);
uint32_t FloatTohex(float HEX);
int float_to_uint(float x_float, float x_min, float x_max, int bits);
float uint_to_float(int x_int, float x_min, float x_max, int bits);
float get_vbus_input(uint16_t value);
void convertAngleToIndex(float angle, float *index) ;

typedef struct
{
    uint16_t adc_val[2];
    float vbus;
}V_Input_t;

#endif //FDCAN_TEST_G4_CKQ_MATH_H