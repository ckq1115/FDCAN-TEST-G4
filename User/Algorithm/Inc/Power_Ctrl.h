//
// Created by CaoKangqi on 2026/2/23.
//

#ifndef G4_FRAMEWORK_POWER_CTRL_H
#define G4_FRAMEWORK_POWER_CTRL_H

#include "All_Motor.h"
#include "main.h"
#include "user_lib.h"
#include "Power_CAP.h"
#include "Referee.h"

typedef struct
{
    float Kp;
    float Ki;
    float Kd;
    float ILt;
    float AlLt;
    float Error[2];
    float P_out;
    float I_out;
    float D_out;
    float All_out;

}PID_buffer_t;

typedef struct
{
    PID_buffer_t PID_Buffer;
    float scaled_give_power[4];///按比例分配后可供电机使用的电功率
    float k1;//kt：M3508鼙鼓的转矩常数Nm/A,对应机械功率项，与转速和电流乘积成正比
    float k2;//kr：M3508鼙鼓和C620电调的电阻Ω，对应铜损项，与电流平方成正比
    float k3;//k_iron：M3508电机铁损系数 (W/(rad/s)²)，对应铁损项（磁滞/涡流损耗），与转速平方成正比
    float k4;//k0：M3508电机和C620电调的静态功率W，对应固定损耗项，与转速和电流无关
    float rpm_to_rad;//RPM 转 rad/s
}model_t;

void Power_control_init(model_t *model);
uint8_t chassis_power_control(CONTAL_Typedef *RUI_V_CONTAL_V,
                           User_Data_T *usr_data,
                           model_t *model,
                           CAP_RXDATA *CAP_GET,
                           MOTOR_Typdef *MOTOR);

typedef struct {
    float shunt_volt;
    float bus_volt;
    float current;
    float power;
} Power_Typedef;

typedef struct {
    Power_Typedef P1, P2, P3, P4, P5;
} ALL_POWER_RX;

extern ALL_POWER_RX All_Power;

void CAN_POWER_Rx(Power_Typedef* pPower, uint8_t *rx_data);

#endif //G4_FRAMEWORK_POWER_CTRL_H