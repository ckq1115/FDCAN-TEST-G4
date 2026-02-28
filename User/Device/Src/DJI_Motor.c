//
// Created by CaoKangqi on 2026/2/14.
//
#include "DJI_Motor.h"
#include "All_define.h"

/**
 * @brief DJI 电机协议解析内核 (3508/2006/6020 通用)
 */
void DJI_Motor_Resolve(void* instance, uint8_t* rx_data) {
    DJI_MOTOR_DATA_Typedef* DATA = &((DJI_MOTOR_Typedef*)instance)->DATA;

    DATA->Angle_last = DATA->Angle_now;
    DATA->Angle_now  = (int16_t)((rx_data[0] << 8) | rx_data[1]);
    DATA->Speed_last = DATA->Speed_now;
    int16_t spd_raw = (int16_t)((rx_data[2] << 8) | rx_data[3]);
    DATA->Speed_now  = OneFilter1(spd_raw,DATA->Speed_last, 25000);
    DATA->current    = (int16_t)((rx_data[4] << 8) | rx_data[5]);
    DATA->temperature = rx_data[6]; // 6020/3508有温度，2006不看即可

    // 统一处理越界/圈数逻辑
    int16_t diff = DATA->Angle_now - DATA->Angle_last;
    if      (diff < -4000) DATA->Laps++;
    else if (diff >  4000) DATA->Laps--;

    // 圈数异常保护
    if (DATA->Laps > 32500 || DATA->Laps < -32500) {
        DATA->Laps = 0;
        DATA->Aim  = DATA->Angle_now;
    }

    DATA->ONLINE_JUDGE_TIME = MOTOR_OFFLINE_TIME;
    DATA->Angle_Infinite = (int32_t)((DATA->Laps << 13) + DATA->Angle_now);
}

/************************************************************万能分隔符**************************************************************
 *	@performance:	    //电机清空函数
 *	@parameter:		    //
 *	@time:				//23-04-13 19:23
 *	@ReadMe:			//
 *  @LastUpDataTime:    //2023-05-07 17:06    bestrui
 *  @UpData：           //不太好描述
 ************************************************************万能分隔符**************************************************************/
void HEAD_MOTOR_CLEAR(DJI_MOTOR_Typedef* MOTOR , uint8_t mode)
{
    MOTOR->PID_P.Iout  = 0.0f;
    MOTOR->PID_S.Iout  = 0.0f;
    MOTOR->DATA.Aim    = (float)MOTOR->DATA.Angle_Infinite;
    if (mode)       MOTOR->DATA.Laps = 0;
}

/**
 * @brief 通用发送函数
 */
void DJI_Motor_Send(FDCAN_HandleTypeDef* hcan, uint32_t stdid, int16_t n1, int16_t n2, int16_t n3, int16_t n4) {
    uint8_t data[8];
    data[0] = n1 >> 8; data[1] = n1;
    data[2] = n2 >> 8; data[3] = n2;
    data[4] = n3 >> 8; data[5] = n3;
    data[6] = n4 >> 8; data[7] = n4;
    FDCAN_Send_Msg(hcan, stdid, data, 8);
}

/**
 * @brief 电机堵转检测
 */
void DJI_Motor_Stuck_Check(DJI_MOTOR_Typedef* motor, float angle_err, float speed_limit, uint16_t time_limit) {
    // 逻辑：误差大 且 速度小
    if (MATH_ABS_float(motor->PID_P.Err) > angle_err && MATH_ABS_float(motor->DATA.Speed_now) < speed_limit) {
        motor->DATA.Stuck_Time++;
        if (motor->DATA.Stuck_Time > time_limit) {
            HEAD_MOTOR_CLEAR(motor, 0);
            motor->DATA.Stuck_Time = 0;
            motor->DATA.Stuck_Flag[0]++;
        }
    } else {
        motor->DATA.Stuck_Time = 0;
    }
}