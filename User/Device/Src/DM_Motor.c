//
// Created by CaoKangqi on 2026/2/14.
//
#include "DM_Motor.h"
#include "All_define.h"

/**
 * @brief 达妙电机数据解析函数
 * @param motor   电机结构体指针
 * @param rx_data 接收到的8字节数据数组
 * @note 该函数根据达妙电机的通信协议，将接收到的字节数据解析为电机的物理量，并更新电机状态
 */
void DM_Standard_Resolve(DM_MOTOR_Typdef *motor, uint8_t *rx_data)
{
    motor->DATA.id = (rx_data[0]) & 0x0F;
    motor->DATA.state = (rx_data[0]) >> 4;
    motor->DATA.p_int = (rx_data[1] << 8) | rx_data[2];
    motor->DATA.v_int = (rx_data[3] << 4) | (rx_data[4] >> 4);
    motor->DATA.t_int = ((rx_data[4] & 0xF) << 8) | rx_data[5];

    // 映射到物理量
    motor->DATA.pos = uint_to_float(motor->DATA.p_int, P_MIN, P_MAX, 16);
    motor->DATA.vel = uint_to_float(motor->DATA.v_int, V_MIN, V_MAX, 12);
    motor->DATA.tor = uint_to_float(motor->DATA.t_int, T_MIN, T_MAX, 12);

    motor->DATA.Tmos = (float)(rx_data[6]);
    motor->DATA.Tcoil = (float)(rx_data[7]);
    motor->DATA.ONLINE_JUDGE_TIME = MOTOR_OFFLINE_TIME;
}

/**
 * @brief 达妙电机一拖四模式数据解析函数
 * @param motor   电机结构体指针
 * @param rx_data 接收到的8字节数据数组
 * @note 该函数在标准解析的基础上，增加了多圈角度处理、速度滤波和离线计时等功能，以适应更复杂的应用场景
 */
void DM_1to4_Resolve(DM_MOTOR_Typdef *motor, uint8_t *rx_data)
{
    motor->DATA.Angle_last = motor->DATA.Angle_now;
    motor->DATA.Angle_now = (rx_data[0] << 8) | rx_data[1];

    int16_t spd_raw = (rx_data[2] << 8) | rx_data[3];
    int16_t cur_raw = (rx_data[4] << 8) | rx_data[5];

    // 多圈逻辑与零位偏移
    int16_t angleError = motor->DATA.Angle_now - INIT_ANGLE;
    if (angleError > 4096) angleError -= 8192;
    else if (angleError < -4096) angleError += 8192;

    motor->DATA.ralativeAngle = angleError * 0.043945f; // 360/8192

    // 圈数统计
    if ((motor->DATA.Angle_now - motor->DATA.Angle_last) < -4096) motor->DATA.round++;
    else if ((motor->DATA.Angle_now - motor->DATA.Angle_last) > 4096) motor->DATA.round--;

    // 速度滤波
    motor->DATA.Speed_last = motor->DATA.Speed_now;
    motor->DATA.Speed_now = OneFilter1(spd_raw / 100, motor->DATA.Speed_last, 500);

    motor->DATA.current = ((float)cur_raw);
    motor->DATA.Tcoil = (float)(rx_data[6]);
    motor->DATA.Tmos = (float)(rx_data[7]);
    motor->DATA.reality = (int32_t)((motor->DATA.round * 8192) + motor->DATA.Angle_now);
    motor->DATA.ONLINE_JUDGE_TIME = MOTOR_OFFLINE_TIME;
}

/**
 * @brief 达妙电机模式切换命令
 * @param hcan     FDCAN 句柄
 * @param motor_id 电机 ID (Base ID)
 * @param mode_id  模式 ID (MIT_MODE, POS_MODE, SPEED_MODE)
 * @param what     具体操作 (DMMotor_Mode_e 枚举值)
 */
void Motor_Mode(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id, DMMotor_Mode_e what)
{
    uint8_t data[8];
    uint16_t id = motor_id + mode_id;

    data[0] = 0xFF;
    data[1] = 0xFF;
    data[2] = 0xFF;
    data[3] = 0xFF;
    data[4] = 0xFF;
    data[5] = 0xFF;
    data[6] = 0xFF;
    data[7] = 0xFC;

    FDCAN_Send_Msg(hcan, id, data, 8);
}

/**
 * @brief 达妙电机 MIT 模式控制
 * @param hcan     FDCAN 句柄
 * @param motor_id 电机 ID (Base ID)
 * @param pos      目标位置 (float)
 * @param vel      目标速度 (float)
 * @param kp       位置环增益 (float)
 * @param kd       速度环增益 (float)
 * @param torq     目标扭矩 (float)
 */
void MIT_Ctrl(FDCAN_HandleTypeDef* hcan, uint16_t motor_id, float pos, float vel, float kp, float kd, float torq)
{
    uint8_t data[8];
    uint16_t p = float_to_uint(pos, P_MIN, P_MAX, 16);
    uint16_t v = float_to_uint(vel, V_MIN, V_MAX, 12);
    uint16_t k_p = float_to_uint(kp, KP_MIN, KP_MAX, 12);
    uint16_t k_d = float_to_uint(kd, KD_MIN, KD_MAX, 12);
    uint16_t t = float_to_uint(torq, T_MIN, T_MAX, 12);

    data[0] = p >> 8;
    data[1] = p & 0xFF;
    data[2] = v >> 4;
    data[3] = ((v & 0x0F) << 4) | (k_p >> 8);
    data[4] = k_p & 0xFF;
    data[5] = k_d >> 4;
    data[6] = ((k_d & 0x0F) << 4) | (t >> 8);
    data[7] = t & 0xFF;

    FDCAN_Send_Msg(hcan, motor_id + MIT_MODE, data, 8);
}

/**
 * @brief 达妙电机位置速度控制模式
 * @param hcan     FDCAN 句柄
 * @param motor_id 电机 ID (Base ID)
 * @param pos      目标位置 (float)
 * @param vel      目标速度 (float)
 */
void Pos_Speed_Ctrl(FDCAN_HandleTypeDef* hcan, uint16_t motor_id, float pos, float vel)
{
    uint8_t data[8];
    memcpy(&data[0], &pos, 4);
    memcpy(&data[4], &vel, 4);
    FDCAN_Send_Msg(hcan, motor_id + POS_MODE, data, 8);
}

/**
 * @brief 达妙电机速度控制模式
 * @param hcan     FDCAN 句柄
 * @param motor_id 电机 ID (Base ID)
 * @param vel      目标速度 (float)
 */
void Speed_Ctrl(FDCAN_HandleTypeDef* hcan, uint16_t motor_id, float vel)
{
    uint8_t data[8];
    memcpy(&data[0], &vel, sizeof(float));
    data[4] = 0x00;
    data[5] = 0x00;
    data[6] = 0x00;
    data[7] = 0x00;
    uint16_t send_id = motor_id + SPEED_MODE;
    FDCAN_Send_Msg(hcan, send_id, data, 8);
}

/**
 * @brief 达妙电机一拖四模式电流控制
 * @param hcan      FDCAN 句柄
 * @param master_id 主控电机 ID (Base ID)
 * @param m1_cur    电机1目标电流 (float)
 * @param m2_cur    电机2目标电流 (float)
 * @param m3_cur    电机3目标电流 (float)
 * @param m4_cur    电机4目标电流 (float)
 */
void DM_Motor_Send(FDCAN_HandleTypeDef* hcan, uint16_t master_id, float m1_cur, float m2_cur, float m3_cur, float m4_cur)
{
    uint8_t data[8];
    float ratio = 819.2f; // 16384/20
    int16_t cur_val[4];

    cur_val[0] = (int16_t)(m1_cur * ratio);
    cur_val[1] = (int16_t)(m2_cur * ratio);
    cur_val[2] = (int16_t)(m3_cur * ratio);
    cur_val[3] = (int16_t)(m4_cur * ratio);

    for(int i=0; i<4; i++) {
        data[i*2]   = (uint8_t)(cur_val[i] >> 8);
        data[i*2+1] = (uint8_t)(cur_val[i] & 0xFF);
    }
    FDCAN_Send_Msg(hcan, master_id, data, 8);
}