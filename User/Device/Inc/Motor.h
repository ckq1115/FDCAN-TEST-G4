//
// Created by CaoKangqi on 2026/1/19.
//

#ifndef FDCAN_TEST_G4_MOTOR_H
#define FDCAN_TEST_G4_MOTOR_H

#include "BSP-FDCAN.h"
#include "CKQ_MATH.h"
#include "controller.h"

/*大疆电机*/
typedef struct
{
    int8_t ONLINE_JUDGE_TIME;
    int16_t Angle_last; // 上一个角度值
    int16_t Angle_now;  // 现在的角度值
    int32_t round;
    int32_t conEncode;     // 处理后的连续的编码器值
    int16_t Speed_last; // 上一个速度值
    int16_t Speed_now;  // 现在的速度值
    int16_t acceleration;//加速度
    int16_t current;
    int8_t temperature;
    int32_t Angle_Infinite;
    int64_t Stuck_Time;
    uint16_t Stuck_Flag[2];
    int16_t Laps;
    float Error;
    float Aim;
    float Aim_last;
    float dt;
}DJI_MOTOR_DATA_Typedef;

typedef struct
{
    uint8_t PID_INIT;
    DJI_MOTOR_DATA_Typedef DATA;
    Feedforward_t PID_F;
    PID_t PID_P;
    PID_t PID_S;
}DJI_MOTOR_Typedef;



typedef struct{
    float shunt_volt;
    float bus_volt;
    float current;
    float power;
}Power_Typedef;

typedef struct{
    Power_Typedef P1;
    Power_Typedef P2;
    Power_Typedef P3;
    Power_Typedef P4;
    Power_Typedef P5;
}ALL_POWER_RX;

extern ALL_POWER_RX All_Power;


void MOTOR_CAN_RX_3508RM(DJI_MOTOR_DATA_Typedef* DATA , uint8_t* can_data);
void MOTOR_CAN_RX_2006RM(DJI_MOTOR_DATA_Typedef* MOTOR , uint8_t* can_data);
void MOTOR_CAN_RX_6020RM(DJI_MOTOR_DATA_Typedef* DATA , uint8_t* can_data);
void HEAD_MOTOR_CLEAR(DJI_MOTOR_Typedef* MOTOR , uint8_t mode);
void HEAD_MOTOR2006_STUCK(DJI_MOTOR_Typedef* MOTOR , float ERROR_ANGLE , float ERROR_SPEED , uint16_t ERROR_TIME);
//void RUI_F_HEAD_MOTOR3508_STUCK(DJI_MOTOR_Typedef* MOTOR , float ERROR_CURRENT , float ERROR_SPEED);
void DJI_Current_Ctrl(hcan_t* hcan, uint16_t stdid, int16_t num1, int16_t num2, int16_t num3, int16_t num4);
void MotorRoundResolve(DJI_MOTOR_Typedef *motor);
void DJI_Motor_Init(void);
int16_t Motor_PID_Control(DJI_MOTOR_Typedef *motor, float target);

void CAN_POWER_Rx(Power_Typedef* pPower, uint8_t *rx_data);

/*达妙电机*/
#define MIT_MODE  0x000
#define POS_MODE  0x100
#define SPEED_MODE  0x200

#define P_MIN   -12.5f
#define P_MAX   12.5f
#define V_MIN   -30.0f
#define V_MAX   30.0f
#define KP_MIN  0.0f
#define KP_MAX  500.0f
#define KD_MIN  0.0f
#define KD_MAX  5.0f
#define T_MIN   -10.0f
#define T_MAX   10.0f

typedef struct
{
	uint16_t id;
	uint16_t state;
	int p_int;
	int v_int;
	int t_int;
	int kp_int;
	int kd_int;
	float tor;
	float Kp;
	float Kd;
	float Tmos;
	float Tcoil;
	float aim;
	float cur;
	int16_t pos[2];
  int16_t vel[2];
	int16_t round;
	int32_t reality;
	uint16_t initialAngle;
	float ralativeAngle;
	uint8_t ONLINE_JUDGE_TIME;

}motor_fbpara_t;

typedef struct
{
    uint16_t mode;
    uint16_t id;
    uint16_t state;
    int p_int;
    int v_int;
    int t_int;
    int kp_int;
    int kd_int;
    float pos;
    float vel;
    float tor;
    float Kp;
    float Kd;
    float Tmos;
    float Tcoil;
    int8_t ONLINE_JUDGE_TIME;
}DM_MOTOR_DATA_Typdef;
typedef struct {
	float A;
	float B;
    float rin;
    float lastRin;
    float perrRin;
}FFC_typedef;
typedef struct
{
	uint16_t mode[2];
	motor_fbpara_t para;
	DM_MOTOR_DATA_Typdef DM_DATA;
  FFC_typedef FFC;
  PID_t PID_P;
  PID_t PID_S;

}DM_MOTOR_Typdef ;

typedef struct
{
    float position_des;
    float velocity_des;
    float torque_des;
    float Kp;
    float Kd;
}DM_MOTOR_MIT_Typdef;

typedef struct
{
    float POS;
    float SPE;
}DM_MOTOR_PV_Typdef;

typedef struct
{
    float SPE;
    float none_null;
}DM_MOTOR_V_Typdef;

extern DM_MOTOR_V_Typdef M3507;



typedef enum
{
  DM_CMD_MOTOR_MODE = 0xfc,   // 使能,会响应指令
  DM_CMD_RESET_MODE = 0xfd,   // 停止
  DM_CMD_ZERO_POSITION = 0xfe, // 将当前的位置设置为编码器零位
  DM_CMD_CLEAR_ERROR = 0xfb // 清除电机过热错误
}DMMotor_Mode_e;
int16_t OneFilter1(int16_t last, int16_t now, float thresholdValue);
void motor_mode(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id, DMMotor_Mode_e what);
void pos_speed_ctrl(hcan_t* hcan,uint16_t motor_id, DM_MOTOR_PV_Typdef *PVS);
void speed_ctrl(hcan_t* hcan,uint16_t motor_id, DM_MOTOR_V_Typdef *vel);
float uint_to_float(int x_int, float x_min, float x_max, int bits);
void DM_current_set(hcan_t* hcan, uint16_t id, float m1_cur_set, float m2_cur_set, float m3_cur_set, float m4_cur_set);
void DM_RXdata(DM_MOTOR_Typdef  *motor, uint8_t *rx_data);
void DM_FBdata(DM_MOTOR_Typdef *motor, uint8_t *rx_data);
void mit_ctrl(hcan_t* hcan, uint16_t motor_id, float pos, float vel,float kp, float kd, float torq);


typedef struct
{
	DJI_MOTOR_Typedef  GM6020_1;
	DJI_MOTOR_Typedef  M3508_1;
	DJI_MOTOR_Typedef  M3508_2;
	DJI_MOTOR_Typedef  M3508_3;
	DJI_MOTOR_Typedef  M3508_4;
	DM_MOTOR_Typdef DM3507_1;
} All_Motor_TypeDef;
extern All_Motor_TypeDef All_Motor;
#endif //FDCAN_TEST_G4_MOTOR_H