//
// Created by CaoKangqi on 2026/2/23.
//

#ifndef G4_FRAMEWORK_CHASSIS_CALC_H
#define G4_FRAMEWORK_CHASSIS_CALC_H
#include <stdint.h>

typedef struct
{
    float wheel_perimeter;    /* 轮的周长（mm）*/
    float wheeltrack;         /* 轮距（mm）*/
    float wheelbase;          /* 轴距（mm）*/
    float rotate_x_offset;    /* 相对于底盘中心的x轴旋转偏移量(mm) */
    float rotate_y_offset;    /* 相对于底盘中心的y轴旋转偏移量(mm) */
    float deceleration_ratio; /* 电机减速比 */
    int max_vx_speed;         /* 底盘的x轴的最大速度(mm/s) */
    int max_vy_speed;         /* 底盘的y轴的最大速度(mm/s) */
    int max_vw_speed;         /* 底盘的自转的最大速度(degree/s) */
    int max_wheel_ramp;       /* 3508最大转速 */
    // 每一个轮子的旋转比率//与旋转中心点相关
    float raid_fr;            // 右前
    float raid_fl;            // 左前
    float raid_bl;            // 左后
    float raid_br;            // 右后
    float wheel_rpm_ratio;    // 用来将速度转化成转每分钟
} mecanumInit_typdef;

typedef struct
{
    float wheel_perimeter;    /* 轮的周长（mm）*/
    float CHASSIS_DECELE_RATIO; /* 电机减速比 */
    float LENGTH_A;         /* 底盘长的一半（mm）*/
    float LENGTH_B;          /* 底盘宽的一半（mm）*/
    float rotate_radius;     /* (A+B)/57.3, 将角速度(deg/s)转换为线速度(mm/s) */
    float wheel_rpm_ratio;   /* 线速度(mm/s) -> 轮速(rpm) */
    int max_vx_speed;        /* 底盘x轴最大速度(mm/s) */
    int max_vy_speed;        /* 底盘y轴最大速度(mm/s) */
    int max_vw_speed;        /* 底盘自转最大速度(deg/s) */
    int max_wheel_ramp;      /* 3508最大转速 */
} OmniInit_typdef;

extern OmniInit_typdef OmniInit_t;

uint8_t MecanumInit(mecanumInit_typdef *mecanumInitT);
void MecanumResolve(float *wheel_rpm, float vx_temp, float vy_temp, float vr, mecanumInit_typdef *mecanumInit_t);

uint8_t OmniInit(OmniInit_typdef *OmniInitT);
void Omni_calc(float *wheel_rpm, float vx_temp, float vy_temp, float vr, OmniInit_typdef *OmniInit_t);

#endif //G4_FRAMEWORK_CHASSIS_CALC_H