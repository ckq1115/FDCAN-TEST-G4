//
// Created by CaoKangqi on 2026/2/23.
//
#include "Chassis_Calc.h"

#include "All_define.h"

static float ClampFloat(float val, float min_val, float max_val)
{
    if (val < min_val) return min_val;
    if (val > max_val) return max_val;
    return val;
}

static float AbsFloat(float val)
{
    return (val < 0.0f) ? -val : val;
}

uint8_t MecanumInit(mecanumInit_typdef *mecanumInitT)
{
    /*初始化参数*/
    mecanumInitT->deceleration_ratio = 3591/187; // 减速比1/19
    mecanumInitT->max_vw_speed       = 50000;     // r方向上的最大速度单位：毫米/秒
    mecanumInitT->max_vx_speed       = 50000;     // x方向上的最大速度单位：毫米/秒
    mecanumInitT->max_vy_speed       = 50000;     // y方向上的最大速度单位：毫米/秒
    mecanumInitT->max_wheel_ramp     = 8000;      // 3508最大转速不包含限速箱
    mecanumInitT->rotate_x_offset    = 00.0f;     // 云台在x轴的偏移量  mm
    mecanumInitT->rotate_y_offset    = 00.0f;     // 云台在y轴的偏移量  mm
    mecanumInitT->wheelbase          = 300;       // 轮距	mm
    mecanumInitT->wheeltrack         = 300;       // 轴距	mm
    mecanumInitT->wheel_perimeter    = 478;       // 轮子的周长(mm)

    /*计算旋转比率*/
    mecanumInitT->raid_fr = ((mecanumInitT->wheelbase + mecanumInitT->wheeltrack) / 2.0f -
                             mecanumInitT->rotate_x_offset + mecanumInitT->rotate_y_offset) /
                            57.3f;
    mecanumInitT->raid_fl = ((mecanumInitT->wheelbase + mecanumInitT->wheeltrack) / 2.0f -
                             mecanumInitT->rotate_x_offset - mecanumInitT->rotate_y_offset) /
                            57.3f;
    mecanumInitT->raid_bl = ((mecanumInitT->wheelbase + mecanumInitT->wheeltrack) / 2.0f +
                             mecanumInitT->rotate_x_offset - mecanumInitT->rotate_y_offset) /
                            57.3f;
    mecanumInitT->raid_br = ((mecanumInitT->wheelbase + mecanumInitT->wheeltrack) / 2.0f +
                             mecanumInitT->rotate_x_offset + mecanumInitT->rotate_y_offset) /
                            57.3f;
// 将算出来的数据转化到转每分钟上去 raid = 60/(电机减速比*轮的周长)
    mecanumInitT->wheel_rpm_ratio = 60.0f / (mecanumInitT->wheel_perimeter * mecanumInitT->deceleration_ratio);

    return 0;
}

void MecanumResolve(float *wheel_rpm, float vx_temp, float vy_temp, float vr, mecanumInit_typdef *mecanumInit_t)
{
    float vx = ClampFloat(vx_temp, -mecanumInit_t->max_vx_speed, mecanumInit_t->max_vx_speed);
    float vy = ClampFloat(vy_temp, -mecanumInit_t->max_vy_speed, mecanumInit_t->max_vy_speed);
    float vw = ClampFloat(vr, -mecanumInit_t->max_vw_speed, mecanumInit_t->max_vw_speed);

    wheel_rpm[0] = (-vx + vy + vw * mecanumInit_t->raid_fr) * mecanumInit_t->wheel_rpm_ratio;
    wheel_rpm[1] = ( vx + vy + vw * mecanumInit_t->raid_fl) * mecanumInit_t->wheel_rpm_ratio;
    wheel_rpm[2] = ( vx - vy + vw * mecanumInit_t->raid_bl) * mecanumInit_t->wheel_rpm_ratio;
    wheel_rpm[3] = (-vx - vy + vw * mecanumInit_t->raid_br) * mecanumInit_t->wheel_rpm_ratio;

    float max_abs = 0.0f;
    for (uint8_t i = 0; i < 4; i++) {
        float a = AbsFloat(wheel_rpm[i]);
        if (a > max_abs) max_abs = a;
    }
    if (max_abs > (float)mecanumInit_t->max_wheel_ramp && max_abs > 0.0f) {
        float rate = (float)mecanumInit_t->max_wheel_ramp / max_abs;
        for (uint8_t i = 0; i < 4; i++) {
            wheel_rpm[i] *= rate;
        }
    }
}

uint8_t OmniInit(OmniInit_typdef *OmniInitT)
{
    OmniInitT->wheel_perimeter = 155*PI;// 轮子的周长(mm)
    OmniInitT->CHASSIS_DECELE_RATIO = 3591/187;
    OmniInitT->LENGTH_A = 180;
    OmniInitT->LENGTH_B = 180;
    OmniInitT->max_vx_speed = 50000;
    OmniInitT->max_vy_speed = 50000;
    OmniInitT->max_vw_speed = 50000;
    OmniInitT->max_wheel_ramp = 8000;

    OmniInitT->rotate_radius = (OmniInitT->LENGTH_A + OmniInitT->LENGTH_B) / 57.3f;
    OmniInitT->wheel_rpm_ratio = 60.0f / (OmniInitT->wheel_perimeter) * OmniInitT->CHASSIS_DECELE_RATIO;
    return 0;
}

/* 计算每个轮子的转速
 * @param wheel_rpm 输出参数，长度为4的数组，分别对应4个轮子的转速(rpm)
 * @param vx_temp 期望的x轴速度(mm/s)
 * @param vy_temp 期望的y轴速度(mm/s)
 * @param vr 期望的自转速度(deg/s)
 * @param OmniInit_t 指向已初始化的OmniInit_typdef结构体的指针，包含底盘参数和限速设置
 * @return void
 */
void Omni_calc(float *wheel_rpm, float vx_temp, float vy_temp, float vr, OmniInit_typdef *OmniInit_t)
{
    float vx = ClampFloat(vx_temp, -OmniInit_t->max_vx_speed, OmniInit_t->max_vx_speed);
    float vy = ClampFloat(vy_temp, -OmniInit_t->max_vy_speed, OmniInit_t->max_vy_speed);
    float vw = ClampFloat(vr, -OmniInit_t->max_vw_speed, OmniInit_t->max_vw_speed);
    float rot = vw * OmniInit_t->rotate_radius;// 将角速度转换为线速度，单位mm/s

    wheel_rpm[0] = (  vx + vy + rot) * OmniInit_t->wheel_rpm_ratio;//left//x，y方向速度,w底盘转动速度
    wheel_rpm[1] = (  vx - vy + rot) * OmniInit_t->wheel_rpm_ratio;//forward
    wheel_rpm[2] = ( -vx - vy + rot) * OmniInit_t->wheel_rpm_ratio;//right
    wheel_rpm[3] = ( -vx + vy + rot) * OmniInit_t->wheel_rpm_ratio;//back

    float max_abs = 0.0f;
    for (uint8_t i = 0; i < 4; i++) {
        float a = AbsFloat(wheel_rpm[i]);
        if (a > max_abs) max_abs = a;
    }
    if (max_abs > (float)OmniInit_t->max_wheel_ramp && max_abs > 0.0f) {
        float rate = (float)OmniInit_t->max_wheel_ramp / max_abs;
        for (uint8_t i = 0; i < 4; i++) {
            wheel_rpm[i] *= rate;
        }
    }
}