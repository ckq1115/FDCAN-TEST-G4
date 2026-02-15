//
// Created by CaoKangqi on 2026/2/14.
//
#include "System_Status.h"

#include <stdbool.h>

#include "All_define.h"
#include "All_Init.h"
#include "tim.h"
#include "WS2812.h"

void System_Root(ROOT_STATUS_Typedef *Root, DBUS_Typedef *DBUS, MOTOR_Typdef *MOTOR, CAP_RXDATA *CAP_GET)
{
    All_Status(Root, DBUS, MOTOR, CAP_GET);
    LED_Show_Status(Root);
}

/**
 * @brief 大疆电机在线状态监测函数
 * @param DATA 电机数据结构体指针，包含在线计时器
 * @return DEVICE_ONLINE 或 DEVICE_OFFLINE
 * @note 该函数通过递减在线计时器来判断电机是否在线，若计时器小于5则认为离线，并重置计时器为0
 */
uint8_t DJI_MOTOR_STATUS(DJI_MOTOR_DATA_Typedef* DATA)
{
    DATA->ONLINE_JUDGE_TIME--;

    if (DATA->ONLINE_JUDGE_TIME < 5)
    {
        DATA->ONLINE_JUDGE_TIME = 0;
        return DEVICE_OFFLINE;
    }
    else
        return DEVICE_ONLINE;
}

/**
 * @brief 达妙电机在线状态监测函数
 * @param DATA 电机数据结构体指针，包含在线计时器
 * @return DEVICE_ONLINE 或 DEVICE_OFFLINE
 * @note 该函数通过递减在线计时器来判断电机是否在线，若计时器小于5则认为离线，并重置计时器为0
 */
uint8_t DM_MOTOR_STATUS(DM_MOTOR_DATA_Typdef* DATA)
{
    DATA->ONLINE_JUDGE_TIME--;

    if (DATA->ONLINE_JUDGE_TIME < 5)
    {
        DATA->ONLINE_JUDGE_TIME = 0;
        return DEVICE_OFFLINE;
    }
    return DEVICE_ONLINE;
}

void All_Status(ROOT_STATUS_Typedef *Root, DBUS_Typedef *DBUS, MOTOR_Typdef *MOTOR, CAP_RXDATA *CAP_GET)
{
    if (DBUS->DBUS_ONLINE_JUDGE_TIME < 5)
    {
        DBUS->DBUS_ONLINE_JUDGE_TIME = 3;
        Root->RM_DBUS = DEVICE_OFFLINE;
    }
    else
    {
        Root->RM_DBUS = DEVICE_ONLINE;
    }
    DBUS->DBUS_ONLINE_JUDGE_TIME--;

    //电容在线监测
    if(CAP_GET->ONLINE_JUDGE_TIME < 5)
    {
        CAP_GET->ONLINE_JUDGE_TIME = 3;
        Root->Cap = DEVICE_OFFLINE;
    }
    else
    {
        Root->Cap = DEVICE_ONLINE;
    }
    /*Root->MOTOR_Chassis_1 = DJI_MOTOR_STATUS(&All_Motor.DJI_3508_Chassis[0].DATA);
    Root->MOTOR_Chassis_2 = DJI_MOTOR_STATUS(&All_Motor.DJI_3508_Chassis[1].DATA);
    Root->MOTOR_Chassis_3 = DJI_MOTOR_STATUS(&All_Motor.DJI_3508_Chassis[2].DATA);
    Root->MOTOR_Chassis_4 = DJI_MOTOR_STATUS(&All_Motor.DJI_3508_Chassis[3].DATA);
    Root->MOTOR_HEAD_Pitch = DM_MOTOR_STATUS(&All_Motor.DM4310_Pitch.DATA);*/
    Root->MOTOR_HEAD_Yaw = DM_MOTOR_STATUS(&All_Motor.DM4310_Yaw.DATA);
}

static const Status_t Display_Map[] = {
    [DEVICE_ONLINE]  = { .r = 0,   .g = 60, .b = 0, .breathe = 2.0f, .buzzer = false },
    [DEVICE_OFFLINE] = { .r = 150, .g = 0, .b = 0, .breathe = 0.5f, .buzzer = true  }
};

void LED_Show_Status(ROOT_STATUS_Typedef *Root)
{
    const Status_t *dbus  = &Display_Map[Root->RM_DBUS];
    const Status_t *motor = &Display_Map[Root->MOTOR_HEAD_Yaw];

    WS2812_SetPixel(2, motor->r, motor->g, motor->b);
    WS2812_UpdateBreathing(2, motor->breathe);
    WS2812_SetPixel(3, dbus->r, dbus->g, dbus->b);
    WS2812_UpdateBreathing(3, dbus->breathe);


    if (motor->buzzer && dbus->buzzer) {
        Buzzer_UpdateCycle(0.25f, 0.5f, 20);
    }
    else if (dbus->buzzer || motor->buzzer) {
        Buzzer_UpdateCycle(0.5f, 1.0f, 20);
    }
    else {
        Buzzer_Stop();
    }
    WS2812_Submit();
}

/**
 * @brief  非阻塞式蜂鸣器周期控制函数
 * @param  activeTime : 一个周期内响的时间 (秒)
 * @param  period     : 整个周期的总时间 (秒)
 * @param  maxVolume  : 目标响度 (PWM 占空比数值，通常对应 CCR 寄存器值)
 */
void Buzzer_UpdateCycle(float activeTime, float period, uint16_t maxVolume) {
    if (period <= 0.0f || activeTime <= 0.0f) {
        __HAL_TIM_SET_COMPARE(&htim20, TIM_CHANNEL_2, 0);
        return;
    }

    // 1. 获取当前高精度时间
    float currentTime = DWT_GetTimeline_s();

    // 2. 计算当前时间在周期内的位置 (取模运算)
    // fmodf 用于对浮点数取余，得到当前处于周期的第几秒
    float phase = fmodf(currentTime, period);

    // 3. 判断当前处于“响”还是“灭”的状态
    uint16_t currentCCR = 0;
    if (phase < activeTime) {
        // 处于“响”的时间段内
        currentCCR = maxVolume;
    } else {
        // 处于“灭”的时间段内
        currentCCR = 0;
    }

    // 4. 更新硬件 PWM 寄存器
    __HAL_TIM_SET_COMPARE(&htim20, TIM_CHANNEL_2, currentCCR);
}

void Buzzer_Start() {
    HAL_TIM_Base_Start_IT(&htim20);
}
void Buzzer_Stop() {
    __HAL_TIM_SET_COMPARE(&htim20, TIM_CHANNEL_2, 0);
}