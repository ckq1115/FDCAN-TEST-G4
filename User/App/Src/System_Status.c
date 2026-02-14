//
// Created by CaoKangqi on 2026/2/14.
//
#include "System_Status.h"

#include "All_define.h"
#include "tim.h"
#include "WS2812.h"

void System_Root(ROOT_STATUS_Typedef *Root, DBUS_Typedef *DBUS, MOTOR_Typdef *MOTOR, CAP_RXDATA *CAP_GET)
{
    All_Status(Root, DBUS, MOTOR, CAP_GET);
    LED_Show_Status(Root);
}

uint8_t DJI_F_MOTOR_STATUS(DJI_MOTOR_DATA_Typedef* DATA)
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

uint8_t DM_F_MOTOR_STATUS(DM_MOTOR_DATA_Typdef* DATA)
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
}

void LED_Show_Status(ROOT_STATUS_Typedef *Root)
{
    if (Root->RM_DBUS == DEVICE_OFFLINE)
    {
        WS2812_SetPixel(3, 180, 0, 0); // 红色表示遥控离线
        Buzzer_UpdateCycle(0.5f, 1.0f, 20);
        WS2812_UpdateBreathing(3, 0.2f);
    }
    /*else if (Root->MOTOR_HEAD_Pitch == DEVICE_OFFLINE || Root->MOTOR_HEAD_Yaw == DEVICE_OFFLINE)
    {
        WS2812_SetPixel(3, 255, 255, 0); // 黄色表示头部电机离线
    }
    else if (Root->MOTOR_Shoot_L == DEVICE_OFFLINE || Root->MOTOR_Shoot_R == DEVICE_OFFLINE || Root->MOTOR_Shoot_M == DEVICE_OFFLINE)
    {
        WS2812_SetPixel(3, 255, 0, 255); // 紫色表示发射电机离线
    }
    else if (Root->MOTOR_Chassis_1 == DEVICE_OFFLINE || Root->MOTOR_Chassis_2 == DEVICE_OFFLINE || Root->MOTOR_Chassis_3 == DEVICE_OFFLINE || Root->MOTOR_Chassis_4 == DEVICE_OFFLINE)
    {
        WS2812_SetPixel(3, 255, 255, 255); // 青色表示底盘电机离线
    }
    else if (Root->Cap == DEVICE_OFFLINE)
    {
        WS2812_SetPixel(3, 255, 255, 0); // 黄色表示电容离线
    }*/
    else if (Root->RM_DBUS == DEVICE_ONLINE)
    {
        WS2812_SetPixel(3, 0, 60, 0); // 绿色表示系统正常
        WS2812_UpdateBreathing(3, 2.0f);
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