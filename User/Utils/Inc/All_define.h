//
// Created by CaoKangqi on 2026/2/13.
//

#ifndef G4_FRAMEWORK_ALL_DEFINE_H
#define G4_FRAMEWORK_ALL_DEFINE_H

//CCMRAM配置
#define CCM_DATA  __attribute__((section(".ccmram")))
#define CCM_FUNC  __attribute__((section(".ccmram.text"), noinline, flatten))

/// 圆周率
#define PI 3.14159265358979f
/// 角度转弧度
#define DEG2RAD (PI / 180.0f)
/// 弧度转角度
#define RAD2DEG (180.0f / PI)

//设备离线
#define DEVICE_OFFLINE 0
//设备在线
#define DEVICE_ONLINE  1
//电机离线检测时间
#define MOTOR_OFFLINE_TIME 15;
//电容离线检测时间
#define CAP_OFFLINE_TIME 15;
//遥控离线检测时间
#define DBUS_OFFLINE_TIME 30;

#define INIT_ANGLE 0;

#endif //G4_FRAMEWORK_ALL_DEFINE_H