//
// Created by CaoKangqi on 2026/2/13.
//

#ifndef FDCAN_TEST_G4_ALL_DEFINE_H
#define FDCAN_TEST_G4_ALL_DEFINE_H

//CCMRAM配置
#define CCM_DATA  __attribute__((section(".ccmram")))
#define CCM_FUNC  __attribute__((section(".ccmram.text"), noinline, flatten))

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

#endif //FDCAN_TEST_G4_ALL_DEFINE_H