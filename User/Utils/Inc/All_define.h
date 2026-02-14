//
// Created by CaoKangqi on 2026/2/13.
//

#ifndef FDCAN_TEST_G4_ALL_DEFINE_H
#define FDCAN_TEST_G4_ALL_DEFINE_H

//CCMRAM配置
#define CCM_DATA  __attribute__((section(".ccmram")))
#define CCM_FUNC  __attribute__((section(".ccmram.text"), noinline, flatten))
//电机离线检测时间
#define MOTOR_OFFLINE_TIME 15;
#define INIT_ANGLE 0;

#endif //FDCAN_TEST_G4_ALL_DEFINE_H