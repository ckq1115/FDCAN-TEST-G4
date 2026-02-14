//
// Created by CaoKangqi on 2026/2/13.
//

#ifndef FDCAN_TEST_G4_ALL_INIT_H
#define FDCAN_TEST_G4_ALL_INIT_H

#include <string.h>
#include "main.h"
#include "DBUS.h"
#include "DJI_Motor.h"
#include "DM_Motor.h"
#include "BSP_ICM42688P.h"
#include "tim.h"
#include "usart.h"
#include "WS2812.h"
#include "controller.h"
#include "QuaternionEKF.h"
#include "Vofa.h"
#include "spi.h"
#include "task.h"
#include "mahony_filter.h"
#include "user_lib.h"
#include "IMU_Task.h"
#include "All_define.h"
#include "cmsis_os2.h"

extern uint8_t DBUS_RX_DATA[18];
extern DBUS_Typedef C_DBUS;
extern DBUS_UNION_Typdef C_DBUS_UNION;


void All_Init(void);
#endif //FDCAN_TEST_G4_ALL_INIT_H