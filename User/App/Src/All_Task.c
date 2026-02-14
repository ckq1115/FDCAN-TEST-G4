//
// Created by CaoKangqi on 2026/1/19.
//
#include "All_Task.h"
#include <stdint.h>

void MY_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    static uint32_t last_trigger_cnt = 0;
    float actual_period;
    if (htim->Instance == TIM4) {
        actual_period = DWT_GetDeltaT(&last_trigger_cnt);
        WS2812_UpdateBreathing(0, 2.0f);
        WS2812_UpdateBreathing(3, 0.2f);
        WS2812_Submit();
        //DJI_Current_Ctrl(&hfdcan3,0x1FE,0,0,1000,0);
    }
}

static uint32_t INS_DWT_Count = 0;   // DWT计数基准
static float dt_s = 0.0f; // 任务单次执行耗时（秒）
static float imu_period_s = 0.0f;
static float imu_operate_us = 0;
void IMU_Task(void *argument)
{
    (void)argument;
    portTickType currentTimeIMU = xTaskGetTickCount();
    ICM42688_Init();
    uint32_t last_wake_time = DWT->CYCCNT; // 初始化记录时间戳

    for(;;)
    {
        imu_period_s = DWT_GetDeltaT(&INS_DWT_Count);
        uint32_t cnt_last = DWT->CYCCNT;
        ICM42688_Read_Fast(IMU_Data.gyro, IMU_Data.accel,&IMU_Data.temp);
        IMU_Update_Task();
        uint32_t operate_end = DWT->CYCCNT;
        uint32_t cycle_diff = operate_end - cnt_last;
        imu_operate_us = cycle_diff / 170;
        static uint32_t exec_start_cnt = 0;
        dt_s = DWT_GetDeltaT(&exec_start_cnt);
        if(xTaskGetTickCount() - currentTimeIMU > 1)
        {
            currentTimeIMU = xTaskGetTickCount();
        }
        vTaskDelayUntil(&currentTimeIMU, 1);
    }
    /* USER CODE END StartDefaultTask */
}

void Motor_Task(void *argument)
{
    (void)argument;
    //motor_mode(&hfdcan1,1,0x100,0xfc);
    for(;;)
    {
        /*VOFA_justfloat(
            dt_s,
            IMU_Data.pitch,
            IMU_Data.roll,
            IMU_Data.yaw,
            IMU_Data.YawTotalAngle,
            0,imu_period_s,imu_operate_us,0,0);*/
        osDelay(1);
    }
}


void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){
    if (huart->Instance == USART3){
        if (Size == 18){
            DBUS_Resolved(DBUS_RX_DATA, &C_DBUS, &C_DBUS_UNION);
            if (C_DBUS.Remote.S1_u8 == 1) {
                imu_ctrl_state = GYRO_CALIB;
            }
        }
    }
}
/*void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    FDCAN_RxHeaderTypeDef RxHeader1;
    uint8_t g_Can1RxData[64];

    FDCAN_RxHeaderTypeDef RxHeader3;
    uint8_t g_Can3RxData[64];

    if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
    {
        if(hfdcan->Instance == FDCAN1)
        {
            /* Retrieve Rx messages from RX FIFO0 #1#
            memset(g_Can1RxData, 0, sizeof(g_Can1RxData));	//接收前先清空数组
            HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader1, g_Can1RxData);
            switch(RxHeader1.Identifier)
            {
                /*case 0x601:
                    CAN_POWER_Rx(&All_Power.P1,g_Can1RxData);
                case 0x602:
                    CAN_POWER_Rx(&All_Power.P2,g_Can1RxData);
                case 0x603:
                    CAN_POWER_Rx(&All_Power.P3,g_Can1RxData);
                case 0x604:
                    CAN_POWER_Rx(&All_Power.P4,g_Can1RxData);#1#
                case 0x207:
                    //MOTOR_CAN_RX_6020RM(&All_Motor.GM6020_1.DATA,g_Can1RxData);
                    break;
                case 0x00:
                    //DM_FBdata(&All_Motor.DM3507_1, g_Can1RxData);
                    break;
            }
        }

        if(hfdcan->Instance == FDCAN3)
        {
            /* Retrieve Rx messages from RX FIFO0 #1#
            memset(g_Can3RxData, 0, sizeof(g_Can3RxData));
            HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader3, g_Can3RxData);
            switch(RxHeader3.Identifier)
            {
                case  0x201:
                    break;
                case 0x207:
                    //MOTOR_CAN_RX_6020RM(&All_Motor.GM6020_1.DATA,g_Can3RxData);
                        break;
                case 0x605:
                    CAN_POWER_Rx(&All_Power.P5,g_Can3RxData);
                    break;
            }
        }
    }
}

void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
    FDCAN_RxHeaderTypeDef RxHeader2;
    uint8_t g_Can2RxData[64];

    if((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) != RESET)
    {
        if(hfdcan->Instance == FDCAN2)
        {
            /* Retrieve Rx messages from RX FIFO0 #1#
            memset(g_Can2RxData, 0, sizeof(g_Can2RxData));
            HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &RxHeader2, g_Can2RxData);
            switch(RxHeader2.Identifier)
            {
                case 0x201:

                    break;
                case 0x202:

                    break;
                case 0x203:

                    break;
                case 0x207:
                    //MOTOR_CAN_RX_6020RM(&All_Motor.GM6020_1.DATA,g_Can2RxData);
                    break;
            }
        }
    }
}*/