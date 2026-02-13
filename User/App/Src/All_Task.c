//
// Created by CaoKangqi on 2026/1/19.
//
#include "All_Task.h"
#include <stdint.h>

// 记录起始计数
float dt_s;
float run_time_s;

void MY_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    static uint32_t last_trigger_cnt = 0;
    float actual_period;
    if (htim->Instance == TIM4) {
        actual_period = DWT_GetDeltaT(&last_trigger_cnt);
        //DJI_Current_Ctrl(&hfdcan3,0x1FE,0,0,1000,0);
    }
}


void IMU_Task(void *argument)
{
    /* USER CODE BEGIN StartDefaultTask */
    /*if (!ESKF_Init(&eskf, 0.001f)) {
        // 初始化失败，挂起任务
        vTaskSuspend(NULL);
    }*/
    ICM42688_Init();
    int32_t last_wake_time = DWT->CYCCNT; // 初始化记录时间戳
    float task_period = 0;
    /* Infinite loop */
    for(;;)
    {
        task_period = DWT_GetDeltaT(&last_wake_time);
        run_time_s = DWT_GetTimeline_s();
        uint32_t cnt_last = DWT->CYCCNT;
        ICM42688_read(IMU_Data.gyro, IMU_Data.accel,&IMU_Data.temp);
        IMU_Update_Task();
        /*VOFA_justfloat(
            IMU_Data.pitch,
            IMU_Data.roll,
            IMU_Data.yaw,
            IMU_Data.YawTotalAngle,
            IMU_Data.accel[0],
            IMU_Data.accel[1],
            IMU_Data.accel[2],
            IMU_Data.gyro[0],
            IMU_Data.gyro[1],
            IMU_Data.gyro[2]
        );*/
        dt_s = DWT_GetDeltaT(&cnt_last);
        osDelay(1);
    }
    /* USER CODE END StartDefaultTask */
}
DM_MOTOR_V_Typdef M3507;
void Motor_Task(void *argument)
{
    motor_mode(&hfdcan1,1,0x200,0xfc);
    /* USER CODE BEGIN Motor_Task */
    /* Infinite loop */
    for(;;)
    {
        //M3507.SPE = C_DBUS.Remote.CH1_int16/8.0f;
        if (C_DBUS.Remote.S1_u8==1) {
            M3507.SPE = IMU_Data.pitch;
        }
        else if (C_DBUS.Remote.S1_u8==3) {
            M3507.SPE = C_DBUS.Remote.CH1_int16/8.0f;
        }
        else {
            M3507.SPE = IMU_Data.yaw;
        }
        speed_ctrl(&hfdcan1,1,&M3507);
        //DM_current_set(&hfdcan1,0x3FE,,0,0,0);
        osDelay(1);
    }
    /* USER CODE END Motor_Task */
}


void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){
    if (huart->Instance == USART3){
        if (Size == 18){
            DBUS_Resolved(DBUS_RX_DATA, &C_DBUS, &C_DBUS_UNION);
            /*if (C_DBUS.Remote.S1_u8 == 1) {
                imu_ctrl_state = GYRO_CALIB;
            }*/
        }
    }
}
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    FDCAN_RxHeaderTypeDef RxHeader1;
    uint8_t g_Can1RxData[64];

    FDCAN_RxHeaderTypeDef RxHeader3;
    uint8_t g_Can3RxData[64];

    if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
    {
        if(hfdcan->Instance == FDCAN1)
        {
            /* Retrieve Rx messages from RX FIFO0 */
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
                    CAN_POWER_Rx(&All_Power.P4,g_Can1RxData);*/
                case 0x605:
                    CAN_POWER_Rx(&All_Power.P5,g_Can1RxData);
                    break;
                case 0x207:
                    MOTOR_CAN_RX_6020RM(&All_Motor.GM6020_1.DATA,g_Can1RxData);
                    break;
                case 0x00:
                    DM_FBdata(&All_Motor.DM3507_1, g_Can1RxData);
                    break;
            }
        }

        if(hfdcan->Instance == FDCAN3)
        {
            /* Retrieve Rx messages from RX FIFO0 */
            memset(g_Can3RxData, 0, sizeof(g_Can3RxData));
            HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader3, g_Can3RxData);
            switch(RxHeader3.Identifier)
            {
                case  0x201:
                    break;
                case 0x207:
                    MOTOR_CAN_RX_6020RM(&All_Motor.GM6020_1.DATA,g_Can3RxData);
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
            /* Retrieve Rx messages from RX FIFO0 */
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
                    MOTOR_CAN_RX_6020RM(&All_Motor.GM6020_1.DATA,g_Can2RxData);
            }
        }
    }
}