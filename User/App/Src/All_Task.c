//
// Created by CaoKangqi on 2026/1/19.
//
#include "All_Task.h"
#include <stdint.h>

#include "main.h"

void MY_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    static uint32_t last_trigger_cnt = 0;
    float actual_period;
    if (htim->Instance == TIM4) {
        actual_period = DWT_GetDeltaT(&last_trigger_cnt);
        (imu_ctrl_state == ERROR_STATE) ? WS2812_UpdateBreathing(0, 0.2f) : WS2812_UpdateBreathing(0, 2.0f);
        System_Root(&ROOT_Status, &C_DBUS, &All_Motor, NULL);
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
        VOFA_justfloat(
            dt_s,
            IMU_Data.pitch,
            IMU_Data.roll,
            IMU_Data.yaw,
            IMU_Data.YawTotalAngle,
            imu_period_s,imu_operate_us,0,0,0);
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
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0)
        return;
    FDCAN_RxHeaderTypeDef rx;
    uint8_t data[8];
    while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0)
    {
        HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx, data);
        if (hfdcan->Instance == FDCAN1)
        {
            switch (rx.Identifier)
            {
                case 0x207: // MOTOR_CAN_RX_6020RM(&All_Motor.GM6020_1.DATA, data);
                    break;
                case 0x000: // DM_FBdata(&All_Motor.DM3507_1, data);
                    break;
                case 0x605: CAN_POWER_Rx(&All_Power.P5, data);
                    break;
                default:
                    break;
            }
        }
        else if (hfdcan->Instance == FDCAN3)
        {
            switch (rx.Identifier)
            {
                case 0x201:
                    break;
                case 0x207: // MOTOR_CAN_RX_6020RM(&All_Motor.GM6020_1.DATA, data);
                    break;
                case 0x605: CAN_POWER_Rx(&All_Power.P5, data);
                    break;
                default:
                    break;
            }
        }
    }
}


void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
    FDCAN_RxHeaderTypeDef rx;
    uint8_t data[8];
    while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO1) > 0)
    {
        HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &rx, data);
        switch (rx.Identifier)
        {
            case 0x207: // GM6020_Decode(&All_Motor.GM6020_1, data);
                break;
            case 0x605: CAN_POWER_Rx(&All_Power.P5, data);
                break;
            default:
                break;
        }
    }
}
