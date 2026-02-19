//
// Created by CaoKangqi on 2026/1/19.
//
#include "All_Task.h"
#include <stdio.h>

uint32_t stm32_id[3];
void Get_UID(uint32_t *uid) {
    uid[0] = HAL_GetUIDw0();
    uid[1] = HAL_GetUIDw1();
    uid[2] = HAL_GetUIDw2();
}

CCM_FUNC void MY_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
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
    ICM42688_Init();
    // 设置 FreeRTOS 任务周期为 1Tick (1ms)
    portTickType xLastWakeTime = xTaskGetTickCount();
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
        if(xTaskGetTickCount() - xLastWakeTime > 1)
        {
            xLastWakeTime = xTaskGetTickCount();
        }
        vTaskDelayUntil(&xLastWakeTime, 1);
    }
}
float a = 0;
uint8_t flash_id[3];
void Motor_Task(void *argument)
{
    (void)argument;
    Get_UID(stm32_id);
    //Motor_Mode(&hfdcan1,1,0x200,0xfc);
    for(;;)
    {
        /*W25N01GV_ReadID(flash_id);// ID 应该是 EF AA 21
        //Speed_Ctrl(&hfdcan1,1,IMU_Data.yaw);
        DM_Motor_Send(&hfdcan1, 0x3FE, a, 0, 0, 0);*/

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
        }
    }
}

CCM_DATA CAN_Stats_t can1_stats;
CCM_DATA CAN_Stats_t can2_stats;
CCM_DATA CAN_Stats_t can3_stats;
/**
 * @brief FDCAN FIFO0 接收中断回调函数
 * @note 优化要点：
 *       1. 循环读取FIFO直到为空，确保不丢帧
 *       2. 检测并处理FIFO溢出情况
 *       3. 减少中断内处理时间，提高实时性
 *       4. 统计接收数据，便于调试
 */
CCM_FUNC void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    FDCAN_RxHeaderTypeDef rx;
    uint8_t data[8];
    CAN_Stats_t *stats = NULL;

    // 确定统计结构
    if (hfdcan->Instance == FDCAN1)
        stats = &can1_stats;
    else if (hfdcan->Instance == FDCAN3)
        stats = &can3_stats;
    // 检测FIFO溢出，如果溢出则记录错误
    if (RxFifo0ITs & FDCAN_IT_RX_FIFO0_FULL)
    {
        if (stats) stats->fifo_full_count++;
    }
    if (RxFifo0ITs & FDCAN_IT_RX_FIFO0_MESSAGE_LOST)
    {
        if (stats) stats->msg_lost_count++;
    }
    // 循环读取FIFO中的所有消息，确保不遗漏
    uint32_t fill_level;
    while ((fill_level = HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0)) > 0)
    {
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx, data) != HAL_OK)
        {
            if (stats) stats->error_count++;
            break; // 读取失败，退出循环
        }
        if (stats) stats->rx_count++;
        // 根据不同的FDCAN实例和ID分发消息
        if (hfdcan->Instance == FDCAN1)
        {
            switch (rx.Identifier)
            {
                case 0x201:
                    DM_1to4_Resolve(&All_Motor.DM4310_Yaw, data);
                    break;
                case 0x207:
                    // MOTOR_CAN_RX_6020RM(&All_Motor.GM6020_1.DATA, data);
                    break;
                case 0x605:
                    CAN_POWER_Rx(&All_Power.P5, data);
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
                case 0x207:
                    // MOTOR_CAN_RX_6020RM(&All_Motor.GM6020_1.DATA, data);
                    break;
                case 0x605:
                    CAN_POWER_Rx(&All_Power.P5, data);
                    break;
                default:
                    break;
            }
        }
        // 安全保护：避免死循环（理论上不应该发生）
        if (fill_level > 64) break; // FIFO最大深度一般不超过64
    }
}

/**
 * @brief FDCAN FIFO1 接收中断回调函数
 * @note 优化要点同FIFO0
 */
CCM_FUNC void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
    FDCAN_RxHeaderTypeDef rx;
    uint8_t data[8];
    CAN_Stats_t *stats = NULL;

    // 确定统计结构
    if (hfdcan->Instance == FDCAN2)
        stats = &can2_stats;

    // 检测FIFO溢出
    if (RxFifo1ITs & FDCAN_IT_RX_FIFO1_FULL)
    {
        if (stats) stats->fifo_full_count++;
    }

    if (RxFifo1ITs & FDCAN_IT_RX_FIFO1_MESSAGE_LOST)
    {
        if (stats) stats->msg_lost_count++;
    }

    // 循环读取FIFO中的所有消息
    uint32_t fill_level;
    while ((fill_level = HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO1)) > 0)
    {
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &rx, data) != HAL_OK)
        {
            if (stats) stats->error_count++;
            break;
        }

        if (stats) stats->rx_count++;

        // FDCAN2使用FIFO1
        if (hfdcan->Instance == FDCAN2)
        {
            switch (rx.Identifier)
            {
                case 0x207:
                    // GM6020_Decode(&All_Motor.GM6020_1, data);
                    break;
                case 0x605:
                    CAN_POWER_Rx(&All_Power.P5, data);
                    break;
                default:
                    break;
            }
        }

        // 安全保护
        if (fill_level > 64) break;
    }
}