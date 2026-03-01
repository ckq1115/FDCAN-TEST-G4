//
// Created by CaoKangqi on 2026/1/19.
//
#include "All_Task.h"
#include <stdio.h>

#include "Chassis_Task.h"
#include "Test_Task.h"

CCM_FUNC void MY_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
        if (htim->Instance == TIM4) {
            System_Root(&ROOT_Status, &C_DBUS, &All_Motor, NULL);
        }
}
static uint8_t icm_tx_buf[15];//包含寄存器地址和14字节数据，预先填充寄存器地址以优化DMA读取
static uint8_t icm_rx_buf[15];//包含寄存器地址和14字节数据，预先填充寄存器地址以优化DMA读取
static uint8_t icm_raw_cache[14];//DMA读取完成后会先存放在这里，等待主任务处理转换为物理量
static TaskHandle_t xIMUTaskHandle = NULL;
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    // 显式声明并初始化xHigherPriorityTaskWoken
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (GPIO_Pin == ICM_DRDY_PIN) {
        ICM42688_StartRead_IntDMA(icm_tx_buf, icm_rx_buf);
        // 清除MCU EXTI挂起位
        __HAL_GPIO_EXTI_CLEAR_IT(ICM_DRDY_PIN);
        // 触发APP层任务
        if(xIMUTaskHandle != NULL) {
            xTaskNotifyFromISR(xIMUTaskHandle,
                              0,
                              eIncrement,
                              &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
}
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi == ICM_SPI_HANDLE) {
        ICM_CS_PORT->BSRR = ICM_CS_PIN;
        memcpy(icm_raw_cache, &icm_rx_buf[1], sizeof(icm_raw_cache));
    }
}

static uint32_t INS_DWT_Count = 0; // DWT计数基准
static float imu_period_s = 0.0f;
void IMU_Task(void *argument)
{
    (void)argument;
    xIMUTaskHandle = xTaskGetCurrentTaskHandle();
    ICM42688_Init();
    // 预填充DMA缓冲区：寄存器地址+0xFF占位数据，优化后续DMA读取效率
    icm_tx_buf[0] = (REG_TEMP_DATA1 | 0x80);
    for (int i = 1; i < (int)sizeof(icm_tx_buf); i++) {
        icm_tx_buf[i] = 0xFF;
    }
    INS_DWT_Count = DWT->CYCCNT;
    for(;;)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        imu_period_s = DWT_GetDeltaT(&INS_DWT_Count);
        //ICM42688_Read_Fast(IMU_Data.gyro, IMU_Data.accel,&IMU_Data.temp);
        ICM42688_ResolveRaw(icm_raw_cache, IMU_Data.gyro, IMU_Data.accel,&IMU_Data.temp);
        IMU_Update_Task(imu_period_s);
    }
}

uint8_t flash_id[3] = {0};
void Motor_Task(void *argument)
{
    (void)argument;
    W25N01GV_Init();
    if (Chassis_Control_Init(&All_Motor) != DF_READY)
    {
        Error_Handler();
    }
    //Motor_Mode(&hfdcan1,1,0x200,0xfc);
    for(;;)
    {
        Chassis_Control_Task(&All_Motor);
        //W25N01GV_ReadID(flash_id);// ID 应该是 EF AA 21
        VOFA_justfloat(
            IMU_Data.pitch,
            IMU_Data.roll,
            IMU_Data.yaw,
            IMU_Data.YawTotalAngle,
            All_Motor.DJI_3508_Chassis[1].DATA.Speed_now,
            All_Motor.DJI_3508_Chassis[2].PID_S.Output,
            All_Motor.DJI_3508_Chassis[2].DATA.Speed_now,
            m,All_Power.P5.power,60);
        osDelay(1);
    }
}

void Test_Task(void *argument)
{
    Test_Init();
    for(;;)
    {
        Ctrl_Test_Task();
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
                    DJI_Motor_Resolve(&All_Motor.DJI_3508_Chassis[0], data);
                    break;
                case 0x202:
                    DJI_Motor_Resolve(&All_Motor.DJI_3508_Chassis[1], data);
                    break;
                case 0x203:
                    DJI_Motor_Resolve(&All_Motor.DJI_3508_Chassis[2], data);
                    break;
                case 0x204:
                    DJI_Motor_Resolve(&All_Motor.DJI_3508_Chassis[3], data);
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
                case 0x207:
                    DJI_Motor_Resolve(&All_Motor.DJI_6020_Pitch, data);
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