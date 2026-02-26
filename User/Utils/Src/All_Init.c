//
// Created by CaoKangqi on 2026/2/13.
//
#include "All_Init.h"

//DBUS
uint8_t DBUS_RX_DATA[18];
DBUS_Typedef C_DBUS = { 0 };
DBUS_UNION_Typdef C_DBUS_UNION = { 0 };

CCM_DATA MOTOR_Typdef All_Motor;
CCM_DATA ROOT_STATUS_Typedef ROOT_Status;

uint32_t stm32_id[3];
void Get_UID(uint32_t *uid) {
    uid[0] = HAL_GetUIDw0();
    uid[1] = HAL_GetUIDw1();
    uid[2] = HAL_GetUIDw2();
}
void All_Init() {
    DWT_Init(170);
    Get_UID(stm32_id);
    /* 清除串口错误标志 */
    HAL_DMA_DeInit(&hdma_usart3_rx);
    HAL_DMA_Init(&hdma_usart3_rx);
    HAL_UART_DMAStop(&huart3);
    __HAL_UART_CLEAR_OREFLAG(&huart3);
    __HAL_UART_CLEAR_FEFLAG(&huart3);
    __HAL_UART_CLEAR_NEFLAG(&huart3);
    __HAL_UART_CLEAR_PEFLAG(&huart3);
    volatile uint32_t tmp = huart3.Instance->RDR;
    (void)tmp;
    __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);//关闭 DMA 半传中断
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3,DBUS_RX_DATA,18);//启动 DMA + IDLE
    FDCAN_Config(&hfdcan1, FDCAN_RX_FIFO0);
    FDCAN_Config(&hfdcan2, FDCAN_RX_FIFO1);
    FDCAN_Config(&hfdcan3, FDCAN_RX_FIFO0);
    WS2812_Init();
    W25N01GV_Init();
    IMU_Gyro_Calib_Initiate();
    HAL_TIM_Base_Start_IT(&htim4);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim20, TIM_CHANNEL_2);
    __HAL_TIM_SET_COMPARE(&htim20, TIM_CHANNEL_2, 100.0);
    HAL_Delay(500);
    __HAL_TIM_SET_COMPARE(&htim20, TIM_CHANNEL_2, 0);
    //HAL_TIM_PWM_Stop(&htim20, TIM_CHANNEL_2);
}