//
// Created by CaoKangqi on 2026/2/13.
//
#include "All_Init.h"

//DBUS
uint8_t DBUS_RX_DATA[18];
DBUS_Typedef C_DBUS = { 0 };
DBUS_UNION_Typdef C_DBUS_UNION = { 0 };

//IMU






void All_Init() {
    DWT_Init(170);
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
    FDCAN1_Config();
    FDCAN2_Config();
    FDCAN3_Config();
    WS2812_Init();

    IMU_Gyro_Calib_Initiate();
    HAL_TIM_Base_Start_IT(&htim4);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim20, TIM_CHANNEL_2);
    __HAL_TIM_SET_COMPARE(&htim20, TIM_CHANNEL_2, 50);
    HAL_Delay(500);
    __HAL_TIM_SET_COMPARE(&htim20, TIM_CHANNEL_2, 0);
}