//
// Created by CaoKangqi on 2026/1/19.
//
#include "All_Task.h"

#include <stdint.h>
float a=0;
uint8_t DBUS_RX_DATA[19] = { 0 };
DBUS_Typedef C_DBUS = { 0 };
DBUS_UNION_Typdef C_DBUS_UNION = { 0 };
ICM42688_t imu;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM4) {
        a+=0.001;
        if (ICM42688_IsDataReady()) {
            ICM42688_Update(&imu);
        }
        //DJI_Current_Ctrl(&hfdcan3,0x1FE,0,0,1000,0);
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance ==USART3)//遥控接收串口
    {
        DBUS_Resolved(DBUS_RX_DATA, &C_DBUS,&C_DBUS_UNION);
        HAL_UART_Receive_DMA(&huart3, (uint8_t *)DBUS_RX_DATA,37);  //重启开始DMA传输

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

void All_Init() {
    FDCAN1_Config();
    FDCAN2_Config();
    FDCAN3_Config();
    HAL_TIM_Base_Start_IT(&htim4);
    HAL_DMA_DeInit(&hdma_usart3_rx);
    HAL_DMA_Init(&hdma_usart3_rx);
    HAL_UART_DMAStop(&huart3);
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);//遥控串口
    HAL_UART_Receive_DMA(&huart3,(uint8_t *)DBUS_RX_DATA,37);
    WS2812_Init();
    if (ICM42688_Init() != 0) {
        Error_Handler();
    }
}