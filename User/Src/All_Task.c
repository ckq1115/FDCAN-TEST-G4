//
// Created by CaoKangqi on 2026/1/19.
//
#include "All_Task.h"

#include <stdint.h>

#include "BSP_DWT.h"

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


void StartDefaultTask(void *argument)
{
    /* USER CODE BEGIN StartDefaultTask */
    uint32_t last_wake_time = DWT->CYCCNT; // 初始化记录时间戳
    float task_period = 0;
    /* Infinite loop */
    for(;;)
    {
        task_period = DWT_GetDeltaT(&last_wake_time);
        run_time_s = DWT_GetTimeline_s();
        //uint32_t cnt_last = DWT->CYCCNT;
        ICM42688_read(IMU_Data.gyro, IMU_Data.accel,&IMU_Data.temp);
        IMU_Temp_Control_Task();
        VOFA_justfloat(IMU_Data.accel[0],
            IMU_Data.accel[1],
            IMU_Data.accel[2],
            IMU_Data.yaw,
            IMU_Data.pitch,
            IMU_Data.roll,run_time_s,QEKF_INS.ChiSquare_Data[0],task_period,0);
        //dt_s = DWT_GetDeltaT(&cnt_last);
        osDelay(1);
    }
    /* USER CODE END StartDefaultTask */
}

uint8_t DBUS_RX_DATA[18];//__attribute__((section(".ARM.__at_0x24000000")));
DBUS_Typedef C_DBUS = { 0 };
DBUS_UNION_Typdef C_DBUS_UNION = { 0 };
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){
    if (huart->Instance == USART3){
        if (Size == 18){
            DBUS_Resolved(DBUS_RX_DATA, &C_DBUS, &C_DBUS_UNION);
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
    if (ICM42688_Init() != 0) {
        Error_Handler();
    }
    IMU_Gyro_Calib_Initiate();
    HAL_TIM_Base_Start_IT(&htim4);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim20, TIM_CHANNEL_2);
}