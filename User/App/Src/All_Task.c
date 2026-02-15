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
uint8_t test_buf[2048];
uint8_t read_buf[2048];
uint8_t flash_id[3];
void Motor_Task(void *argument)
{
    (void)argument;
    //Motor_Mode(&hfdcan1,1,0x200,0xfc);
    for(;;)
    {
        W25N01GV_ReadID(flash_id);// ID 应该是 EF AA 21
        // 2. 擦除第 10 块
        W25N01GV_EraseBlock(10);
        // 3. 准备测试数据并写入
        for(int i=0; i<2048; i++) test_buf[i] = i % 256;
        W25N01GV_WritePage(10 * 64, test_buf, 2048);
        // 4. 读回并验证
        W25N01GV_ReadPage(10 * 64, read_buf, 2048);
        //Speed_Ctrl(&hfdcan1,1,IMU_Data.yaw);
        DM_Motor_Send(&hfdcan1, 0x3FE, a, 0, 0, 0);
        Get_UID(stm32_id);
        /*VOFA_justfloat(
            dt_s,
            IMU_Data.pitch,
            IMU_Data.roll,
            IMU_Data.yaw,
            IMU_Data.YawTotalAngle,
            imu_period_s,imu_operate_us,0,0,0);*/
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
                case 0x101:
                    break;
                case 0x201: DM_1to4_Resolve(&All_Motor.DM4310_Yaw,g_Can1RxData);

                    default:
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
			}
		}
	}
}
