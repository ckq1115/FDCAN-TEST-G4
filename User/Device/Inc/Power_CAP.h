//
// Created by CaoKangqi on 2026/2/14.
//

#ifndef FDCAN_TEST_G4_POWER_CAP_H
#define FDCAN_TEST_G4_POWER_CAP_H
#include <stdint.h>


typedef struct  // 接收的电容的数据
{
    float BUFFER;        // 电容缓冲
    float CAP_VOLT;      // 电容电压
    float NOW_POWER;     // 当前总功率
    int16_t OUT_BOLL;      // 当前电容是否在输出
    int8_t ONLINE_JUDGE_TIME;
}CAP_RXDATA;

typedef union  // 发给电容的数据//使用共用体整合数据
{
    struct __packed
    {
        uint8_t BUFFER_NOW; // 当前缓冲
        uint8_t MAX_POWER;  // 最大功率
        uint8_t VOLT;       // 当前电池电压，因为最大电压大于25.5V，处理时加一个50
        uint8_t OUT_POWER;  // 当前总功率
        uint8_t OUT_SWITCH;
        uint8_t POWER_KEY;
    }CHANNAL;
    // CAN发送的数据
    uint8_t SEND_DATA[8];
}CAP_SETDATA;

typedef struct
{
    CAP_SETDATA SET;
    CAP_RXDATA GET;
} CAPDATE_TYPDEF;

void Power_CAP_CAN_RX(CAPDATE_TYPDEF *CAP_DATA, uint8_t *DATA);
//void Power_CAP_CAN_TX(hcan_t* hcan,uint16_t cap_id, CAP_SETDATA *CAP_SET, User_Data_T *User_data);

#endif //FDCAN_TEST_G4_POWER_CAP_H