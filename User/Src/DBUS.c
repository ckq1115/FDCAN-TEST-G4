//
// Created by CaoKangqi on 2026/1/19.
//
#include "main.h"
#include "DBUS.h"

remote_typedef remote_t;

/**
 * @brief  遥控数据解算
 * @param  data 串口接收数组
 * @retval void
 */
void DBUS_Resolve(uint8_t *data)
{
    /*************遥控数据*************************/
    remote_t.control.ch0_int16 = (data[0] | (int16_t)data[1] << 8) & 0x07FF;
    remote_t.control.ch1_int16 = (data[1] >> 3 | (int16_t)data[2] << 5) & 0x07FF;
    remote_t.control.ch2_int16 = (data[2] >> 6 | (int16_t)data[3] << 2 | data[4] << 10) & 0x07FF;
    remote_t.control.ch3_int16 = (data[4] >> 1 | (int16_t)data[5] << 7) & 0x07FF;

    remote_t.control.dial_int16 = (data[16] | (int16_t)data[17] << 8) & 0x07FF;

    if (remote_t.control.ch0_int16 && remote_t.control.ch1_int16 && remote_t.control.ch2_int16 && remote_t.control.ch3_int16) {
        remote_t.control.ch0_int16 -= 1024;
        remote_t.control.ch1_int16 -= 1024;
        remote_t.control.ch2_int16 -= 1024;
        remote_t.control.ch3_int16 -= 1024;
        remote_t.control.dial_int16 -= 1024;
    }

    // 死区控制
    if (remote_t.control.ch0_int16 < 10 && remote_t.control.ch0_int16 > -10) {
        remote_t.control.ch0_int16 = 0;
    }
    if (remote_t.control.ch0_int16 < 10 && remote_t.control.ch0_int16 > -10) {
        remote_t.control.ch0_int16 = 0;
    }
    if (remote_t.control.ch0_int16 < 10 && remote_t.control.ch0_int16 > -10) {
        remote_t.control.ch0_int16 = 0;
    }
    if (remote_t.control.ch0_int16 < 10 && remote_t.control.ch0_int16 > -10) {
        remote_t.control.ch0_int16 = 0;
    }

    remote_t.control.last_s1 = remote_t.control.s1_u8;
    remote_t.control.s1_u8   = (data[5] >> 4) & 0x03;
    remote_t.control.s2_u8   = (data[5] >> 6) & 0x03;
}

void DBUS_offLine(void)
{
    remote_t.control.ch0_int16 = 0;
    remote_t.control.ch1_int16 = 0;
    remote_t.control.ch2_int16 = 0;
    remote_t.control.ch3_int16 = 0;
    remote_t.control.s1_u8     = 3;
    remote_t.control.s2_u8     = 3;
}
