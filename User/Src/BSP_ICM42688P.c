//
// Created by CaoKangqi on 2026/1/23.
//
#include "BSP_ICM42688P.h"

float acc_res = 0;
float gyr_res = 0;
static uint8_t current_bank = 0xFF;

/* 内部私有：控制 CS */
static inline void ICM42688P_CS(uint8_t state) {
    HAL_GPIO_WritePin(ICM_CS_PORT, ICM_CS_PIN, (GPIO_PinState)state);
}

/* 内部私有：切换寄存器 Bank */
static void SelectBank(uint8_t bank) {
    if (current_bank != bank) {
        uint8_t cmd[2] = {REG_BANK_SEL, bank};
        ICM42688P_CS(0);
        HAL_SPI_Transmit(ICM_SPI_HANDLE, cmd, 2, 10);
        ICM42688P_CS(1);
        current_bank = bank;
    }
}

/* 内部私有：写寄存器 (支持自动切 Bank) */
static void WriteReg(uint16_t reg, uint8_t val) {
    SelectBank((uint8_t)(reg >> 8));
    uint8_t cmd[2] = {(uint8_t)(reg & 0xFF) & 0x7F, val};
    ICM42688P_CS(0);
    HAL_SPI_Transmit(ICM_SPI_HANDLE, cmd, 2, 10);
    ICM42688P_CS(1);
}

/* 内部私有：读寄存器 (支持自动切 Bank) */
static uint8_t ReadReg(uint16_t reg) {
    SelectBank((uint8_t)(reg >> 8));
    uint8_t addr = (uint8_t)(reg & 0xFF) | 0x80, val = 0;
    ICM42688P_CS(0);
    HAL_SPI_Transmit(ICM_SPI_HANDLE, &addr, 1, 10);
    HAL_SPI_Receive(ICM_SPI_HANDLE, &val, 1, 10);
    ICM42688P_CS(1);
    return val;
}

uint8_t ICM42688_Init(void) {
    // 1. 软复位
    WriteReg(REG_DEVICE_CONFIG, 0x01);
    HAL_Delay(100);

    // 2. ID 校验
    if (ReadReg(REG_WHO_AM_I) != ICM_WHO_AM_I_VAL) return 1;

    // 3. 配置中断引脚 (解决 DRDY 不拉高问题的关键)
    // INT_CONFIG (0x14): INT1 推挽输出, 高电平触发, 脉冲模式
    WriteReg(REG_INT_CONFIG, 0x00);
    // INT_SOURCE0 (0x65): 将 UI 数据就绪中断 (UI_DRDY) 路由到 INT1
    WriteReg(REG_INT_SOURCE0, 0x08);

    // 4. 电源管理：开启陀螺仪和加速度计的全性能模式 (LN Mode)
    WriteReg(REG_PWR_MGMT0, 0x0F);
    HAL_Delay(50);

    // 5. 默认配置量程和频率 (1kHz, 16g, 2000dps)
    ICM42688_SetFormat(ODR_1kHz, ACCEL_FS_16G, ODR_1kHz, GYRO_FS_2000DPS);

    return 0;
}

/**
 * @brief 设置采样率(ODR)和量程(FSR)
 */
void ICM42688_SetFormat(ODR_t a_odr, AccelFS_t a_fsr, ODR_t g_odr, GyroFS_t g_fsr) {
    // 1. 写入寄存器配置
    // ACCEL_CONFIG0: bit[7:5]是量程, bit[3:0]是频率
    WriteReg(REG_ACCEL_CONFIG0, (a_fsr << 5) | a_odr);
    // GYRO_CONFIG0:  bit[7:5]是量程, bit[3:0]是频率
    WriteReg(REG_GYRO_CONFIG0,  (g_fsr << 5) | g_odr);

    // 2. 自动更新物理量转换系数 (LSB to Physical Unit)
    // 原理：ICM42688 的 ADC 是 16 位的，范围 -32768 到 32767

    acc_res = (16.0f / (float)(1 << a_fsr)) / 32768.0f * 9.80665f;

    // 2. 陀螺仪转换系数：dps -> rad/s
    // 增加 0.01745329f (即 PI/180) 乘子
    gyr_res = (2000.0f / (float)(1 << g_fsr)) / 32768.0f * 0.01745329f;
}

/**
 * @brief 开启/关闭 FIFO 功能
 */
void ICM42688_Config_FIFO(uint8_t enable) {
    if(enable) {
        WriteReg(REG_FIFO_CONFIG, 0x40); // Stream-to-FIFO 模式
    } else {
        WriteReg(REG_FIFO_CONFIG, 0x00); // 禁用 FIFO
    }
}

uint8_t ICM42688_IsDataReady(void) {
    // 方式 A: 读取硬件引脚
    //return HAL_GPIO_ReadPin(ICM_DRDY_PORT, ICM_DRDY_PIN) == GPIO_PIN_SET;
    // 方式 B: 读取状态寄存器
     return (ReadReg(REG_INT_STATUS) & 0x08);
}

/**
 * @brief 读取并转换 ICM42688 数据
 * @param gyro 输出数组，存放 X, Y, Z 轴陀螺仪数据 (dps)
 * @param accel 输出数组，存放 X, Y, Z 轴加速度数据 (g)
 * @param temperature 输出指针，存放温度数据 (Celsius)
 */
void ICM42688_read(float gyro[3], float accel[3], float *temperature)
{
    uint8_t buf[14];
    int16_t raw_temp;

    // 1. 发送寄存器首地址 (温度寄存器地址) 并读取 14 字节连续数据
    // 数据顺序: Temp_H, Temp_L, Accel_X_H, Accel_X_L, ..., Gyro_Z_L
    uint8_t addr = (REG_TEMP_DATA1 & 0xFF) | 0x80;

    ICM42688P_CS(0);
    HAL_SPI_Transmit(ICM_SPI_HANDLE, &addr, 1, 10);
    HAL_SPI_Receive(ICM_SPI_HANDLE, buf, 14, 10);
    ICM42688P_CS(1);

    // 2. 处理加速度数据 (buf[2]~buf[7])
    // ICM42688 是 Big-Endian: MSB 在前
    accel[0] = (int16_t)((buf[2] << 8) | buf[3]) * acc_res;
    accel[1] = (int16_t)((buf[4] << 8) | buf[5]) * acc_res;
    accel[2] = (int16_t)((buf[6] << 8) | buf[7]) * acc_res;

    // 3. 处理陀螺仪数据 (buf[8]~buf[13])
    gyro[0] = (int16_t)((buf[8] << 8) | buf[9])   * gyr_res;
    gyro[1] = (int16_t)((buf[10] << 8) | buf[11]) * gyr_res;
    gyro[2] = (int16_t)((buf[12] << 8) | buf[13]) * gyr_res;

    // 4. 处理温度数据 (buf[0]~buf[1])
    raw_temp = (int16_t)((buf[0] << 8) | buf[1]);
    *temperature = (raw_temp / 132.48f) + 25.0f;
}