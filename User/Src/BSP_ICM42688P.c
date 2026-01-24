//
// Created by CaoKangqi on 2026/1/23.
//
#include "BSP_ICM42688P.h"

static float acc_res = 0, gyr_res = 0;
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
    ICM42688_SetODR_FSR(0x06, 0x00, 0x06, 0x00);

    return 0;
}

/**
 * @brief 设置采样率(ODR)和量程(FSR)
 */
void ICM42688_SetODR_FSR(uint8_t a_odr, uint8_t a_fsr, uint8_t g_odr, uint8_t g_fsr) {
    // Accel: bit[3:0] ODR, bit[7:5] FSR
    WriteReg(REG_ACCEL_CONFIG0, (a_fsr << 5) | a_odr);
    // Gyro:  bit[3:0] ODR, bit[7:5] FSR
    WriteReg(REG_GYRO_CONFIG0,  (g_fsr << 5) | g_odr);

    // 更新转换因子
    float acc_scales[] = {16.0f, 8.0f, 4.0f, 2.0f};
    float gyr_scales[] = {2000.0f, 1000.0f, 500.0f, 250.0f, 125.0f, 62.5f, 31.25f, 15.625f};
    acc_res = acc_scales[a_fsr] / 32768.0f;
    gyr_res = gyr_scales[g_fsr] / 32768.0f;
}

/**
 * @brief 开启/关闭 FIFO 功能 (高级功能)
 */
void ICM42688_Config_FIFO(uint8_t enable) {
    if(enable) {
        WriteReg(REG_FIFO_CONFIG, 0x40); // Stream-to-FIFO 模式
    } else {
        WriteReg(REG_FIFO_CONFIG, 0x00); // 禁用 FIFO
    }
}

uint8_t ICM42688_IsDataReady(void) {
    // 方式 A: 读取硬件引脚 (推荐，如果你连接了 DRDY 引脚)
    //return HAL_GPIO_ReadPin(ICM_DRDY_PORT, ICM_DRDY_PIN) == GPIO_PIN_SET;

    // 方式 B: 如果没连引脚，可以读取状态寄存器
     return (ReadReg(REG_INT_STATUS) & 0x08);
}

void ICM42688_Update(ICM42688_t *dev) {
    uint8_t buf[14];
    uint8_t addr = (REG_TEMP_DATA1 & 0xFF) | 0x80;

    ICM42688P_CS(0);
    HAL_SPI_Transmit(ICM_SPI_HANDLE, &addr, 1, 10);
    HAL_SPI_Receive(ICM_SPI_HANDLE, buf, 14, 10);
    ICM42688P_CS(1);

    // 大端转小端并赋值
    dev->raw.temp   = (int16_t)((buf[0] << 8) | buf[1]);
    dev->raw.acc[0] = (int16_t)((buf[2] << 8) | buf[3]);
    dev->raw.acc[1] = (int16_t)((buf[4] << 8) | buf[5]);
    dev->raw.acc[2] = (int16_t)((buf[6] << 8) | buf[7]);
    dev->raw.gyr[0] = (int16_t)((buf[8] << 8) | buf[9]);
    dev->raw.gyr[1] = (int16_t)((buf[10] << 8) | buf[11]);
    dev->raw.gyr[2] = (int16_t)((buf[12] << 8) | buf[13]);

    // 转换为物理单位
    for(int i=0; i<3; i++) {
        dev->acc_g[i]   = dev->raw.acc[i] * acc_res;
        dev->gyr_dps[i] = dev->raw.gyr[i] * gyr_res;
    }
    dev->temp_c = (dev->raw.temp / 132.48f) + 25.0f;
}