//
// Created by CaoKangqi on 2026/1/23.
//
#include "BSP_ICM42688P.h"

#include <tgmath.h>

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
    ICM42688_SetFormat(ODR_1kHz, ACCEL_FS_8G, ODR_1kHz, GYRO_FS_2000DPS);

    ICM42688_Config_AAF(1, 7, 49, 9);
    ICM42688_Config_AAF(0, 7, 49, 9);
    ICM42688_Config_UI_Filter(UI_FILT_ORD_2ND, UI_FILT_BW_ODR_DIV_4, UI_FILT_ORD_2ND, UI_FILT_BW_ODR_DIV_4);
    //ICM42688_Config_Gyro_Notch_Filter(1, 1000.0f, GYRO_NF_BW_40HZ);
    SelectBank(0);
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

    acc_res = (16.0f / (float)(1 << a_fsr)) / 32768.0f * 9.81f;

    // 2. 陀螺仪转换系数：dps -> °/s
    gyr_res = (2000.0f / (float)(1 << g_fsr)) / 32768.0f;
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
/**
 * @brief 读取并转换 ICM42688 数据
 * @param gyro 输出数组，存放 X, Y, Z 轴陀螺仪数据 (dps)
 * @param accel 输出数组，存放 X, Y, Z 轴加速度数据 (g)
 * @param temperature 输出指针，存放温度数据 (Celsius)
 * @note 通过直接操作 SPI 寄存器实现更快的数据读取，耗时约为ICM42688_read的一半
 */
void ICM42688_Read_Fast(float gyro[3], float accel[3], float *temperature)
{
    uint8_t raw_data[14];

    // 1. 获取 SPI 寄存器基地址
    // 既然 ICM_SPI_HANDLE 是 &hspi2 (指针)，这里使用 -> 访问 Instance
    SPI_TypeDef *spi_inst = ((SPI_HandleTypeDef *)ICM_SPI_HANDLE)->Instance;

    // 2. 拉低 CS
    ICM_CS_PORT->BSRR = (uint32_t)ICM_CS_PIN << 16;

    // 3. 发送首地址 (READ command)
    // 等待发送缓冲区为空 (TXE)
    while (!(spi_inst->SR & SPI_FLAG_TXE));

    // 强制按字节(8bit)写入 DR 寄存器，防止 G4 的 FIFO 自动发送 16bit
    *(__IO uint8_t *)&spi_inst->DR = (REG_TEMP_DATA1 | 0x80);

    // 等待接收非空 (RXNE)
    while (!(spi_inst->SR & SPI_FLAG_RXNE));
    __IO uint8_t trash = *(__IO uint8_t *)&spi_inst->DR; // 读出首字节产生的废数据
    (void)trash;

    // 4. 连续读取 14 字节
    for (int i = 0; i < 14; i++) {
        while (!(spi_inst->SR & SPI_FLAG_TXE)); // 确保上一次发送完成
        *(__IO uint8_t *)&spi_inst->DR = 0xFF;  // 发送 Dummy 产生时钟

        while (!(spi_inst->SR & SPI_FLAG_RXNE)); // 等待数据回来
        raw_data[i] = *(__IO uint8_t *)&spi_inst->DR;
    }

    // 5. 拉高 CS
    ICM_CS_PORT->BSRR = ICM_CS_PIN;

    // 6. 数据解算 (使用 FPU 硬件加速)
    int16_t t_raw = (int16_t)((raw_data[0] << 8) | raw_data[1]);

    // 加速度和陀螺仪直接通过指针转换，减少移位开销
    accel[0] = (int16_t)((raw_data[2] << 8) | raw_data[3]) * acc_res;
    accel[1] = (int16_t)((raw_data[4] << 8) | raw_data[5]) * acc_res;
    accel[2] = (int16_t)((raw_data[6] << 8) | raw_data[7]) * acc_res;

    gyro[0] = (int16_t)((raw_data[8] << 8) | raw_data[9])   * gyr_res;
    gyro[1] = (int16_t)((raw_data[10] << 8) | raw_data[11]) * gyr_res;
    gyro[2] = (int16_t)((raw_data[12] << 8) | raw_data[13]) * gyr_res;

    *temperature = (t_raw / 132.48f) + 25.0f;
}

/* ==============================================================================
 * 滤波器配置函数
 * ============================================================================== */

#ifndef ICM_PI
#define ICM_PI 3.14159265358979323846f
#endif

void ICM42688_Config_UI_Filter(UIFiltOrd_t a_ord, UIFiltBW_t a_bw, UIFiltOrd_t g_ord, UIFiltBW_t g_bw) {
    // 1. 配置加速度计滤波器阶数 (ACCEL_CONFIG1)
    // bit[4:3] = ACCEL_UI_FILT_ORD
    uint8_t reg_acc_conf1 = ReadReg(REG_ACCEL_CONFIG1);
    reg_acc_conf1 &= ~(0x18); // 清除 bit 4:3
    reg_acc_conf1 |= (a_ord << 3);
    WriteReg(REG_ACCEL_CONFIG1, reg_acc_conf1);

    // 2. 配置陀螺仪滤波器阶数 (GYRO_CONFIG1)
    // bit[3:2] = GYRO_UI_FILT_ORD
    uint8_t reg_gyr_conf1 = ReadReg(REG_GYRO_CONFIG0); // 注意：这里需要读的是 GYRO_CONFIG1 对应的地址，原宏定义是 REG_GYRO_CONFIG1
    // 修正：需要在头文件定义 REG_GYRO_CONFIG1 (0x0051)
    // 假设上方头文件已补充定义，这里直接读写 REG_GYRO_CONFIG1
    // 若原代码未定义，请使用 0x0051
    uint8_t val_gyr_c1 = ReadReg(REG_GYRO_CONFIG1);
    val_gyr_c1 &= ~(0x0C); // 清除 bit 3:2
    val_gyr_c1 |= (g_ord << 2);
    WriteReg(REG_GYRO_CONFIG1, val_gyr_c1);

    // 3. 配置带宽 (GYRO_ACCEL_CONFIG0)
    // bit[7:4] = ACCEL_UI_FILT_BW, bit[3:0] = GYRO_UI_FILT_BW
    uint8_t bw_reg = (a_bw << 4) | (g_bw & 0x0F);
    WriteReg(REG_GYRO_ACCEL_CONFIG0, bw_reg);
}

void ICM42688_Config_Gyro_Notch_Filter(uint8_t enable, float center_freq_hz, GyroNFBW_t bw_sel) {
    // 1. 开关控制 (GYRO_CONFIG_STATIC2, Bank 1, 0x0B)
    // Bit 0: GYRO_NF_DIS (0=Enable, 1=Disable)
    uint8_t static2 = ReadReg(REG_GYRO_CONFIG_STATIC2);
    if (enable) {
        static2 &= ~0x01; // Enable (Bit 0 = 0)
    } else {
        static2 |= 0x01;  // Disable (Bit 0 = 1)
        WriteReg(REG_GYRO_CONFIG_STATIC2, static2);
        return; // 如果禁用，无需计算参数
    }
    WriteReg(REG_GYRO_CONFIG_STATIC2, static2);

    // 2. 计算参数 (参考 Datasheet 5.2 节)
    // f_desired 在 kHz 单位
    float f_desired_khz = center_freq_hz / 1000.0f;
    float coswz = cosf(2 * ICM_PI * f_desired_khz / 32.0f); // 32kHz is internal clock

    int16_t nf_coswz_val;
    uint8_t nf_coswz_sel;

    if (fabsf(coswz) <= 0.875f) {
        nf_coswz_val = (int16_t)roundf(coswz * 256.0f);
        nf_coswz_sel = 0;
    } else {
        nf_coswz_sel = 1;
        if (coswz > 0.875f) {
            nf_coswz_val = (int16_t)roundf(8.0f * (1.0f - coswz) * 256.0f);
        } else { // coswz < -0.875f
            nf_coswz_val = (int16_t)roundf(-8.0f * (1.0f + coswz) * 256.0f);
        }
    }

    // 3. 写入参数 (X, Y, Z 轴设为相同)
    // NF_COSWZ[7:0] 写到 STATIC6(X), STATIC7(Y), STATIC8(Z)
    uint8_t low_byte = (uint8_t)(nf_coswz_val & 0xFF);
    WriteReg(REG_GYRO_CONFIG_STATIC6, low_byte);
    WriteReg(REG_GYRO_CONFIG_STATIC7, low_byte);
    WriteReg(REG_GYRO_CONFIG_STATIC8, low_byte);

    // STATIC9 包含 MSB (第8位) 和 SEL 位
    // Bit 0: X_NF_COSWZ[8]
    // Bit 1: Y_NF_COSWZ[8]
    // Bit 2: Z_NF_COSWZ[8]
    // Bit 3: X_NF_COSWZ_SEL[0]
    // Bit 4: Y_NF_COSWZ_SEL[0]
    // Bit 5: Z_NF_COSWZ_SEL[0]
    uint8_t msb = (nf_coswz_val >> 8) & 0x01;
    uint8_t static9 = 0;

    // 设置 X, Y, Z 的 MSB
    if(msb) static9 |= (1<<0) | (1<<1) | (1<<2);
    // 设置 X, Y, Z 的 SEL
    if(nf_coswz_sel) static9 |= (1<<3) | (1<<4) | (1<<5);

    WriteReg(REG_GYRO_CONFIG_STATIC9, static9);

    // 4. 设置带宽 (GYRO_CONFIG_STATIC10)
    // Bit[6:4] = GYRO_NF_BW_SEL
    uint8_t static10 = ReadReg(REG_GYRO_CONFIG_STATIC10);
    static10 &= ~(0x70);
    static10 |= (bw_sel << 4);
    WriteReg(REG_GYRO_CONFIG_STATIC10, static10);
}

void ICM42688_Config_AAF(uint8_t is_accel, uint8_t delt, uint16_t deltSqr, uint8_t bitshift) {
    if (is_accel) {
        // 加速度计配置 (Bank 2)
        // Static2: Bit 0 = AAF_DIS (1=Disable), Bit [6:1] = DELT
        if (delt == 0) {
            // Disable
            uint8_t val = ReadReg(REG_ACCEL_CONFIG_STATIC2);
            WriteReg(REG_ACCEL_CONFIG_STATIC2, val | 0x01);
        } else {
            // Enable and set parameters
            WriteReg(REG_ACCEL_CONFIG_STATIC2, (delt << 1) & 0xFE); // Enable(0) | DELT
            WriteReg(REG_ACCEL_CONFIG_STATIC3, (uint8_t)(deltSqr & 0xFF));
            // Static4: Bit[7:4]=Bitshift, Bit[3:0]=deltSqr[11:8]
            uint8_t static4 = (bitshift << 4) | ((deltSqr >> 8) & 0x0F);
            WriteReg(REG_ACCEL_CONFIG_STATIC4, static4);
        }
    } else {
        // 陀螺仪配置 (Bank 1)
        // Static2: Bit 1 = AAF_DIS
        uint8_t static2 = ReadReg(REG_GYRO_CONFIG_STATIC2);
        if (delt == 0) {
            WriteReg(REG_GYRO_CONFIG_STATIC2, static2 | 0x02); // Disable
        } else {
            WriteReg(REG_GYRO_CONFIG_STATIC2, static2 & ~0x02); // Enable

            // Static3: Bit[5:0] = DELT
            WriteReg(REG_GYRO_CONFIG_STATIC3, delt & 0x3F);
            WriteReg(REG_GYRO_CONFIG_STATIC4, (uint8_t)(deltSqr & 0xFF));
            // Static5: Bit[7:4]=Bitshift, Bit[3:0]=deltSqr[11:8]
            uint8_t static5 = (bitshift << 4) | ((deltSqr >> 8) & 0x0F);
            WriteReg(REG_GYRO_CONFIG_STATIC5, static5);
        }
    }
}