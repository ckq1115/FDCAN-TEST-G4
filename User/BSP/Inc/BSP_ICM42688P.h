//
// Created by CaoKangqi on 2026/1/23.
//

#ifndef BSP_ICM42688P_H
#define BSP_ICM42688P_H

#include "main.h"

extern SPI_HandleTypeDef hspi2;
#define ICM_SPI_HANDLE      &hspi2

/* 引脚定义 */
#define ICM_CS_PORT         GPIOB
#define ICM_CS_PIN          GPIO_PIN_12
#define ICM_DRDY_PORT       GPIOB
#define ICM_DRDY_PIN        GPIO_PIN_10

/* 寄存器地址 (带 Bank 标记：高4位为 Bank 索引，低8位为地址) */
#define REG_BANK_SEL        0x76

// BANK 0
#define REG_DEVICE_CONFIG   0x0011
#define REG_DRIVE_CONFIG    0x0013
#define REG_INT_CONFIG      0x0014
#define REG_FIFO_CONFIG     0x0016
#define REG_TEMP_DATA1      0x001D
#define REG_ACCEL_DATA_X1   0x001F
#define REG_INT_STATUS      0x002D
#define REG_FIFO_STH_MSB    0x002E
#define REG_PWR_MGMT0       0x004E
#define REG_GYRO_CONFIG0    0x004F
#define REG_ACCEL_CONFIG0   0x0050
#define REG_GYRO_ACCEL_CONFIG0 0x0052
#define REG_INT_SOURCE0     0x0065
#define REG_WHO_AM_I        0x0075
#define REG_GYRO_CONFIG1        0x0051
#define REG_ACCEL_CONFIG1       0x0053
// BANK 1
#define REG_GYRO_CONFIG_STATIC2 0x010B
#define REG_GYRO_CONFIG_STATIC3 0x010C
#define REG_GYRO_CONFIG_STATIC4 0x010D
#define REG_GYRO_CONFIG_STATIC5 0x010E
#define REG_GYRO_CONFIG_STATIC6 0x010F  // X COSWZ[7:0]
#define REG_GYRO_CONFIG_STATIC7 0x0110  // Y COSWZ[7:0]
#define REG_GYRO_CONFIG_STATIC8 0x0111  // Z COSWZ[7:0]
#define REG_GYRO_CONFIG_STATIC9 0x0112  // MSBs and SEL
#define REG_GYRO_CONFIG_STATIC10 0x0113 // BW_SEL

// BANK 2
#define REG_ACCEL_CONFIG_STATIC2 0x0203
#define REG_ACCEL_CONFIG_STATIC3 0x0204
#define REG_ACCEL_CONFIG_STATIC4 0x0205
// 常量定义
#define ICM_WHO_AM_I_VAL    0x47

typedef struct {
    struct {
        int16_t acc[3];
        int16_t gyr[3];
        int16_t temp;
    } raw;
    float acc_g[3];
    float gyr_dps[3];
    float temp_c;
} ICM42688_t;

/* --- 加速度计量程枚举 --- */
typedef enum {
    ACCEL_FS_16G = 0,  // 对应寄存器值 0x00
    ACCEL_FS_8G  = 1,  // 对应寄存器值 0x01
    ACCEL_FS_4G  = 2,  // 对应寄存器值 0x02
    ACCEL_FS_2G  = 3,  // 对应寄存器值 0x03
} AccelFS_t;

/* --- 陀螺仪量程枚举 --- */
typedef enum {
    GYRO_FS_2000DPS = 0,
    GYRO_FS_1000DPS = 1,
    GYRO_FS_500DPS  = 2,
    GYRO_FS_250DPS  = 3,
    GYRO_FS_125DPS  = 4,
    GYRO_FS_62_5DPS = 5,
    GYRO_FS_31_25DPS = 6,
    GYRO_FS_15_625DPS = 7,
} GyroFS_t;

/* --- 采样频率 ODR 枚举 --- */
typedef enum {
    ODR_32kHz  = 0x01,
    ODR_16kHz  = 0x02,
    ODR_8kHz   = 0x03,
    ODR_4kHz   = 0x04,
    ODR_2kHz   = 0x05,
    ODR_1kHz   = 0x06,
    ODR_200Hz  = 0x07,
    ODR_100Hz  = 0x08,
    ODR_50Hz   = 0x09,
    ODR_25Hz   = 0x0A,
} ODR_t;
/* UI 滤波器阶数 (Filter Order) */
typedef enum {
    UI_FILT_ORD_1ST = 0,
    UI_FILT_ORD_2ND = 1,
    UI_FILT_ORD_3RD = 2,
} UIFiltOrd_t;

/* UI 滤波器带宽 (Bandwidth) - 仅列出常用项，详见手册 5.5 */
typedef enum {
    UI_FILT_BW_ODR_DIV_2  = 0, // BW = ODR/2
    UI_FILT_BW_ODR_DIV_4  = 1, // BW = ODR/4 (Default)
    UI_FILT_BW_ODR_DIV_5  = 2,
    UI_FILT_BW_ODR_DIV_8  = 3,
    UI_FILT_BW_ODR_DIV_10 = 4,
    UI_FILT_BW_ODR_DIV_16 = 5,
    UI_FILT_BW_LL_MAX_200 = 15 // Low Latency
} UIFiltBW_t;

/* 陀螺仪陷波滤波器带宽 (Notch Filter Bandwidth) */
typedef enum {
    GYRO_NF_BW_1449HZ = 0, // 3dB Bandwidth
    GYRO_NF_BW_680HZ  = 1,
    GYRO_NF_BW_329HZ  = 2,
    GYRO_NF_BW_162HZ  = 3,
    GYRO_NF_BW_80HZ   = 4,
    GYRO_NF_BW_40HZ   = 5,
    GYRO_NF_BW_20HZ   = 6,
    GYRO_NF_BW_10HZ   = 7,
} GyroNFBW_t;

/* ==============================================================================
 * 新增 API 函数声明
 * ============================================================================== */
/**
 * @brief 配置 UI 路径的滤波器 (LPF)
 * @param a_ord 加速度计滤波器阶数
 * @param a_bw  加速度计滤波器带宽
 * @param g_ord 陀螺仪滤波器阶数
 * @param g_bw  陀螺仪滤波器带宽
 */
void ICM42688_Config_UI_Filter(UIFiltOrd_t a_ord, UIFiltBW_t a_bw, UIFiltOrd_t g_ord, UIFiltBW_t g_bw);

/**
 * @brief 配置陀螺仪陷波滤波器 (Notch Filter) - 对 X,Y,Z 三轴统一配置
 * @param enable 1:开启, 0:关闭
 * @param center_freq_hz 中心频率 (Hz), 范围 1000Hz - 3000Hz
 * @param bw_sel 带宽选择 (枚举)
 */
void ICM42688_Config_Gyro_Notch_Filter(uint8_t enable, float center_freq_hz, GyroNFBW_t bw_sel);

/**
 * @brief 配置抗混叠滤波器 (AAF) - 使用手册参数
 * @note  由于 AAF 参数查表非常复杂，此处直接传入寄存器参数。
 * 请参考 Datasheet 表格计算 delt, deltSqr, bitshift。
 * 如果全部传 0，则禁用 AAF。
 */
void ICM42688_Config_AAF(uint8_t is_accel, uint8_t delt, uint16_t deltSqr, uint8_t bitshift);
// 全局分辨率变量，供数据处理函数使用
extern float acc_res;
extern float gyr_res;
/* API 函数 */
uint8_t ICM42688_Init(void);
uint8_t ICM42688_IsDataReady(void);
void ICM42688_Config_FIFO(uint8_t enable);
void ICM42688_SetFormat(ODR_t a_odr, AccelFS_t a_fsr, ODR_t g_odr, GyroFS_t g_fsr);
void ICM42688_read(float gyro[3], float accel[3], float *temperature);
#endif //FDCAN_TEST_G4_BSP_ICM42688P_H