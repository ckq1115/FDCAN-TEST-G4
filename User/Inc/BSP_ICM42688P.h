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

// BANK 1
#define REG_GYRO_CONFIG_STATIC2 0x010B

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