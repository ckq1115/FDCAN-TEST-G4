//
// Created by CaoKangqi on 2026/1/23.
//
#ifndef G4_FRAMEWORK_ICM42688P_H
#define G4_FRAMEWORK_ICM42688P_H

#include "BSP_SPI.h"

/* Register address (bank in high byte, addr in low byte) */
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
#define REG_INT_CONFIG1          0x0064  // 新增：异步复位配置寄存器
#define REG_INT_ENABLE0          0x0062  // 修正：真正的中断使能寄存器（DRDY等）
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

// constants
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

/* Accel full-scale range */
typedef enum {
    ACCEL_FS_16G = 0,
    ACCEL_FS_8G  = 1,
    ACCEL_FS_4G  = 2,
    ACCEL_FS_2G  = 3,
} AccelFS_t;

/* Gyro full-scale range */
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

/* Output data rate */
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

/* UI filter order */
typedef enum {
    UI_FILT_ORD_1ST = 0,
    UI_FILT_ORD_2ND = 1,
    UI_FILT_ORD_3RD = 2,
} UIFiltOrd_t;

/* UI filter bandwidth */
typedef enum {
    UI_FILT_BW_ODR_DIV_2  = 0,
    UI_FILT_BW_ODR_DIV_4  = 1,
    UI_FILT_BW_ODR_DIV_5  = 2,
    UI_FILT_BW_ODR_DIV_8  = 3,
    UI_FILT_BW_ODR_DIV_10 = 4,
    UI_FILT_BW_ODR_DIV_16 = 5,
    UI_FILT_BW_LL_MAX_200 = 15
} UIFiltBW_t;

/* Gyro notch filter bandwidth */
typedef enum {
    GYRO_NF_BW_1449HZ = 0,
    GYRO_NF_BW_680HZ  = 1,
    GYRO_NF_BW_329HZ  = 2,
    GYRO_NF_BW_162HZ  = 3,
    GYRO_NF_BW_80HZ   = 4,
    GYRO_NF_BW_40HZ   = 5,
    GYRO_NF_BW_20HZ   = 6,
    GYRO_NF_BW_10HZ   = 7,
} GyroNFBW_t;

#define ICM_DMA_FRAME_LEN  15

void ICM42688_Config_UI_Filter(UIFiltOrd_t a_ord, UIFiltBW_t a_bw, UIFiltOrd_t g_ord, UIFiltBW_t g_bw);
void ICM42688_Config_Gyro_Notch_Filter(uint8_t enable, float center_freq_hz, GyroNFBW_t bw_sel);
void ICM42688_Config_AAF(uint8_t is_accel, uint8_t delt, uint16_t deltSqr, uint8_t bitshift);

extern float acc_res;
extern float gyr_res;

uint8_t ICM42688_Init(void);
uint8_t ICM42688_IsDataReady(void);
void ICM42688_Config_FIFO(uint8_t enable);
void ICM42688_SetFormat(ODR_t a_odr, AccelFS_t a_fsr, ODR_t g_odr, GyroFS_t g_fsr);
void ICM42688_read(float gyro[3], float accel[3], float *temperature);
void ICM42688_Read_Fast(float gyro[3], float accel[3], float *temperature);
void ICM42688_ResolveRaw(const uint8_t raw_data[14], float gyro[3], float accel[3], float *temperature);

void ICM42688_StartRead_IntDMA(uint8_t tx_buf[ICM_DMA_FRAME_LEN], uint8_t rx_buf[ICM_DMA_FRAME_LEN]);
uint8_t ICM42688_Read_IntDMA(float gyro[3], float accel[3], float *temperature);
void ICM42688_OnExti(void);
void ICM42688_OnSpiDmaCplt(SPI_HandleTypeDef *hspi);
void ICM42688_OnSpiError(SPI_HandleTypeDef *hspi);

#endif // G4_FRAMEWORK_ICM42688P_H

