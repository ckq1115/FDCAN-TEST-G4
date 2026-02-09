#ifndef BSP_ICM42688P_H
#define BSP_ICM42688P_H

#include "main.h" // 包含 HAL 库定义
#include <stdint.h>
#include <math.h>

/* =================================================================================
 * 寄存器定义 (格式: (Bank << 8) | Address)
 * ================================================================================= */
// BANK 0
#define REG_DEVICE_CONFIG      0x0011
#define REG_DRIVE_CONFIG       0x0013
#define REG_INT_CONFIG         0x0014
#define REG_FIFO_CONFIG        0x0016
#define REG_INTF_CONFIG1       0x004D  // [Fixed] Added missing register definition
#define REG_PWR_MGMT0          0x004E
#define REG_GYRO_CONFIG0       0x004F
#define REG_ACCEL_CONFIG0      0x0050
#define REG_GYRO_CONFIG1       0x0051
#define REG_GYRO_ACCEL_CONFIG0 0x0052
#define REG_ACCEL_CONFIG1      0x0053
#define REG_TMST_CONFIG        0x0054
#define REG_APEX_CONFIG0       0x0056
#define REG_SMD_CONFIG         0x0057
#define REG_FIFO_CONFIG1       0x005F
#define REG_FIFO_CONFIG2       0x0060
#define REG_FIFO_CONFIG3       0x0061
#define REG_FSYNC_CONFIG       0x0062
#define REG_INT_CONFIG0        0x0063
#define REG_INT_CONFIG1        0x0064
#define REG_INT_SOURCE0        0x0065
#define REG_INT_SOURCE1        0x0066
#define REG_INT_SOURCE3        0x0068
#define REG_INT_SOURCE4        0x0069
#define REG_FIFO_LOST_PKT0     0x006C
#define REG_INT_SOURCE6        0x006D
#define REG_INT_SOURCE7        0x006E
#define REG_INT_SOURCE8        0x006F
#define REG_INT_SOURCE9        0x0070
#define REG_SELF_TEST_CONFIG   0x0070
#define REG_WHO_AM_I           0x0075
#define REG_BANK_SEL           0x0076

// BANK 0 - Data Registers
#define REG_TEMP_DATA1         0x001D
#define REG_ACCEL_DATA_X1      0x001F
#define REG_GYRO_DATA_X1       0x0025
#define REG_TMST_FSYNCH        0x002B
#define REG_INT_STATUS         0x002D
#define REG_FIFO_COUNTH        0x002E
#define REG_FIFO_DATA          0x0030
#define REG_APEX_DATA0         0x0031
#define REG_APEX_DATA1         0x0032
#define REG_APEX_DATA2         0x0033
#define REG_APEX_DATA3         0x0034
#define REG_APEX_DATA4         0x0035
#define REG_APEX_DATA5         0x0036
#define REG_INT_STATUS2        0x0037
#define REG_INT_STATUS3        0x0038

// BANK 1
#define REG_SENSOR_CONFIG0      0x0103
#define REG_GYRO_CONFIG_STATIC2 0x010B
#define REG_GYRO_CONFIG_STATIC3 0x010C
#define REG_GYRO_CONFIG_STATIC4 0x010D
#define REG_GYRO_CONFIG_STATIC5 0x010E
#define REG_GYRO_CONFIG_STATIC6 0x010F
#define REG_GYRO_CONFIG_STATIC7 0x0110
#define REG_GYRO_CONFIG_STATIC8 0x0111
#define REG_GYRO_CONFIG_STATIC9 0x0112
#define REG_GYRO_CONFIG_STATIC10 0x0113
#define REG_XG_ST_DATA          0x015F
#define REG_YG_ST_DATA          0x0160
#define REG_ZG_ST_DATA          0x0161
#define REG_TMSTVAL0            0x0162
#define REG_INTF_CONFIG4        0x017A
#define REG_INTF_CONFIG5        0x017B
#define REG_INTF_CONFIG6        0x017C

// BANK 2
#define REG_ACCEL_CONFIG_STATIC2 0x0203
#define REG_ACCEL_CONFIG_STATIC3 0x0204
#define REG_ACCEL_CONFIG_STATIC4 0x0205
#define REG_XA_ST_DATA           0x023B
#define REG_YA_ST_DATA           0x023C
#define REG_ZA_ST_DATA           0x023D

// BANK 4
#define REG_APEX_CONFIG1        0x0440
#define REG_APEX_CONFIG2        0x0441
#define REG_APEX_CONFIG3        0x0442
#define REG_APEX_CONFIG4        0x0443
#define REG_APEX_CONFIG5        0x0444
#define REG_APEX_CONFIG6        0x0445
#define REG_APEX_CONFIG7        0x0446
#define REG_APEX_CONFIG8        0x0447
#define REG_APEX_CONFIG9        0x0448
#define REG_ACCEL_WOM_X_THR     0x044A
#define REG_ACCEL_WOM_Y_THR     0x044B
#define REG_ACCEL_WOM_Z_THR     0x044C
#define REG_OFFSET_USER0        0x0477
#define REG_OFFSET_USER1        0x0478
#define REG_OFFSET_USER2        0x0479
#define REG_OFFSET_USER3        0x047A
#define REG_OFFSET_USER4        0x047B
#define REG_OFFSET_USER5        0x047C
#define REG_OFFSET_USER6        0x047D
#define REG_OFFSET_USER7        0x047E
#define REG_OFFSET_USER8        0x047F

/* =================================================================================
 * 枚举与配置定义
 * ================================================================================= */

typedef enum {
    ICM_ODR_32KHZ = 1,
    ICM_ODR_16KHZ = 2,
    ICM_ODR_8KHZ  = 3,
    ICM_ODR_4KHZ  = 4,
    ICM_ODR_2KHZ  = 5,
    ICM_ODR_1KHZ  = 6, // Default
    ICM_ODR_200HZ = 7,
    ICM_ODR_100HZ = 8,
    ICM_ODR_50HZ  = 9,
    ICM_ODR_25HZ  = 10,
    ICM_ODR_12_5HZ = 11,
    ICM_ODR_500HZ = 15
} ICM_ODR_t;

typedef enum {
    ICM_ACCEL_16G = 0,
    ICM_ACCEL_8G  = 1,
    ICM_ACCEL_4G  = 2,
    ICM_ACCEL_2G  = 3
} ICM_AccelFS_t;

typedef enum {
    ICM_GYRO_2000DPS = 0,
    ICM_GYRO_1000DPS = 1,
    ICM_GYRO_500DPS  = 2,
    ICM_GYRO_250DPS  = 3,
    ICM_GYRO_125DPS  = 4,
    ICM_GYRO_62_5DPS = 5,
    ICM_GYRO_31_25DPS = 6,
    ICM_GYRO_15_625DPS = 7
} ICM_GyroFS_t;

typedef enum {
    ICM_MODE_SLEEP      = 0,
    ICM_MODE_STANDBY    = 1, // Only Gyro
    ICM_MODE_ACCEL_LP   = 2,
    ICM_MODE_ACCEL_LN   = 3,
    ICM_MODE_GYRO_LN    = 3, // Logic handled in driver
    ICM_MODE_6AXIS_LN   = 9  // Custom logic
} ICM_PowerMode_t;

typedef enum {
    ICM_INT1 = 0,
    ICM_INT2 = 1
} ICM_IntPin_t;

/* 滤波器带宽选择 (UI_FILT_BW) */
typedef enum {
    ICM_BW_ODR_DIV_2  = 0,
    ICM_BW_ODR_DIV_4  = 1, // Default
    ICM_BW_ODR_DIV_5  = 2,
    ICM_BW_ODR_DIV_8  = 3,
    ICM_BW_ODR_DIV_10 = 4,
    ICM_BW_ODR_DIV_16 = 5,
    ICM_BW_ODR_DIV_20 = 6,
    ICM_BW_ODR_DIV_40 = 7,
    ICM_BW_LL_MAX_400 = 14, // Low Latency
    ICM_BW_LL_MAX_200 = 15
} ICM_FilterBW_t;

/* APEX 功能掩码 */
typedef enum {
    ICM_APEX_PEDOMETER = 0x20, // 计步器
    ICM_APEX_TILT      = 0x10, // 倾斜检测
    ICM_APEX_R2W       = 0x08, // 抬手唤醒 (Raise to Wake)
    ICM_APEX_TAP       = 0x40, // 轻敲检测
    ICM_APEX_SMD       = 0xFF  // 特殊标记位，用于逻辑判断
} ICM_ApexFeature_t;

/* 设备对象句柄 */
typedef struct {
    SPI_HandleTypeDef *spiHandle;
    GPIO_TypeDef      *csPort;
    uint16_t          csPin;
    float             accelScale; // g -> m/s^2
    float             gyroScale;  // dps -> rad/s
    uint8_t           currentBank;
} ICM42688_Device_t;

/* 测量数据结构 */
typedef struct {
    float accel[3]; // m/s^2
    float gyro[3];  // rad/s
    float temp;     // degC
    uint32_t timestamp;
} ICM_Data_t;

typedef struct {
    uint16_t stepCount;
    uint8_t  stepCadence;
    char* activity; // "Walk", "Run", "Unknown"
} ICM_PedometerData_t;

// AAF 配置参数结构体
typedef struct {
    uint16_t freq_hz;    // 截止频率
    uint8_t  delt;       // DELT 值
    uint16_t deltsqr;    // DELTSQR 值
    uint8_t  bitshift;   // BITSHIFT 值
} ICM_AAF_Config_t;

// 根据手册 Table 18 & 19 整理的常用频率表 (以陀螺仪为例)
static const ICM_AAF_Config_t AAF_Table[] = {
    {10,  3,   9,    15},
    {21,  6,   36,   13},
    {42,  12,  144,  11},
    {83,  24,  576,  9},
    {167, 48,  2304, 7},
    {250, 72,  5184, 6},
    {500, 144, 20736, 4},
};
#define AAF_TABLE_SIZE (sizeof(AAF_Table)/sizeof(AAF_Table[0]))

/* =================================================================================
 * 函数声明
 * ================================================================================= */

/* 初始化与基础配置 */
uint8_t ICM42688_Init(ICM42688_Device_t *dev, SPI_HandleTypeDef *spi, GPIO_TypeDef *csPort, uint16_t csPin);
void ICM42688_Reset(ICM42688_Device_t *dev);
uint8_t ICM42688_CheckID(ICM42688_Device_t *dev);
void ICM42688_SetPowerMode(ICM42688_Device_t *dev, uint8_t accelMode, uint8_t gyroMode);
void ICM42688_SetSensorConfig(ICM42688_Device_t *dev, ICM_ODR_t odr, ICM_AccelFS_t accelFs, ICM_GyroFS_t gyroFs);

/* 数据读取 */
void ICM42688_ReadData(ICM42688_Device_t *dev, ICM_Data_t *data);
void ICM42688_ReadFIFO(ICM42688_Device_t *dev, uint8_t *buffer, uint16_t len);
uint16_t ICM42688_GetFIFOCount(ICM42688_Device_t *dev);

/* 信号路径与滤波 (Section 5) */
void ICM42688_SetNotchFilter(ICM42688_Device_t *dev, float centerFreqHz, uint8_t bwCode);
void ICM42688_SetAntiAliasFilter(ICM42688_Device_t *dev, uint8_t sensorType, uint16_t freqHz); // 0=Gyro, 1=Accel
void ICM42688_SetUIFilter(ICM42688_Device_t *dev, ICM_FilterBW_t bw, uint8_t order);

/* 中断配置 */
void ICM42688_ConfigIntPin(ICM42688_Device_t *dev, ICM_IntPin_t pin, uint8_t activeHigh, uint8_t pushPull, uint8_t latch);
void ICM42688_EnableIntSource(ICM42688_Device_t *dev, ICM_IntPin_t pin, uint8_t sourceMask); // Use defines in C file

/* APEX / DMP 功能 (Section 8) */
void ICM42688_ApexEnable(ICM42688_Device_t *dev, ICM_ApexFeature_t feature, uint8_t enable);
void ICM42688_ConfigPedometer(ICM42688_Device_t *dev);
void ICM42688_ConfigTap(ICM42688_Device_t *dev);
void ICM42688_ConfigTilt(ICM42688_Device_t *dev, uint8_t waitTime);
void ICM42688_ConfigWOM(ICM42688_Device_t *dev, uint8_t threshold_mg, uint8_t x_en, uint8_t y_en, uint8_t z_en);
void ICM42688_GetPedometerData(ICM42688_Device_t *dev, ICM_PedometerData_t *data);
uint8_t ICM42688_GetTapData(ICM42688_Device_t *dev, uint8_t *axis, uint8_t *direction);

/* 自检与校准 */
uint8_t ICM42688_RunSelfTest(ICM42688_Device_t *dev);
void ICM42688_SetOffsets(ICM42688_Device_t *dev, int16_t accBias[3], int16_t gyroBias[3]);

#endif