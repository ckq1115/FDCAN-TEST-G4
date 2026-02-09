#include "BSP_ICM42688P.h"
#include <string.h> // for memset

/* 内部常量与宏 */
#define PI 3.14159265358979f
#define WHO_AM_I_VAL 0x47

/* 私有函数声明 */
static void ICM_SetBank(ICM42688_Device_t *dev, uint8_t bank);
static void ICM_WriteReg(ICM42688_Device_t *dev, uint16_t reg, uint8_t val);
static uint8_t ICM_ReadReg(ICM42688_Device_t *dev, uint16_t reg);
static void ICM_ReadRegs(ICM42688_Device_t *dev, uint16_t reg, uint8_t *buf, uint16_t len);

/* =================================================================================
 * 基础驱动实现
 * ================================================================================= */

/**
 * @brief  检查设备 ID 是否正确
 * @param  dev: 设备句柄指针
 * @return 0: 成功 (ID匹配), 1: 失败 (ID不匹配)
 */
uint8_t ICM42688_CheckID(ICM42688_Device_t *dev) {
    uint8_t who = ICM_ReadReg(dev, REG_WHO_AM_I);
    return (who == WHO_AM_I_VAL) ? 0 : 1;
}

/**
 * @brief  初始化 ICM42688
 */
uint8_t ICM42688_Init(ICM42688_Device_t *dev, SPI_HandleTypeDef *spi, GPIO_TypeDef *csPort, uint16_t csPin) {
    dev->spiHandle = spi;
    dev->csPort = csPort;
    dev->csPin = csPin;
    dev->currentBank = 0xFF; // Force update bank logic

    // 1. 软复位 (Section 14.1)
    ICM_WriteReg(dev, REG_DEVICE_CONFIG, 0x01);
    HAL_Delay(5); // Wait 1ms is required, 5ms is safe

    // 2. 身份验证
    if (ICM42688_CheckID(dev) != 0) return 1;

    // 3. 配置接口与时钟 (UI interface settings)
    // 推荐使用 Auto Clock Selection (PLL or RC)
    ICM_WriteReg(dev, REG_INTF_CONFIG1, 0x01); // ring_osc_clk_sel=0, acc_lp_clk_sel=0, rtc=0, clksel=1(PLL)

    // 4. 配置 ODR 和量程
    // 姿态解算建议 ODR >= 1kHz, 加速度 ±16g, 陀螺仪 ±2000dps
    ICM42688_SetSensorConfig(dev, ICM_ODR_1KHZ, ICM_ACCEL_16G, ICM_GYRO_2000DPS);

    // 5. 开启传感器 (进入 Low Noise 模式)
    // Section 10.2: 先配置再开启
    ICM42688_SetPowerMode(dev, 3, 3); // 3 = Low Noise Mode for both

    // 6. 陀螺仪稳定时间 (Spec 4.2: Gyro startup time 45ms)
    HAL_Delay(50);

    return 0;
}

/**
 * @brief  设置传感器配置 (ODR, FSR)
 * Reference: Section 14.37, 14.38
 */
void ICM42688_SetSensorConfig(ICM42688_Device_t *dev, ICM_ODR_t odr, ICM_AccelFS_t accelFs, ICM_GyroFS_t gyroFs) {
    // Config Accel (FS_SEL in 7:5, ODR in 3:0)
    ICM_WriteReg(dev, REG_ACCEL_CONFIG0, (accelFs << 5) | odr);

    // Config Gyro (FS_SEL in 7:5, ODR in 3:0)
    ICM_WriteReg(dev, REG_GYRO_CONFIG0, (gyroFs << 5) | odr);

    // 更新比例因子用于转换
    // Accel: 16g / 32768
    float accelRangeG = 16.0f / (float)(1 << accelFs);
    dev->accelScale = (accelRangeG * 9.80665f) / 32768.0f; // 转换为 m/s^2

    // Gyro: 2000dps / 32768
    float gyroRangeDps = 2000.0f / (float)(1 << gyroFs);
    dev->gyroScale = (gyroRangeDps * PI / 180.0f) / 32768.0f; // 转换为 rad/s
}

/**
 * @brief  设置电源模式
 * Reference: Section 4.16 & 14.36
 * Mode: 0=OFF, 1=Standby(Gyro only), 2=LowPower(Accel only), 3=LowNoise
 */
void ICM42688_SetPowerMode(ICM42688_Device_t *dev, uint8_t accelMode, uint8_t gyroMode) {
    // bit 3:2 Gyro, bit 1:0 Accel. bit 5 Temp Disable (0=enabled)
    uint8_t regVal = (gyroMode << 2) | accelMode;

    // Ensure IDLE bit is managed if needed, usually 0 is fine for Auto
    ICM_WriteReg(dev, REG_PWR_MGMT0, regVal);

    // Spec says wait 200us after mode transition
    HAL_Delay(1);
}

/* =================================================================================
 * 数据读取实现
 * ================================================================================= */

/**
 * @brief  读取传感器数据 (Accel, Gyro, Temp)
 * Reference: Section 14.5 - 14.18
 */
void ICM42688_ReadData(ICM42688_Device_t *dev, ICM_Data_t *data) {
    uint8_t buffer[14]; // 2 Temp + 6 Accel + 6 Gyro

    // Burst read starting from TEMP_DATA1 (0x1D)
    // Order: Temp(2), Accel X/Y/Z (6), Gyro X/Y/Z (6)
    ICM_ReadRegs(dev, REG_TEMP_DATA1, buffer, 14);

    int16_t rawTemp = (int16_t)((buffer[0] << 8) | buffer[1]);
    int16_t rawAcc[3], rawGyro[3];

    rawAcc[0] = (int16_t)((buffer[2] << 8) | buffer[3]);
    rawAcc[1] = (int16_t)((buffer[4] << 8) | buffer[5]);
    rawAcc[2] = (int16_t)((buffer[6] << 8) | buffer[7]);

    rawGyro[0] = (int16_t)((buffer[8] << 8) | buffer[9]);
    rawGyro[1] = (int16_t)((buffer[10] << 8) | buffer[11]);
    rawGyro[2] = (int16_t)((buffer[12] << 8) | buffer[13]);

    // Apply scales
    data->accel[0] = rawAcc[0] * dev->accelScale;
    data->accel[1] = rawAcc[1] * dev->accelScale;
    data->accel[2] = rawAcc[2] * dev->accelScale;

    data->gyro[0] = rawGyro[0] * dev->gyroScale;
    data->gyro[1] = rawGyro[1] * dev->gyroScale;
    data->gyro[2] = rawGyro[2] * dev->gyroScale;

    // Temp formula: (TEMP_DATA / 132.48) + 25 (Section 4.13)
    data->temp = ((float)rawTemp / 132.48f) + 25.0f;
}

/* =================================================================================
 * 滤波器配置 (Section 5)
 * ================================================================================= */

/**
 * @brief  配置陀螺仪陷波滤波器 (Notch Filter)
 * Reference: Section 5.1 & 5.2
 * param centerFreqHz: 陷波中心频率 (Hz) [1000-3000]
 * param bwCode: 带宽代码 (0-7, see datasheet)
 */
void ICM42688_SetNotchFilter(ICM42688_Device_t *dev, float centerFreqHz, uint8_t bwCode) {
    if (centerFreqHz < 1000.0f || centerFreqHz > 3000.0f) return;

    // Calculation step 1: coswz = cos(2*pi*f / 32kHz)
    float coswz = cosf(2.0f * PI * centerFreqHz / 32000.0f);

    int16_t nf_coswz;
    uint8_t nf_coswz_sel;

    // Step 2: Alg from datasheet Section 5.2
    if (fabsf(coswz) <= 0.875f) {
        nf_coswz = (int16_t)roundf(coswz * 256.0f);
        nf_coswz_sel = 0;
    } else {
        nf_coswz_sel = 1;
        if (coswz > 0.875f) {
            nf_coswz = (int16_t)roundf(8.0f * (1.0f - coswz) * 256.0f);
        } else {
            nf_coswz = (int16_t)roundf(-8.0f * (1.0f + coswz) * 256.0f);
        }
    }

    // Program Registers (Bank 1)
    // Apply to all 3 axes for simplicity

    // 1. Low 8 bits of parameters
    ICM_WriteReg(dev, REG_GYRO_CONFIG_STATIC6, nf_coswz & 0xFF); // X
    ICM_WriteReg(dev, REG_GYRO_CONFIG_STATIC7, nf_coswz & 0xFF); // Y
    ICM_WriteReg(dev, REG_GYRO_CONFIG_STATIC8, nf_coswz & 0xFF); // Z

    // 2. High bit and SEL (Register 0x12)
    // Read-Modify-Write to ensure reserved bits are safe (though currently bits 7:6 are reserved)
    uint8_t static9 = 0;

    // Set SEL bits (3,4,5) for X, Y, Z
    if (nf_coswz_sel) static9 |= (0x07 << 3);

    // Set High bits (0,1,2) for X, Y, Z (bit 8 of nf_coswz)
    if (nf_coswz & 0x100) static9 |= 0x07;

    ICM_WriteReg(dev, REG_GYRO_CONFIG_STATIC9, static9);

    // 3. Set Bandwidth (Register 0x13)
    ICM_WriteReg(dev, REG_GYRO_CONFIG_STATIC10, (bwCode << 4));

    // 4. Enable Notch Filter (Register 0x0B bit 0 = 0 to enable)
    uint8_t regVal = ICM_ReadReg(dev, REG_GYRO_CONFIG_STATIC2);
    regVal &= ~0x01;
    ICM_WriteReg(dev, REG_GYRO_CONFIG_STATIC2, regVal);
}

/**
 * @brief 配置抗混叠滤波器 (AAF)
 * @param sensorType: 0-Gyro, 1-Accel
 * @param freqHz: 截止频率 (10, 21, 42, ... 500)
 */
void ICM42688_SetAntiAliasFilter(ICM42688_Device_t *dev, uint8_t sensorType, uint16_t freqHz) {
    // 禁用 AAF
    if (freqHz == 0) {
        if (sensorType == 0) {
            // Gyro Bank 1, Reg 0x0B, Bit 1 = 1 to Disable
            uint8_t reg = ICM_ReadReg(dev, REG_GYRO_CONFIG_STATIC2);
            ICM_WriteReg(dev, REG_GYRO_CONFIG_STATIC2, reg | 0x02);
        }
        else {
            // Accel Bank 2, Reg 0x03, Bit 0 = 1 to Disable
            uint8_t reg = ICM_ReadReg(dev, REG_ACCEL_CONFIG_STATIC2);
            ICM_WriteReg(dev, REG_ACCEL_CONFIG_STATIC2, reg | 0x01);
        }
        return;
    }

    // 查找配置表
    const ICM_AAF_Config_t *cfg = &AAF_Table[0];
    for (int i = 0; i < AAF_TABLE_SIZE; i++) {
        cfg = &AAF_Table[i];
        if (AAF_Table[i].freq_hz >= freqHz) break;
    }

    if (sensorType == 0) {
        /* ------------------- 陀螺仪 (Bank 1) ------------------- */
        // REG_GYRO_CONFIG_STATIC2 (0x0B): Bit[1]=0 Enable, Bit[0] is Notch
        uint8_t static2 = ICM_ReadReg(dev, REG_GYRO_CONFIG_STATIC2);
        static2 &= ~0x02; // Enable AAF
        ICM_WriteReg(dev, REG_GYRO_CONFIG_STATIC2, static2);

        // REG_GYRO_CONFIG_STATIC3 (0x0C): Bits[5:0] = GYRO_AAF_DELT
        ICM_WriteReg(dev, REG_GYRO_CONFIG_STATIC3, cfg->delt & 0x3F);

        // REG_GYRO_CONFIG_STATIC4 (0x0D): Bits[7:0] = GYRO_AAF_DELTSQR[7:0]
        ICM_WriteReg(dev, REG_GYRO_CONFIG_STATIC4, (uint8_t)(cfg->deltsqr & 0xFF));

        // REG_GYRO_CONFIG_STATIC5 (0x0E): Bits[7:4] = BITSHIFT, Bits[3:0] = DELTSQR[11:8]
        uint8_t static5 = (cfg->bitshift << 4) | ((cfg->deltsqr >> 8) & 0x0F);
        ICM_WriteReg(dev, REG_GYRO_CONFIG_STATIC5, static5);
    }
    else {
        /* ------------------- 加速度计 (Bank 2) ------------------- */
        // 修正了原代码中的寄存器映射错误 (See Datasheet Section 16)

        // REG_ACCEL_CONFIG_STATIC2 (0x03): Bits[6:1] = ACCEL_AAF_DELT, Bit[0] = 0 (Enable)
        uint8_t static2 = (cfg->delt << 1) & 0xFE;
        ICM_WriteReg(dev, REG_ACCEL_CONFIG_STATIC2, static2);

        // REG_ACCEL_CONFIG_STATIC3 (0x04): Bits[7:0] = ACCEL_AAF_DELTSQR[7:0]
        ICM_WriteReg(dev, REG_ACCEL_CONFIG_STATIC3, (uint8_t)(cfg->deltsqr & 0xFF));

        // REG_ACCEL_CONFIG_STATIC4 (0x05): Bits[7:4] = BITSHIFT, Bits[3:0] = ACCEL_AAF_DELTSQR[11:8]
        uint8_t static4 = (cfg->bitshift << 4) | ((cfg->deltsqr >> 8) & 0x0F);
        ICM_WriteReg(dev, REG_ACCEL_CONFIG_STATIC4, static4);
    }
}

/**
 * @brief 设置 UI 滤波器 (低通滤波)
 * @param bw: 带宽选择 (例如 ICM_BW_ODR_DIV_4)
 * @param order: 滤波器阶数 (1: 1st order, 2: 2nd order, 3: 3rd order)
 */
void ICM42688_SetUIFilter(ICM42688_Device_t *dev, ICM_FilterBW_t bw, uint8_t order) {
    // 1. 设置带宽 (Bank 0)
    // GYRO_ACCEL_CONFIG0: [7:4] Accel BW, [3:0] Gyro BW
    ICM_WriteReg(dev, REG_GYRO_ACCEL_CONFIG0, (bw << 4) | bw);

    // 2. 设置滤波器阶数 (Bank 0)
    // Accel: ACCEL_CONFIG1 [4:3]
    uint8_t a_cfg1 = ICM_ReadReg(dev, REG_ACCEL_CONFIG1) & ~0x18;
    ICM_WriteReg(dev, REG_ACCEL_CONFIG1, a_cfg1 | (order << 3));

    // Gyro: GYRO_CONFIG1 [3:2]
    uint8_t g_cfg1 = ICM_ReadReg(dev, REG_GYRO_CONFIG1) & ~0x0C;
    ICM_WriteReg(dev, REG_GYRO_CONFIG1, g_cfg1 | (order << 2));
}

/* =================================================================================
 * APEX / DMP 功能 (Section 8)
 * ================================================================================= */

/**
 * @brief  通用 APEX 使能/禁用序列
 */
void ICM42688_ApexEnable(ICM42688_Device_t *dev, ICM_ApexFeature_t feature, uint8_t enable) {
    // 1. 处理 APEX_CONFIG0 控制的功能 (Pedometer, Tilt, Tap, R2W)
    if (feature != ICM_APEX_SMD) {
        uint8_t config0 = ICM_ReadReg(dev, REG_APEX_CONFIG0);
        if (enable) config0 |= feature;
        else config0 &= ~feature;
        ICM_WriteReg(dev, REG_APEX_CONFIG0, config0);
    }
    // 2. 处理 SMD (显著运动检测)
    else {
        uint8_t smdCfg = ICM_ReadReg(dev, REG_SMD_CONFIG);
        if (enable) {
            smdCfg |= 0x02; // 设置为 Short Window 模式
        } else {
            smdCfg &= ~0x03; // 禁用 SMD
        }
        ICM_WriteReg(dev, REG_SMD_CONFIG, smdCfg);
    }
}

/**
 * @brief  配置并开启计步器
 * Reference: Section 8.3
 */
void ICM42688_ConfigPedometer(ICM42688_Device_t *dev) {
    // 1. Reset DMP Memory (Bank 0)
    ICM_WriteReg(dev, 0x004B, 0x20); // DMP_MEM_RESET_EN
    HAL_Delay(1);

    // 2. Config Parameters (Bank 4) - Using Defaults from datasheet
    ICM_WriteReg(dev, REG_APEX_CONFIG1, 0xA0); // Low Energy Amp TH
    ICM_WriteReg(dev, REG_APEX_CONFIG2, 0x85); // Amp TH, Step Count TH (5 steps)
    ICM_WriteReg(dev, REG_APEX_CONFIG3, 0x51); // Step Det TH, Timer TH, Hi En TH

    // 3. Enable DMP (Bank 0)
    ICM_WriteReg(dev, 0x004B, 0x40); // DMP_INIT_EN
    HAL_Delay(50);

    // 4. Enable Feature
    ICM42688_ApexEnable(dev, ICM_APEX_PEDOMETER, 1);
}

/**
 * @brief  配置运动唤醒 (WOM)
 * Reference: Section 8.7
 */
void ICM42688_ConfigWOM(ICM42688_Device_t *dev, uint8_t threshold_mg, uint8_t x_en, uint8_t y_en, uint8_t z_en) {
    // 1. 设置阈值 (Bank 4) - Resolution approx 3.9mg (1g/256)
    // thr = mg / 3.9 => mg * 256 / 1000 => mg * 32 / 125
    uint8_t thr = (uint8_t)(threshold_mg * 256 / 1000);

    ICM_WriteReg(dev, REG_ACCEL_WOM_X_THR, thr);
    ICM_WriteReg(dev, REG_ACCEL_WOM_Y_THR, thr);
    ICM_WriteReg(dev, REG_ACCEL_WOM_Z_THR, thr);

    // 2. 配置 WOM 工作模式 (Bank 0)
    // WOM_INT_MODE = 0 (OR), WOM_MODE = 0 (Compare with Initial)
    ICM_WriteReg(dev, REG_SMD_CONFIG, 0x00);

    // 3. 核心步骤：使能 WOM 中断源 (Section 14.52)
    // REG_INT_SOURCE1 (0x66) Bits 2:0 for Z, Y, X
    uint8_t intSource = 0;
    if (x_en) intSource |= 0x01; // WOM_X_INT_EN
    if (y_en) intSource |= 0x02; // WOM_Y_INT_EN
    if (z_en) intSource |= 0x04; // WOM_Z_INT_EN

    uint8_t curr = ICM_ReadReg(dev, REG_INT_SOURCE1);
    ICM_WriteReg(dev, REG_INT_SOURCE1, curr | intSource);
}

void ICM42688_GetPedometerData(ICM42688_Device_t *dev, ICM_PedometerData_t *data) {
    uint8_t buf[4]; // STEP_CNT(2), CADENCE(1), ACTIVITY(1)

    // REG_APEX_DATA0 (0x31) -> STEP_CNT[7:0]
    // REG_APEX_DATA1 (0x32) -> STEP_CNT[15:8]
    ICM_ReadRegs(dev, REG_APEX_DATA0, buf, 4);

    data->stepCount = (uint16_t)((buf[1] << 8) | buf[0]); // Little Endian reconstruct? No, check datasheet 14.25/14.26
    // Reg 31 is Lower byte, Reg 32 is Upper byte. So | buf[0] (low) | buf[1] << 8 (high) is correct.

    data->stepCadence = buf[2];

    uint8_t act = buf[3] & 0x03;
    if (act == 1) data->activity = "Walk";
    else if (act == 2) data->activity = "Run";
    else data->activity = "Unknown";
}

/* =================================================================================
 * 底层驱动 (Bank Management)
 * ================================================================================= */

static void ICM_SetBank(ICM42688_Device_t *dev, uint8_t bank) {
    if (dev->currentBank != bank) {
        uint8_t regAddr = REG_BANK_SEL & 0xFF;
        uint8_t tx[2] = {regAddr, bank};

        HAL_GPIO_WritePin(dev->csPort, dev->csPin, GPIO_PIN_RESET);
        HAL_SPI_Transmit(dev->spiHandle, tx, 2, 100);
        HAL_GPIO_WritePin(dev->csPort, dev->csPin, GPIO_PIN_SET);

        dev->currentBank = bank;
    }
}

static void ICM_WriteReg(ICM42688_Device_t *dev, uint16_t reg, uint8_t val) {
    uint8_t bank = (uint8_t)(reg >> 8);
    uint8_t addr = (uint8_t)(reg & 0xFF);

    ICM_SetBank(dev, bank);

    uint8_t tx[2] = {addr & 0x7F, val};
    HAL_GPIO_WritePin(dev->csPort, dev->csPin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(dev->spiHandle, tx, 2, 100);
    HAL_GPIO_WritePin(dev->csPort, dev->csPin, GPIO_PIN_SET);
}

static uint8_t ICM_ReadReg(ICM42688_Device_t *dev, uint16_t reg) {
    uint8_t bank = (uint8_t)(reg >> 8);
    uint8_t addr = (uint8_t)(reg & 0xFF);
    uint8_t val = 0;

    ICM_SetBank(dev, bank);

    uint8_t tx = addr | 0x80;
    HAL_GPIO_WritePin(dev->csPort, dev->csPin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(dev->spiHandle, &tx, 1, 100);
    HAL_SPI_Receive(dev->spiHandle, &val, 1, 100);
    HAL_GPIO_WritePin(dev->csPort, dev->csPin, GPIO_PIN_SET);

    return val;
}

static void ICM_ReadRegs(ICM42688_Device_t *dev, uint16_t reg, uint8_t *buf, uint16_t len) {
    uint8_t bank = (uint8_t)(reg >> 8);
    uint8_t addr = (uint8_t)(reg & 0xFF);

    ICM_SetBank(dev, bank);

    uint8_t tx = addr | 0x80;
    HAL_GPIO_WritePin(dev->csPort, dev->csPin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(dev->spiHandle, &tx, 1, 100);
    HAL_SPI_Receive(dev->spiHandle, buf, len, 100);
    HAL_GPIO_WritePin(dev->csPort, dev->csPin, GPIO_PIN_SET);
}