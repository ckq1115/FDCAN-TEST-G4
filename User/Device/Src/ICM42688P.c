//
// Created by CaoKangqi on 2026/1/23.
//
#include "../Inc/ICM42688P.h"

#include <tgmath.h>

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "portmacro.h"
#include "projdefs.h"

float acc_res = 0;
float gyr_res = 0;
static uint8_t current_bank = 0xFF;

/* internal: switch register bank */
static void SelectBank(uint8_t bank) {
    if (current_bank != bank) {
        uint8_t cmd[2] = {REG_BANK_SEL, bank};
        BSP_SPI_CS(0);
        BSP_SPI_Transmit(cmd, 2, 10);
        BSP_SPI_CS(1);
        current_bank = bank;
    }
}

/* internal: write register with auto bank */
static void WriteReg(uint16_t reg, uint8_t val) {
    SelectBank((uint8_t)(reg >> 8));
    uint8_t cmd[2] = {(uint8_t)(reg & 0xFF) & 0x7F, val};
    BSP_SPI_CS(0);
    BSP_SPI_Transmit(cmd, 2, 10);
    BSP_SPI_CS(1);
}

/* internal: read register with auto bank */
static uint8_t ReadReg(uint16_t reg) {
    SelectBank((uint8_t)(reg >> 8));
    uint8_t addr = (uint8_t)(reg & 0xFF) | 0x80, val = 0;
    BSP_SPI_CS(0);
    BSP_SPI_Transmit(&addr, 1, 10);
    BSP_SPI_Receive(&val, 1, 10);
    BSP_SPI_CS(1);
    return val;
}

uint8_t ICM42688_Init(void) {
    // 1. software reset
    WriteReg(REG_DEVICE_CONFIG, 0x01);
    vTaskDelay(pdMS_TO_TICKS(100));

    // 2. ID check
    if (ReadReg(REG_WHO_AM_I) != ICM_WHO_AM_I_VAL) return 1;

    // 3. interrupt pin config
    WriteReg(REG_INT_CONFIG, 0x06);
    WriteReg(REG_INT_SOURCE0, 0x08);

    // 4. power management
    WriteReg(REG_PWR_MGMT0, 0x0F);
    vTaskDelay(pdMS_TO_TICKS(50));

    // 5. default ODR/FSR
    ICM42688_SetFormat(ODR_1kHz, ACCEL_FS_8G, ODR_1kHz, GYRO_FS_2000DPS);

    ICM42688_Config_AAF(1, 7, 49, 9);
    ICM42688_Config_AAF(0, 7, 49, 9);
    ICM42688_Config_UI_Filter(UI_FILT_ORD_2ND, UI_FILT_BW_ODR_DIV_4, UI_FILT_ORD_2ND, UI_FILT_BW_ODR_DIV_4);
    //ICM42688_Config_Gyro_Notch_Filter(1, 1000.0f, GYRO_NF_BW_40HZ);
    SelectBank(0);
    return 0;
}

void ICM42688_SetFormat(ODR_t a_odr, AccelFS_t a_fsr, ODR_t g_odr, GyroFS_t g_fsr) {
    WriteReg(REG_ACCEL_CONFIG0, (a_fsr << 5) | a_odr);
    WriteReg(REG_GYRO_CONFIG0,  (g_fsr << 5) | g_odr);

    acc_res = (16.0f / (float)(1 << a_fsr)) / 32768.0f * 9.81f;
    gyr_res = (2000.0f / (float)(1 << g_fsr)) / 32768.0f;
}

void ICM42688_Config_FIFO(uint8_t enable) {
    if(enable) {
        WriteReg(REG_FIFO_CONFIG, 0x40);
    } else {
        WriteReg(REG_FIFO_CONFIG, 0x00);
    }
}

uint8_t ICM42688_IsDataReady(void) {
     return (ReadReg(REG_INT_STATUS) & 0x08);
}

void ICM42688_read(float gyro[3], float accel[3], float *temperature)
{
    uint8_t buf[14];
    int16_t raw_temp;

    uint8_t addr = (REG_TEMP_DATA1 & 0xFF) | 0x80;

    BSP_SPI_CS(0);
    BSP_SPI_Transmit(&addr, 1, 10);
    BSP_SPI_Receive(buf, 14, 10);
    BSP_SPI_CS(1);

    accel[0] = (int16_t)((buf[2] << 8) | buf[3]) * acc_res;
    accel[1] = (int16_t)((buf[4] << 8) | buf[5]) * acc_res;
    accel[2] = (int16_t)((buf[6] << 8) | buf[7]) * acc_res;

    gyro[0] = (int16_t)((buf[8] << 8) | buf[9])   * gyr_res;
    gyro[1] = (int16_t)((buf[10] << 8) | buf[11]) * gyr_res;
    gyro[2] = (int16_t)((buf[12] << 8) | buf[13]) * gyr_res;

    raw_temp = (int16_t)((buf[0] << 8) | buf[1]);
    *temperature = (raw_temp / 132.48f) + 25.0f;
}

void ICM42688_Read_Fast(float gyro[3], float accel[3], float *temperature)
{
    uint8_t raw_data[14];

    SPI_TypeDef *spi_inst = BSP_SPI_GetInstance();

    ICM_CS_PORT->BSRR = (uint32_t)ICM_CS_PIN << 16;

    while (!(spi_inst->SR & SPI_FLAG_TXE));
    *(__IO uint8_t *)&spi_inst->DR = (REG_TEMP_DATA1 | 0x80);

    while (!(spi_inst->SR & SPI_FLAG_RXNE));
    __IO uint8_t trash = *(__IO uint8_t *)&spi_inst->DR;
    (void)trash;

    for (int i = 0; i < 14; i++) {
        while (!(spi_inst->SR & SPI_FLAG_TXE));
        *(__IO uint8_t *)&spi_inst->DR = 0xFF;

        while (!(spi_inst->SR & SPI_FLAG_RXNE));
        raw_data[i] = *(__IO uint8_t *)&spi_inst->DR;
    }

    ICM_CS_PORT->BSRR = ICM_CS_PIN;

    int16_t t_raw = (int16_t)((raw_data[0] << 8) | raw_data[1]);

    accel[0] = (int16_t)((raw_data[2] << 8) | raw_data[3]) * acc_res;
    accel[1] = (int16_t)((raw_data[4] << 8) | raw_data[5]) * acc_res;
    accel[2] = (int16_t)((raw_data[6] << 8) | raw_data[7]) * acc_res;

    gyro[0] = (int16_t)((raw_data[8] << 8) | raw_data[9])   * gyr_res;
    gyro[1] = (int16_t)((raw_data[10] << 8) | raw_data[11]) * gyr_res;
    gyro[2] = (int16_t)((raw_data[12] << 8) | raw_data[13]) * gyr_res;

    *temperature = (t_raw / 132.48f) + 25.0f;
}

#ifndef ICM_PI
#define ICM_PI 3.14159265358979323846f
#endif

void ICM42688_Config_UI_Filter(UIFiltOrd_t a_ord, UIFiltBW_t a_bw, UIFiltOrd_t g_ord, UIFiltBW_t g_bw) {
    uint8_t reg_acc_conf1 = ReadReg(REG_ACCEL_CONFIG1);
    reg_acc_conf1 &= ~(0x18);
    reg_acc_conf1 |= (a_ord << 3);
    WriteReg(REG_ACCEL_CONFIG1, reg_acc_conf1);

    uint8_t reg_gyr_conf1 = ReadReg(REG_GYRO_CONFIG0);
    (void)reg_gyr_conf1;
    uint8_t val_gyr_c1 = ReadReg(REG_GYRO_CONFIG1);
    val_gyr_c1 &= ~(0x0C);
    val_gyr_c1 |= (g_ord << 2);
    WriteReg(REG_GYRO_CONFIG1, val_gyr_c1);

    uint8_t bw_reg = (a_bw << 4) | (g_bw & 0x0F);
    WriteReg(REG_GYRO_ACCEL_CONFIG0, bw_reg);
}

void ICM42688_Config_Gyro_Notch_Filter(uint8_t enable, float center_freq_hz, GyroNFBW_t bw_sel) {
    uint8_t static2 = ReadReg(REG_GYRO_CONFIG_STATIC2);
    if (enable) {
        static2 &= ~0x01;
    } else {
        static2 |= 0x01;
        WriteReg(REG_GYRO_CONFIG_STATIC2, static2);
        return;
    }
    WriteReg(REG_GYRO_CONFIG_STATIC2, static2);

    float f_desired_khz = center_freq_hz / 1000.0f;
    float coswz = cosf(2 * ICM_PI * f_desired_khz / 32.0f);

    int16_t nf_coswz_val;
    uint8_t nf_coswz_sel;

    if (fabsf(coswz) <= 0.875f) {
        nf_coswz_val = (int16_t)roundf(coswz * 256.0f);
        nf_coswz_sel = 0;
    } else {
        nf_coswz_sel = 1;
        if (coswz > 0.875f) {
            nf_coswz_val = (int16_t)roundf(8.0f * (1.0f - coswz) * 256.0f);
        } else {
            nf_coswz_val = (int16_t)roundf(-8.0f * (1.0f + coswz) * 256.0f);
        }
    }

    uint8_t low_byte = (uint8_t)(nf_coswz_val & 0xFF);
    WriteReg(REG_GYRO_CONFIG_STATIC6, low_byte);
    WriteReg(REG_GYRO_CONFIG_STATIC7, low_byte);
    WriteReg(REG_GYRO_CONFIG_STATIC8, low_byte);

    uint8_t msb = (nf_coswz_val >> 8) & 0x01;
    uint8_t static9 = 0;

    if(msb) static9 |= (1<<0) | (1<<1) | (1<<2);
    if(nf_coswz_sel) static9 |= (1<<3) | (1<<4) | (1<<5);

    WriteReg(REG_GYRO_CONFIG_STATIC9, static9);

    uint8_t static10 = ReadReg(REG_GYRO_CONFIG_STATIC10);
    static10 &= ~(0x70);
    static10 |= (bw_sel << 4);
    WriteReg(REG_GYRO_CONFIG_STATIC10, static10);
}

void ICM42688_Config_AAF(uint8_t is_accel, uint8_t delt, uint16_t deltSqr, uint8_t bitshift) {
    if (is_accel) {
        if (delt == 0) {
            uint8_t val = ReadReg(REG_ACCEL_CONFIG_STATIC2);
            WriteReg(REG_ACCEL_CONFIG_STATIC2, val | 0x01);
        } else {
            WriteReg(REG_ACCEL_CONFIG_STATIC2, (delt << 1) & 0xFE);
            WriteReg(REG_ACCEL_CONFIG_STATIC3, (uint8_t)(deltSqr & 0xFF));
            uint8_t static4 = (bitshift << 4) | ((deltSqr >> 8) & 0x0F);
            WriteReg(REG_ACCEL_CONFIG_STATIC4, static4);
        }
    } else {
        uint8_t static2 = ReadReg(REG_GYRO_CONFIG_STATIC2);
        if (delt == 0) {
            WriteReg(REG_GYRO_CONFIG_STATIC2, static2 | 0x02);
        } else {
            WriteReg(REG_GYRO_CONFIG_STATIC2, static2 & ~0x02);

            WriteReg(REG_GYRO_CONFIG_STATIC3, delt & 0x3F);
            WriteReg(REG_GYRO_CONFIG_STATIC4, (uint8_t)(deltSqr & 0xFF));
            uint8_t static5 = (bitshift << 4) | ((deltSqr >> 8) & 0x0F);
            WriteReg(REG_GYRO_CONFIG_STATIC5, static5);
        }
    }
}

