//
// Created by CaoKangqi on 2026/2/19.
//
#include "BSP_SPI.h"

void BSP_SPI_CS(uint8_t state) {
    HAL_GPIO_WritePin(ICM_CS_PORT, ICM_CS_PIN, (GPIO_PinState)state);
}

HAL_StatusTypeDef BSP_SPI_Transmit(const uint8_t *data, uint16_t size, uint32_t timeout) {
    return HAL_SPI_Transmit(ICM_SPI_HANDLE, (uint8_t *)data, size, timeout);
}

HAL_StatusTypeDef BSP_SPI_Receive(uint8_t *data, uint16_t size, uint32_t timeout) {
    return HAL_SPI_Receive(ICM_SPI_HANDLE, data, size, timeout);
}

SPI_TypeDef *BSP_SPI_GetInstance(void) {
    return ((SPI_HandleTypeDef *)ICM_SPI_HANDLE)->Instance;
}

