//
// Created by CaoKangqi on 2026/2/19.
//
#ifndef G4_FRAMEWORK_BSP_SPI_H
#define G4_FRAMEWORK_BSP_SPI_H

#include "main.h"

extern SPI_HandleTypeDef hspi2;
#define ICM_SPI_HANDLE      &hspi2

/* Pin definition */
#define ICM_CS_PORT         GPIOB
#define ICM_CS_PIN          GPIO_PIN_12
#define ICM_DRDY_PORT       GPIOB
#define ICM_DRDY_PIN        GPIO_PIN_10

void BSP_SPI_CS(uint8_t state);
HAL_StatusTypeDef BSP_SPI_Transmit(const uint8_t *data, uint16_t size, uint32_t timeout);
HAL_StatusTypeDef BSP_SPI_Receive(uint8_t *data, uint16_t size, uint32_t timeout);
SPI_TypeDef *BSP_SPI_GetInstance(void);

#endif // G4_FRAMEWORK_BSP_SPI_H

