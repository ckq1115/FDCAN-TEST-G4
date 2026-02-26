//
// Created by CaoKangqi on 2026/2/23.
//

#ifndef G4_FRAMEWORK_CRC_DJI_H
#define G4_FRAMEWORK_CRC_DJI_H

#include "main.h"

uint32_t Verify_CRC8_Check_Sum( uint8_t *pchMessage, uint16_t dwLength);
void Append_CRC8_Check_Sum( uint8_t *pchMessage, uint16_t dwLength);
uint32_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);
void Append_CRC16_Check_Sum(uint8_t * pchMessage,uint32_t dwLength);

#endif //G4_FRAMEWORK_CRC_DJI_H