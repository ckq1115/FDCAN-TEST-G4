//
// Created by CaoKangqi on 2026/2/19.
//

#ifndef G4_FRAMEWORK_BSP_QSPI_H
#define G4_FRAMEWORK_BSP_QSPI_H

#include "quadspi.h"

void BSP_QSPI_InitCommand(QSPI_CommandTypeDef *cmd);
void BSP_QSPI_ResetXferDone(void);
uint8_t BSP_QSPI_WaitXferDone(uint32_t timeout_ms);

#endif //G4_FRAMEWORK_BSP_QSPI_H