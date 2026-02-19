//
// Created by CaoKangqi on 2026/2/19.
//
#include "BSP_QSPI.h"

static volatile uint8_t qspi_xfer_done = 0;

/* 中断回调函数：当 DMA 传输完成后由 HAL 库自动调用 */
void HAL_QSPI_TxCpltCallback(QSPI_HandleTypeDef *hqspi) {
    if (hqspi == &hqspi1) {
        qspi_xfer_done = 1;
    }
}
void HAL_QSPI_RxCpltCallback(QSPI_HandleTypeDef *hqspi) {
    if (hqspi == &hqspi1) {
        qspi_xfer_done = 1;
    }
}

void BSP_QSPI_InitCommand(QSPI_CommandTypeDef *cmd) {
    cmd->InstructionMode   = QSPI_INSTRUCTION_1_LINE;
    cmd->Instruction       = 0x00;
    cmd->AddressMode       = QSPI_ADDRESS_NONE;
    cmd->AddressSize       = QSPI_ADDRESS_8_BITS;
    cmd->Address           = 0x00;
    cmd->AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
    cmd->AlternateBytesSize = QSPI_ALTERNATE_BYTES_8_BITS;
    cmd->AlternateBytes    = 0x00;
    cmd->DataMode          = QSPI_DATA_NONE;
    cmd->DummyCycles       = 0;
    cmd->NbData            = 0;
    cmd->DdrMode           = QSPI_DDR_MODE_DISABLE;
    cmd->DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
    cmd->SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;
}

void BSP_QSPI_ResetXferDone(void) {
    qspi_xfer_done = 0;
}

uint8_t BSP_QSPI_WaitXferDone(uint32_t timeout_ms) {
    uint32_t start = HAL_GetTick();

    while (!qspi_xfer_done) {
        if ((HAL_GetTick() - start) > timeout_ms) return 1;
    }

    return 0;
}