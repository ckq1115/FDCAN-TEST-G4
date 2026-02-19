//
// Created by CaoKangqi on 2026/2/16.
//
#include "BSP_W25N01GV.h"
#include "quadspi.h"

extern QSPI_HandleTypeDef hqspi1;
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

static void W25N_InitCommand(QSPI_CommandTypeDef *cmd) {
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

static uint8_t W25N_ReadStatus(uint8_t reg, uint8_t *value) {
    QSPI_CommandTypeDef cmd;
    W25N_InitCommand(&cmd);
    cmd.Instruction = W25N_CMD_READ_STATUS;
    cmd.AddressMode = QSPI_ADDRESS_1_LINE;
    cmd.AddressSize = QSPI_ADDRESS_8_BITS;
    cmd.Address     = reg;
    cmd.DataMode    = QSPI_DATA_1_LINE;
    cmd.NbData      = 1;

    if (HAL_QSPI_Command(&hqspi1, &cmd, W25N_TIMEOUT_MS) != HAL_OK) return 1;
    return (HAL_QSPI_Receive(&hqspi1, value, W25N_TIMEOUT_MS) == HAL_OK) ? 0 : 1;
}

static uint8_t W25N_WriteStatus(uint8_t reg, uint8_t value) {
    QSPI_CommandTypeDef cmd;
    W25N_InitCommand(&cmd);
    cmd.Instruction = W25N_CMD_WRITE_STATUS;
    cmd.AddressMode = QSPI_ADDRESS_1_LINE;
    cmd.AddressSize = QSPI_ADDRESS_8_BITS;
    cmd.Address     = reg;
    cmd.DataMode    = QSPI_DATA_1_LINE;
    cmd.NbData      = 1;

    if (HAL_QSPI_Command(&hqspi1, &cmd, W25N_TIMEOUT_MS) != HAL_OK) return 1;
    return (HAL_QSPI_Transmit(&hqspi1, &value, W25N_TIMEOUT_MS) == HAL_OK) ? 0 : 1;
}

static uint8_t W25N_WaitBusy(uint32_t timeout_ms) {
    uint32_t start = HAL_GetTick();
    uint8_t status = 0;

    do {
        if (W25N_ReadStatus(W25N_SR3_STATUS, &status)) return 1;
        if ((HAL_GetTick() - start) > timeout_ms) return 1;
    } while (status & W25N_SR3_BUSY_BIT);

    return 0;
}

static uint8_t W25N_WaitWEL(uint32_t timeout_ms) {
    uint32_t start = HAL_GetTick();
    uint8_t status = 0;

    do {
        if (W25N_ReadStatus(W25N_SR3_STATUS, &status)) return 1;
        if ((HAL_GetTick() - start) > timeout_ms) return 1;
    } while ((status & W25N_SR3_WEL_BIT) == 0);

    return 0;
}

static uint8_t W25N_CheckProgEraseFail(void) {
    uint8_t status = 0;
    if (W25N_ReadStatus(W25N_SR3_STATUS, &status)) return 1;
    if (status & (W25N_SR3_EFAIL_BIT | W25N_SR3_PFAIL_BIT)) return 1;
    return 0;
}

/* --- 公有接口函数 --- */

uint8_t W25N01GV_Init(void) {
    QSPI_CommandTypeDef cmd;
    uint8_t reg = 0;

    W25N_InitCommand(&cmd);

    /* 1. 复位 */
    cmd.Instruction = W25N_CMD_RESET;
    if (HAL_QSPI_Command(&hqspi1, &cmd, W25N_TIMEOUT_MS) != HAL_OK) return 1;
    HAL_Delay(1);
    if (W25N_WaitBusy(W25N_TIMEOUT_MS)) return 1;

    /* 2. 解除写保护 */
    if (W25N_WriteStatus(W25N_SR1_PROTECTION, 0x00)) return 1;

    /* 3. 使能 Quad 模式 */
    if (W25N_ReadStatus(W25N_SR2_CONFIGURATION, &reg)) return 1;
    if ((reg & W25N_SR2_QE_BIT) == 0) {
        reg |= W25N_SR2_QE_BIT;
        if (W25N_WriteStatus(W25N_SR2_CONFIGURATION, reg)) return 1;
    }

    return 0;
}

uint8_t W25N01GV_ReadID(uint8_t *id) {
    QSPI_CommandTypeDef cmd;
    if (id == NULL) return 1;

    W25N_InitCommand(&cmd);
    cmd.Instruction = W25N_CMD_JEDEC_ID;
    cmd.DataMode    = QSPI_DATA_1_LINE;
    cmd.DummyCycles = 8;
    cmd.NbData      = 3;

    if (HAL_QSPI_Command(&hqspi1, &cmd, W25N_TIMEOUT_MS) != HAL_OK) return 1;
    return (HAL_QSPI_Receive(&hqspi1, id, W25N_TIMEOUT_MS) == HAL_OK) ? 0 : 1;
}

/* 读取一页：分两步 */
uint8_t W25N01GV_ReadPage(uint16_t pageAddr, uint8_t *pBuffer, uint16_t size) {
    QSPI_CommandTypeDef cmd;
    uint32_t start = 0;

    if ((pBuffer == NULL) || (size == 0) || (size > W25N_PAGE_SIZE)) return 1;

    W25N_InitCommand(&cmd);

    /* 第一步：将阵列数据搬运到 Buffer */
    cmd.Instruction     = W25N_CMD_PAGE_DATA_READ;
    cmd.AddressMode     = QSPI_ADDRESS_1_LINE;
    cmd.AddressSize     = QSPI_ADDRESS_16_BITS;
    cmd.Address         = pageAddr;
    if (HAL_QSPI_Command(&hqspi1, &cmd, W25N_TIMEOUT_MS) != HAL_OK) return 1;
    if (W25N_WaitBusy(W25N_TIMEOUT_MS)) return 1;

    /* 第二步：从 Buffer 使用 Quad 模式读出 */
    W25N_InitCommand(&cmd);
    cmd.Instruction     = W25N_CMD_READ_DATA_QUAD;
    cmd.AddressMode     = QSPI_ADDRESS_1_LINE;
    cmd.AddressSize     = QSPI_ADDRESS_16_BITS;
    cmd.Address         = 0; /* Column Addr */
    cmd.DataMode        = QSPI_DATA_4_LINES;
    cmd.DummyCycles     = 8;
    cmd.NbData          = size;

    qspi_xfer_done = 0;
    if (HAL_QSPI_Command(&hqspi1, &cmd, W25N_TIMEOUT_MS) != HAL_OK) return 1;
    if (HAL_QSPI_Receive_DMA(&hqspi1, pBuffer) != HAL_OK) return 1;

    start = HAL_GetTick();
    while (!qspi_xfer_done) {
        if ((HAL_GetTick() - start) > W25N_TIMEOUT_MS) return 1;
    }

    return 0;
}

/* 写入一页：分三步 */
uint8_t W25N01GV_WritePage(uint16_t pageAddr, uint8_t *pBuffer, uint16_t size) {
    QSPI_CommandTypeDef cmd;
    uint32_t start = 0;

    if ((pBuffer == NULL) || (size == 0) || (size > W25N_PAGE_SIZE)) return 1;

    W25N_InitCommand(&cmd);

    /* 1. 写使能 */
    cmd.Instruction = W25N_CMD_WRITE_ENABLE;
    if (HAL_QSPI_Command(&hqspi1, &cmd, W25N_TIMEOUT_MS) != HAL_OK) return 1;
    if (W25N_WaitWEL(W25N_TIMEOUT_MS)) return 1;

    /* 2. 将数据送入 Buffer (Quad 模式) */
    W25N_InitCommand(&cmd);
    cmd.Instruction     = W25N_CMD_LOAD_PROGRAM_QUAD;
    cmd.AddressMode     = QSPI_ADDRESS_1_LINE;
    cmd.AddressSize     = QSPI_ADDRESS_16_BITS;
    cmd.Address         = 0; /* Column Addr */
    cmd.DataMode        = QSPI_DATA_4_LINES;
    cmd.NbData          = size;

    qspi_xfer_done = 0;
    if (HAL_QSPI_Command(&hqspi1, &cmd, W25N_TIMEOUT_MS) != HAL_OK) return 1;
    if (HAL_QSPI_Transmit_DMA(&hqspi1, pBuffer) != HAL_OK) return 1;

    start = HAL_GetTick();
    while (!qspi_xfer_done) {
        if ((HAL_GetTick() - start) > W25N_TIMEOUT_MS) return 1;
    }

    /* 3. 执行物理写入 (Buffer -> Array) */
    W25N_InitCommand(&cmd);
    cmd.Instruction = W25N_CMD_PROGRAM_EXECUTE;
    cmd.AddressMode = QSPI_ADDRESS_1_LINE;
    cmd.AddressSize = QSPI_ADDRESS_16_BITS;
    cmd.Address     = pageAddr;

    if (HAL_QSPI_Command(&hqspi1, &cmd, W25N_TIMEOUT_MS) != HAL_OK) return 1;
    if (W25N_WaitBusy(W25N_TIMEOUT_MS)) return 1;

    return W25N_CheckProgEraseFail();
}

uint8_t W25N01GV_EraseBlock(uint16_t blockAddr) {
    QSPI_CommandTypeDef cmd;
    uint32_t pageAddr = (uint32_t)blockAddr * W25N_BLOCK_SIZE;

    if (blockAddr >= W25N_TOTAL_BLOCKS) return 1;

    W25N_InitCommand(&cmd);

    /* 写使能 */
    cmd.Instruction = W25N_CMD_WRITE_ENABLE;
    if (HAL_QSPI_Command(&hqspi1, &cmd, W25N_TIMEOUT_MS) != HAL_OK) return 1;
    if (W25N_WaitWEL(W25N_TIMEOUT_MS)) return 1;

    /* 块擦除指令 */
    W25N_InitCommand(&cmd);
    cmd.Instruction = W25N_CMD_BLOCK_ERASE;
    cmd.AddressMode = QSPI_ADDRESS_1_LINE;
    cmd.AddressSize = QSPI_ADDRESS_16_BITS;
    cmd.Address     = pageAddr;
    if (HAL_QSPI_Command(&hqspi1, &cmd, W25N_TIMEOUT_MS) != HAL_OK) return 1;

    if (W25N_WaitBusy(W25N_TIMEOUT_MS)) return 1;
    return W25N_CheckProgEraseFail();
}