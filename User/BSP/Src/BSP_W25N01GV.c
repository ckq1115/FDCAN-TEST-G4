//
// Created by CaoKangqi on 2026/2/16.
//
#include "BSP_W25N01GV.h"
#include "quadspi.h"

QSPI_HandleTypeDef hqspi;
volatile uint8_t QSPI_Transfer_Complete = 0;

/* 中断回调函数：当 DMA 传输完成后由 HAL 库自动调用 */
void HAL_QSPI_TxCpltCallback(QSPI_HandleTypeDef *hqspi) { QSPI_Transfer_Complete = 1; }
void HAL_QSPI_RxCpltCallback(QSPI_HandleTypeDef *hqspi) { QSPI_Transfer_Complete = 1; }

/* 等待 Flash Busy 位清零 */
static void W25N_WaitBusy(void) {
    QSPI_CommandTypeDef sCommand = {0};
    uint8_t status = 0;

    sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
    sCommand.Instruction       = W25N_CMD_READ_STATUS;
    sCommand.AddressMode       = QSPI_ADDRESS_1_LINE;
    sCommand.AddressSize       = QSPI_ADDRESS_8_BITS;
    sCommand.Address           = W25N_SR3_STATUS;
    sCommand.DataMode          = QSPI_DATA_1_LINE;
    sCommand.NbData            = 1;

    do {
        HAL_QSPI_Command(&hqspi, &sCommand, 100);
        HAL_QSPI_Receive(&hqspi, &status, 100);
    } while (status & 0x01); /* Bit 0 为 Busy 位 */
}

/* --- 公有接口函数 --- */

uint8_t W25N01GV_Init(void) {
    QSPI_CommandTypeDef sCommand = {0};

    /* 1. 硬件复位 */
    sCommand.InstructionMode = QSPI_INSTRUCTION_1_LINE;
    sCommand.Instruction     = W25N_CMD_RESET;
    HAL_QSPI_Command(&hqspi, &sCommand, 100);
    HAL_Delay(1);
    W25N_WaitBusy();

    /* 2. 解除写保护：将 SR1 设为 0x00 */
    sCommand.Instruction = W25N_CMD_WRITE_STATUS;
    sCommand.AddressMode = QSPI_ADDRESS_1_LINE;
    sCommand.Address     = W25N_SR1_PROTECTION;
    sCommand.DataMode    = QSPI_DATA_1_LINE;
    sCommand.NbData     = 1;
    uint8_t prot = 0x00;
    HAL_QSPI_Command(&hqspi, &sCommand, 100);
    HAL_QSPI_Transmit(&hqspi, &prot, 100);

    return 0;
}

uint8_t W25N01GV_ReadID(uint8_t *id) {
    QSPI_CommandTypeDef sCommand = {0};
    sCommand.InstructionMode = QSPI_INSTRUCTION_1_LINE;
    sCommand.Instruction     = W25N_CMD_JEDEC_ID;
    sCommand.DummyCycles     = 8;
    sCommand.DataMode        = QSPI_DATA_1_LINE;
    sCommand.NbData          = 3;

    if (HAL_QSPI_Command(&hqspi, &sCommand, 100) != HAL_OK) return 1;
    return (HAL_QSPI_Receive(&hqspi, id, 100) == HAL_OK) ? 0 : 1;
}

/* 读取一页：分两步 */
uint8_t W25N01GV_ReadPage(uint16_t pageAddr, uint8_t *pBuffer, uint16_t size) {
    QSPI_CommandTypeDef sCommand = {0};

    /* 第一步：将阵列数据搬运到 Buffer */
    sCommand.Instruction      = W25N_CMD_PAGE_DATA_READ;
    sCommand.InstructionMode  = QSPI_INSTRUCTION_1_LINE;
    sCommand.AddressMode      = QSPI_ADDRESS_1_LINE;
    sCommand.AddressSize      = QSPI_ADDRESS_16_BITS;
    sCommand.Address          = pageAddr;
    HAL_QSPI_Command(&hqspi, &sCommand, 100);
    W25N_WaitBusy();

    /* 第二步：从 Buffer 使用 Quad 模式读出 */
    sCommand.Instruction      = W25N_CMD_READ_DATA_QUAD;
    sCommand.Address          = 0; /* Column Addr: 0 */
    sCommand.DataMode         = QSPI_DATA_4_LINES;
    sCommand.DummyCycles      = 8;
    sCommand.NbData           = size;

    QSPI_Transfer_Complete = 0;
    HAL_QSPI_Command(&hqspi, &sCommand, 100);
    HAL_QSPI_Receive_DMA(&hqspi, pBuffer);

    while (!QSPI_Transfer_Complete); /* 等待中断回调 */
    return 0;
}

/* 写入一页：分三步 */
uint8_t W25N01GV_WritePage(uint16_t pageAddr, uint8_t *pBuffer, uint16_t size) {
    QSPI_CommandTypeDef sCommand = {0};

    /* 1. 写使能 */
    sCommand.InstructionMode = QSPI_INSTRUCTION_1_LINE;
    sCommand.Instruction     = W25N_CMD_WRITE_ENABLE;
    HAL_QSPI_Command(&hqspi, &sCommand, 100);

    /* 2. 将数据送入 Buffer (Quad 模式) */
    sCommand.Instruction     = W25N_CMD_LOAD_PROGRAM_QUAD;
    sCommand.AddressMode     = QSPI_ADDRESS_1_LINE;
    sCommand.AddressSize     = QSPI_ADDRESS_16_BITS;
    sCommand.Address         = 0; /* 从页首开始 */
    sCommand.DataMode        = QSPI_DATA_4_LINES;
    sCommand.NbData          = size;

    QSPI_Transfer_Complete = 0;
    HAL_QSPI_Command(&hqspi, &sCommand, 100);
    HAL_QSPI_Transmit_DMA(&hqspi, pBuffer);
    while (!QSPI_Transfer_Complete);

    /* 3. 执行物理写入 (Buffer -> Array) */
    sCommand.Instruction     = W25N_CMD_PROGRAM_EXECUTE;
    sCommand.Address         = pageAddr;
    sCommand.DataMode        = QSPI_DATA_NONE;
    HAL_QSPI_Command(&hqspi, &sCommand, 100);

    W25N_WaitBusy();
    return 0;
}

uint8_t W25N01GV_EraseBlock(uint16_t blockAddr) {
    QSPI_CommandTypeDef sCommand = {0};

    /* 写使能 */
    sCommand.InstructionMode = QSPI_INSTRUCTION_1_LINE;
    sCommand.Instruction     = W25N_CMD_WRITE_ENABLE;
    HAL_QSPI_Command(&hqspi, &sCommand, 100);

    /* 块擦除指令 */
    sCommand.Instruction     = W25N_CMD_BLOCK_ERASE;
    sCommand.AddressMode     = QSPI_ADDRESS_1_LINE;
    sCommand.AddressSize     = QSPI_ADDRESS_16_BITS;
    sCommand.Address         = blockAddr * W25N_BLOCK_SIZE;
    HAL_QSPI_Command(&hqspi, &sCommand, 100);

    W25N_WaitBusy();
    return 0;
}