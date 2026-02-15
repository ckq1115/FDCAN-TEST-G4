//
// Created by CaoKangqi on 2026/2/16.
//

#ifndef FDCAN_TEST_G4_BSP_W25N01GV_H
#define FDCAN_TEST_G4_BSP_W25N01GV_H

#include "main.h"

/* W25N01GV 常用指令集 */
#define W25N_CMD_RESET              0xFF
#define W25N_CMD_JEDEC_ID           0x9F
#define W25N_CMD_READ_STATUS        0x0F
#define W25N_CMD_WRITE_STATUS       0x1F
#define W25N_CMD_WRITE_ENABLE       0x06
#define W25N_CMD_BLOCK_ERASE        0xD8
#define W25N_CMD_PAGE_DATA_READ     0x13    /* Array -> Buffer */
#define W25N_CMD_READ_DATA_QUAD     0x6B    /* Buffer -> Host (4-Line) */
#define W25N_CMD_LOAD_PROGRAM_QUAD  0x32    /* Host -> Buffer (4-Line) */
#define W25N_CMD_PROGRAM_EXECUTE    0x10    /* Buffer -> Array */

/* 寄存器地址 */
#define W25N_SR1_PROTECTION         0xA0
#define W25N_SR2_CONFIGURATION      0xB0
#define W25N_SR3_STATUS             0xC0

/* 存储结构参数 */
#define W25N_PAGE_SIZE              2048    /* 2KB per page */
#define W25N_BLOCK_SIZE             64      /* 64 pages per block */
#define W25N_TOTAL_BLOCKS           1024    /* 128MB total */

/* 函数声明 */
uint8_t W25N01GV_Init(void);
uint8_t W25N01GV_ReadID(uint8_t *id);
uint8_t W25N01GV_EraseBlock(uint16_t blockAddr);
uint8_t W25N01GV_ReadPage(uint16_t pageAddr, uint8_t *pBuffer, uint16_t size);
uint8_t W25N01GV_WritePage(uint16_t pageAddr, uint8_t *pBuffer, uint16_t size);

#endif //FDCAN_TEST_G4_BSP_W25N01GV_H