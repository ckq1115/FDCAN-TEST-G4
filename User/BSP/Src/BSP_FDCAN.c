#include "BSP-FDCAN.h"

/**
 * @brief 通用 FDCAN 过滤器与启动配置
 * @param hfdcan FDCAN句柄
 * @param fifo   使用的 FIFO (FDCAN_RX_FIFO0 或 FDCAN_RX_FIFO1)
 */
void FDCAN_Config(FDCAN_HandleTypeDef *hfdcan, uint32_t fifo)
{
    FDCAN_FilterTypeDef sFilterConfig = {0};

    /* 1. 配置过滤器：默认接收所有标准帧到指定的 FIFO */
    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterType = FDCAN_FILTER_MASK;
    sFilterConfig.FilterConfig = (fifo == FDCAN_RX_FIFO0) ? FDCAN_FILTER_TO_RXFIFO0 : FDCAN_FILTER_TO_RXFIFO1;
    sFilterConfig.FilterID1 = 0x000;
    sFilterConfig.FilterID2 = 0x000; // 掩码为0表示接收所有ID

    if (HAL_FDCAN_ConfigFilter(hfdcan, &sFilterConfig) != HAL_OK) Error_Handler();

    /* 2. 全局过滤：拒绝远程帧，拒绝不匹配的扩展帧 */
    if (HAL_FDCAN_ConfigGlobalFilter(hfdcan, FDCAN_REJECT, FDCAN_REJECT,
                                    FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK) Error_Handler();

    /* 3. 开启中断：根据传入的 FIFO 开启对应的中断源 */
    uint32_t it_source = (fifo == FDCAN_RX_FIFO0) ? FDCAN_IT_RX_FIFO0_NEW_MESSAGE : FDCAN_IT_RX_FIFO1_NEW_MESSAGE;
    if (HAL_FDCAN_ActivateNotification(hfdcan, it_source, 0) != HAL_OK) Error_Handler();

    /* 4. 启动外设 */
    if (HAL_FDCAN_Start(hfdcan) != HAL_OK) Error_Handler();
}

/**
 * @brief 将字节长度转换为 FDCAN 的 DLC 宏
 */
static uint32_t BytesToDLC(uint32_t len) {
    if (len <= 8)  return len << 16; // 0-8 字节规律一致
    if (len <= 12) return FDCAN_DLC_BYTES_12;
    if (len <= 16) return FDCAN_DLC_BYTES_16;
    if (len <= 20) return FDCAN_DLC_BYTES_20;
    if (len <= 24) return FDCAN_DLC_BYTES_24;
    if (len <= 32) return FDCAN_DLC_BYTES_32;
    if (len <= 48) return FDCAN_DLC_BYTES_48;
    return FDCAN_DLC_BYTES_64;
}

uint8_t FDCAN_Send_Msg(FDCAN_HandleTypeDef *hfdcan, uint32_t id, uint8_t *data, uint32_t len)
{
    FDCAN_TxHeaderTypeDef TxHeader = {
        .Identifier = id,
        .IdType = FDCAN_STANDARD_ID,
        .TxFrameType = FDCAN_DATA_FRAME,
        .DataLength = BytesToDLC(len), // 查表转换
        .ErrorStateIndicator = FDCAN_ESI_ACTIVE,
        .BitRateSwitch = FDCAN_BRS_OFF,
        .FDFormat = (len > 8) ? FDCAN_FD_CAN : FDCAN_CLASSIC_CAN, // 自动切换 FD 模式
        .TxEventFifoControl = FDCAN_NO_TX_EVENTS,
        .MessageMarker = 0
    };

    return (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxHeader, data) == HAL_OK) ? 0 : 1;
}