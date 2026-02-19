#include "BSP-FDCAN.h"

/**
 * @brief FDCAN外设配置函数
 * @param hfdcan FDCAN句柄
 * @param fifo   选择接收FIFO（FDCAN_RX_FIFO0或FDCAN_RX_FIFO1）
 * @note 该函数完成以下配置：
 *       0. 重置外设：在配置前先进行去初始化和重新初始化
 *       1. 配置过滤器：默认接收所有标准帧到指定的 FIFO
 *       2. 全局过滤：拒绝远程帧，拒绝不匹配的扩展帧
 *       3. 开启中断：根据传入的 FIFO 开启对应的中断源（含溢出、丢失等）
 *       4. 启动外设
 */
void FDCAN_Config(FDCAN_HandleTypeDef *hfdcan, uint32_t fifo)
{
    // 重置FDCAN外设：去初始化后重新初始化，确保外设处于干净状态
    if (HAL_FDCAN_DeInit(hfdcan) != HAL_OK) Error_Handler();
    if (HAL_FDCAN_Init(hfdcan) != HAL_OK) Error_Handler();

    FDCAN_FilterTypeDef sFilterConfig = {0};
    // 过滤器配置：仅使用标准帧过滤器，接收所有ID
    sFilterConfig.IdType = FDCAN_STANDARD_ID; // 仅配置标准帧过滤器
    sFilterConfig.FilterIndex = 0; // 过滤器索引，0表示第一个过滤器
    sFilterConfig.FilterType = FDCAN_FILTER_MASK; // 掩码模式：FilterID1与FilterID2配合使用，0表示接收所有ID
    sFilterConfig.FilterConfig = (fifo == FDCAN_RX_FIFO0) ? FDCAN_FILTER_TO_RXFIFO0 : FDCAN_FILTER_TO_RXFIFO1; // 根据参数选择接收FIFO
    sFilterConfig.FilterID1 = 0x000; // 标识符掩码为0表示接收所有ID
    sFilterConfig.FilterID2 = 0x000; // 0表示掩码模式下的ID掩码，配合FilterID1使用

    if (HAL_FDCAN_ConfigFilter(hfdcan, &sFilterConfig) != HAL_OK) Error_Handler();

    // 全局过滤配置：拒绝远程帧，拒绝不匹配的扩展帧，允许所有标准帧
    if (HAL_FDCAN_ConfigGlobalFilter(hfdcan, FDCAN_REJECT, FDCAN_REJECT,
                                    FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK) Error_Handler();

    // 根据选择的 FIFO 开启对应的中断源
    uint32_t it_source;
    if (fifo == FDCAN_RX_FIFO0)
    {
        it_source = FDCAN_IT_RX_FIFO0_NEW_MESSAGE |   // 新消息到达
                    FDCAN_IT_RX_FIFO0_FULL |           // FIFO满（预警）
                    FDCAN_IT_RX_FIFO0_MESSAGE_LOST;    // 消息丢失（溢出）
        // 可选：FDCAN_IT_RX_FIFO0_WATERMARK         // 水线中断（FIFO填充到一定程度）
    }
    else // FDCAN_RX_FIFO1
    {
        it_source = FDCAN_IT_RX_FIFO1_NEW_MESSAGE |   // 新消息到达
                    FDCAN_IT_RX_FIFO1_FULL |           // FIFO满（预警）
                    FDCAN_IT_RX_FIFO1_MESSAGE_LOST;    // 消息丢失（溢出）
        // 可选：FDCAN_IT_RX_FIFO1_WATERMARK
    }
    if (HAL_FDCAN_ActivateNotification(hfdcan, it_source, 0) != HAL_OK) Error_Handler();
    // 启动 FDCAN 外设
    if (HAL_FDCAN_Start(hfdcan) != HAL_OK) Error_Handler();
}

/**
 * @brief 将字节长度转换为 FDCAN DLC 值
 * @param len 数据长度（字节）
 * @return 对应的 DLC 值
 */
static uint32_t BytesToDLC(uint32_t len) {
    switch (len) {
        case 0:  return FDCAN_DLC_BYTES_0;
        case 1:  return FDCAN_DLC_BYTES_1;
        case 2:  return FDCAN_DLC_BYTES_2;
        case 3:  return FDCAN_DLC_BYTES_3;
        case 4:  return FDCAN_DLC_BYTES_4;
        case 5:  return FDCAN_DLC_BYTES_5;
        case 6:  return FDCAN_DLC_BYTES_6;
        case 7:  return FDCAN_DLC_BYTES_7;
        case 8:  return FDCAN_DLC_BYTES_8;
            // FDCAN模式下的长字节
        case 12: return FDCAN_DLC_BYTES_12;
        case 16: return FDCAN_DLC_BYTES_16;
        case 20: return FDCAN_DLC_BYTES_20;
        case 24: return FDCAN_DLC_BYTES_24;
        case 32: return FDCAN_DLC_BYTES_32;
        case 48: return FDCAN_DLC_BYTES_48;
        case 64: return FDCAN_DLC_BYTES_64;
        default: return FDCAN_DLC_BYTES_8; // 默认给8，防止越界
    }
}

/**
 * @brief FDCAN发送通用函数
 * @param hfdcan FDCAN句柄
 * @param id     消息ID（标准帧）
 * @param data   数据指针
 * @param len    数据长度（字节）
 * @return 0表示成功，1表示失败
 */
uint8_t FDCAN_Send_Msg(FDCAN_HandleTypeDef *hfdcan, uint32_t id, uint8_t *data, uint32_t len)
{
    FDCAN_TxHeaderTypeDef TxHeader = {
        .Identifier = id, // 标准帧ID
        .IdType = FDCAN_STANDARD_ID, // 标准帧
        .TxFrameType = FDCAN_DATA_FRAME, // 数据帧
        .DataLength = BytesToDLC(len), // 根据长度转换为DLC
        .ErrorStateIndicator = FDCAN_ESI_ACTIVE, // 错误状态指示器：主动状态
        .BitRateSwitch = FDCAN_BRS_OFF, // 位速率切换：关闭（如果需要FD模式请改为FDCAN_BRS_ON）
        .FDFormat = (len > 8) ? FDCAN_FD_CAN : FDCAN_CLASSIC_CAN, // 自动切换 FD 模式
        .TxEventFifoControl = FDCAN_NO_TX_EVENTS, // 不使用事件FIFO
        .MessageMarker = 0 // 消息标记，用户自定义用途
    };
    return (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxHeader, data) == HAL_OK) ? 0 : 1;
}