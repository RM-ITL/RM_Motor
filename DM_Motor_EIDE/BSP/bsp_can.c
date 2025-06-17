#include "bsp_can.h"

/*
 * @brief  初始化FDCAN控制器并配置中断使能
 * @param  hfdcan: 指向FDCAN句柄的指针，用于管理FDCAN外设
 * @retval 无
 */
void CAN_Init(FDCAN_HandleTypeDef *hfdcan)
{
    HAL_FDCAN_Start(hfdcan);
    HAL_FDCAN_ActivateNotification(hfdcan,
                                       0 | FDCAN_IT_RX_FIFO0_WATERMARK | FDCAN_IT_RX_FIFO0_WATERMARK
                                           | FDCAN_IT_TX_COMPLETE | FDCAN_IT_TX_FIFO_EMPTY | FDCAN_IT_BUS_OFF
                                           | FDCAN_IT_ARB_PROTOCOL_ERROR | FDCAN_IT_DATA_PROTOCOL_ERROR
                                           | FDCAN_IT_ERROR_PASSIVE | FDCAN_IT_ERROR_WARNING,
                                       0x00000F00);
}
/**
  * @brief  初始化FDCAN接收过滤器配置
  * @param  hfdcan - FDCAN句柄指针，指向FDCAN_HandleTypeDef结构体实例
  * @retval None
  */
void can_filter_init(FDCAN_HandleTypeDef *hfdcan)
{
    FDCAN_FilterTypeDef fdcan_filter;
    
    fdcan_filter.IdType = FDCAN_STANDARD_ID;                       //标准ID
    fdcan_filter.FilterIndex = 0;                                  //滤波器索引                   
    fdcan_filter.FilterType = FDCAN_FILTER_MASK;                   
    fdcan_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;           //过滤器0关联到FIFO0  
    fdcan_filter.FilterID1 = 0x00;                               
    fdcan_filter.FilterID2 = 0x00;

    HAL_FDCAN_ConfigFilter(hfdcan,&fdcan_filter);                                                    //接收ID2
    //拒绝接收匹配不成功的标准ID和扩展ID,不接受远程帧
    HAL_FDCAN_ConfigGlobalFilter(hfdcan,FDCAN_REJECT,FDCAN_REJECT,FDCAN_REJECT_REMOTE,FDCAN_REJECT_REMOTE);
    HAL_FDCAN_ConfigFifoWatermark(hfdcan, FDCAN_CFG_RX_FIFO0, 1);
}
/**
 * @brief 发送FDCAN消息
 * @param hfdcan FDCAN句柄指针
 * @param id 消息ID（11位标准帧或29位扩展帧）
 * @param data 数据缓冲区指针
 * @param len 数据长度（最大8字节）
 * @return 0:成功 1:发送失败 2:参数错误
 */
uint8_t fdcan_send(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint8_t len)
{
    // 参数有效性检查
    if(len > 8 || id > 0x1FFFFFFF) {  // 扩展帧最大0x1FFFFFFF
        return 2; // 参数错误
    }

    // 配置发送报文头
    FDCAN_TxHeaderTypeDef pTxHeader = {
        .IdType = (id > 0x7FF) ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID,
        .TxFrameType = FDCAN_DATA_FRAME,
        .DataLength = len,
        .FDFormat = FDCAN_CLASSIC_CAN,
        .BitRateSwitch = FDCAN_BRS_OFF,
        .ErrorStateIndicator = FDCAN_ESI_ACTIVE,
        .TxEventFifoControl = FDCAN_NO_TX_EVENTS,
        .MessageMarker = 0
    };

    // 设置对应ID
    if(pTxHeader.IdType == FDCAN_STANDARD_ID) {
        pTxHeader.Identifier = id & 0x7FF;  // 标准帧11位
    } else {
        pTxHeader.Identifier = id & 0x1FFFFFFF;  // 扩展帧29位
    }

    if(HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &pTxHeader, data)!=HAL_OK) 
        return 1;//发送
    return 0;     

}
/**
 * @brief 经典CAN接收函数（无超时机制）
 * @param hfdcan FDCAN句柄
 * @param rec_id 接收ID（输出）
 * @param buf 数据缓冲区（至少8字节）
 * @retval 0:无数据 1:成功接收 2:数据过长
 */
uint8_t can_receive(FDCAN_HandleTypeDef *hfdcan, uint16_t *rec_id, uint8_t *buf)
{
    // 参数有效性检查（可选）
    if(hfdcan == NULL || rec_id == NULL || buf == NULL) {
        return 2; // 参数错误（简化处理）
    }

    FDCAN_RxHeaderTypeDef pRxHeader;
    
    // 单次尝试接收（不等待）
    if(HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &pRxHeader, buf) != HAL_OK) {
        return 0; // 无数据
    }

    // 提取ID
    *rec_id = pRxHeader.Identifier;
    
    // 数据长度检查（经典CAN最大8字节）
    if(pRxHeader.DataLength > 8) {
        return 2; // 数据长度异常
    }

    return 1; // 成功接收
}


