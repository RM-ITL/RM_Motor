#include "stm32f4xx.h"// Device header
#include "BSP_CAN.h"
#include "gpio.h"
#include "can.h"

CAN_RxHeaderTypeDef rx_header;
CAN_TxHeaderTypeDef tx_header;
uint8_t can1_rdata[24] = {0};

motor_state_t motor_state;
uint8_t motor_read_flag = 0;
/**
 * @brief 初始化CAN总线
 *
 * @param hcan CAN编号
 * @param Callback_Function 处理回调函数
 */
void CAN_Init(CAN_HandleTypeDef *hcan)
{
  HAL_CAN_Start(hcan);
  __HAL_CAN_ENABLE_IT(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
  __HAL_CAN_ENABLE_IT(hcan, CAN_IT_RX_FIFO1_MSG_PENDING);
}

uint8_t can_send(CAN_HandleTypeDef* hcan,uint16_t id,uint8_t *msg,uint16_t len)
{
  if(id > 0x7FF)
  {
    tx_header.IDE=CAN_ID_EXT;
    tx_header.ExtId=id;
  }
  else
  {
    tx_header.IDE=CAN_ID_STD;
    tx_header.StdId=id;
	}
	tx_header.RTR=CAN_RTR_DATA;
	tx_header.DLC=len;
	if(HAL_CAN_AddTxMessage(hcan, &tx_header, msg, (uint32_t*)CAN_TX_MAILBOX0) != HAL_OK) //
	{
    if(HAL_CAN_AddTxMessage(hcan, &tx_header, msg, (uint32_t*)CAN_TX_MAILBOX1) != HAL_OK)
    {
      HAL_CAN_AddTxMessage(hcan, &tx_header, msg, (uint32_t*)CAN_TX_MAILBOX2);
    }
  }
  return 0;
	
}

void can_filter_init(CAN_HandleTypeDef *hcan)
{

    CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterBank = 0;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(hcan, &can_filter_st);
    can_filter_st.SlaveStartFilterBank = 14;
    can_filter_st.FilterBank = 14;
    HAL_CAN_ConfigFilter(hcan, &can_filter_st);
   
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef header;

    HAL_CAN_GetRxMessage(hcan, CAN_FILTER_FIFO0, &header, can1_rdata);
}

/**
 * @brief 力矩模式
 * @param id 电机ID
 * @param tqe 力矩
 */
void motor_control_tqe(CAN_HandleTypeDef *hcan, uint8_t id, int32_t tqe)
{
    uint8_t tdata[8] = {0xA1, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    tdata[4] = *(uint8_t*)(&tqe);
	tdata[5] = *((uint8_t*)(&tqe)+1);

    can_send(hcan, 0x140 | id, tdata, 8);
}

/**
 * @brief 速度控制
 * @param id 电机ID
 * @param vel 速度：单位 0.01dps/LSB
 * @param tqe 力矩：单位：0.01 NM，如 torque = 110 表示最大力矩为 1.1NM
 */
void motor_control_vel(CAN_HandleTypeDef *hcan, uint8_t id, int32_t vel, int16_t tqe)
{
    uint8_t tdata[8] = {0xA2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    tdata[2] = *(uint8_t*)(&tqe);
	tdata[3] = *((uint8_t*)(&tqe)+1);
	tdata[4] = *(uint8_t*)(&vel);
	tdata[5] = *((uint8_t*)(&vel)+1);
	tdata[6] = *((uint8_t*)(&vel)+2);
	tdata[7] = *((uint8_t*)(&vel)+3);

    can_send(hcan, 0x140 | id, tdata, 8);
}
/**
 * @brief 多圈位置闭环控制
 * @param id 电机ID
 * @param vel 速度：单位 1dps/LSB
 * @param angle 角度：单位：0.01 degree/LSB，如 angle = 36000 表示360度
 */
void motor_angleControl(CAN_HandleTypeDef *hcan, uint8_t id, int32_t angle, uint16_t speed)
{
	uint8_t tdata[8] = {0xA4, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    tdata[2] = *(uint8_t*)(&speed);
	tdata[3] = *((uint8_t*)(&speed)+1);
	tdata[4] = *(uint8_t*)(&angle);
	tdata[5] = *((uint8_t*)(&angle)+1);
	tdata[6] = *((uint8_t*)(&angle)+2);
	tdata[7] = *((uint8_t*)(&angle)+3);

    can_send(hcan, 0x140 | id, tdata, 8);
}
/**
 * @brief 单圈位置闭环控制
 * @param id 电机ID
 * @param spinDirection 电机方向 0x00 顺时针 0x01逆时针
 * @param vel 速度：单位 1dps/LSB
 * @param angle 角度：单位：0.01 degree/LSB，如 angle = 36000 表示360度
 */
void motor_angleControl1(CAN_HandleTypeDef *hcan, uint8_t id, uint32_t angle, uint8_t spinDirection, uint16_t speed)
{
	uint8_t tdata[8] = {0xA6, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

	tdata[1] = spinDirection;
    tdata[2] = *(uint8_t*)(&speed);
	tdata[3] = *((uint8_t*)(&speed)+1);
	tdata[4] = *(uint8_t*)(&angle);
	tdata[5] = *((uint8_t*)(&angle)+1);
	tdata[6] = *((uint8_t*)(&angle)+2);
	tdata[7] = *((uint8_t*)(&angle)+3);

    can_send(hcan, 0x140 | id, tdata, 8);
}
/**
 * @brief 增量位置闭环控制
 * @param id 电机ID
 * @param vel 速度：单位 1dps/LSB
 * @param anglelncrement 角度：单位：0.01 degree/LSB，如 angle = 36000 表示360度
 */
void motor_angleControl2(CAN_HandleTypeDef *hcan, uint8_t id, int32_t anglelncrement, uint32_t speed)
{
	uint8_t tdata[8] = {0xA8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    tdata[2] = *(uint8_t*)(&speed);
	tdata[3] = *((uint8_t*)(&speed)+1);
	tdata[4] = *(uint8_t*)(&anglelncrement);
	tdata[5] = *((uint8_t*)(&anglelncrement)+1);
	tdata[6] = *((uint8_t*)(&anglelncrement)+2);
	tdata[7] = *((uint8_t*)(&anglelncrement)+3);

    can_send(hcan, 0x140 | id, tdata, 8);
}
