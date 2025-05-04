#include "stm32f4xx.h"                  // Device header
#include "gpio.h"
#include "can.h"
#include "CAN1.h"
#include "string.h"

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


//高擎历程
CAN_RxHeaderTypeDef rx_header;
CAN_TxHeaderTypeDef tx_header;
uint8_t can1_rdata[24] = {0};

motor_state_t motor_state;
uint8_t motor_read_flag = 0;

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
    HAL_CAN_Start(hcan);
    HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  uint8_t len = 0;
  if(hcan->Instance == CAN1)
  {
      HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, can1_rdata);
      if (rx_header.DLC != 0)
      {
          len = rx_header.DLC;
          motor_state.motor_data.motor.id = rx_header.StdId;

          memcpy(&motor_state.motor_data.data[4], &can1_rdata[2], len - 2);  
          motor_state.motor_data.motor.position = (*(int16_t *)&can1_rdata[2]) * 0.0001f;
          motor_state.motor_data.motor.velocity = (*(int16_t *)&can1_rdata[4]) * 0.00025f;
          motor_state.motor_data.motor.torque = (*(int16_t *)&can1_rdata[6]) * 0.004563f;
          //print_log("motor pos-vel-torque:%lf,%lf,%lf\n", motor_state.motor_data.motor.position, motor_state.motor_data.motor.velocity , motor_state.motor_data.motor.torque);
          motor_read_flag = 1;
      }
  }  
}

/**
 * @brief 位置控制
 * @param id  电机ID
 * @param pos 位置：单位 0.0001 圈，如 pos = 5000 表示转到 0.5 圈的位置。
 * @param 力矩：单位：0.01 NM，如 torque = 110 表示最大力矩为 1.1NM
 */
void motor_control_pos(CAN_HandleTypeDef *hcan, uint8_t id, int32_t pos, int16_t tqe)
{
    uint8_t tdata[8] = {0x07, 0x07, 0x0A, 0x05, 0x00, 0x00, 0x80, 0x00};

    *(int16_t *)&tdata[2] = pos;
    *(int16_t *)&tdata[6] = tqe;

    can_send(hcan, 0x8000 | id, tdata, 8);
}
/**
 * @brief 速度控制
 * @param id 电机ID
 * @param vel 速度：单位 0.00025 转/秒，如 val = 1000 表示 0.25 转/秒
 * @param tqe 力矩：单位：0.01 NM，如 torque = 110 表示最大力矩为 1.1NM
 */
void motor_control_vel(CAN_HandleTypeDef *hcan, uint8_t id, int16_t vel, int16_t tqe)
{
    uint8_t tdata[8] = {0x07, 0x07, 0x00, 0x80, 0x20, 0x00, 0x80, 0x00};

    *(int16_t *)&tdata[4] = vel;
    *(int16_t *)&tdata[6] = tqe;

    can_send(hcan, 0x8000 | id, tdata, 8);
}
/**
 * @brief 力矩模式
 * @param id 电机ID
 * @param tqe 力矩：单位：0.01 NM，如 torque = 110 表示最大力矩为 1.1NM
 */
void motor_control_tqe(CAN_HandleTypeDef *hcan, uint8_t id, int32_t tqe)
{
    uint8_t tdata[8] = {0x05, 0x13, 0x00, 0x80, 0x20, 0x00, 0x80, 0x00};

    *(int16_t *)&tdata[2] = tqe;

    can_send(hcan, 0x8000 | id, tdata, 4);
}
/**
 * @brief 电机位置-速度-最大力矩控制，int16型
 * @param id  电机ID
 * @param pos 位置：单位 0.0001 圈，如 pos = 5000 表示转到 0.5 圈的位置。
 * @param vel 速度：单位 0.00025 转/秒，如 val = 1000 表示 0.25 转/秒
 * @param tqe 最大力矩：单位：0.01 NM，如 torque = 110 表示最大力矩为 1.1NM
 */
void motor_control_pos_vel_tqe(CAN_HandleTypeDef *hcan, uint8_t id, int16_t pos, int16_t vel, int16_t tqe)
{
    static uint8_t tdata[8] = {0x07, 0x35, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    *(int16_t *)&tdata[2] = vel;
    *(int16_t *)&tdata[4] = tqe;
    *(int16_t *)&tdata[6] = pos;

    can_send(hcan, 0x8000 | id, tdata, 8);
}
/**
 * @brief 将当前位置设为电机零位(此指令只是在 RAM 中修改，还需配合 `conf write` 指令保存到 flash 中)
 * @param id 电机ID
 */
void rezero_pos(CAN_HandleTypeDef *hcan, uint8_t id)
{
    uint8_t tdata[] = {0x40, 0x01, 0x04, 0x64, 0x20, 0x63, 0x0a};

    can_send(hcan, 0x8000 | id, tdata, sizeof(tdata));
    HAL_Delay(1000);  // 建议延时1s

    conf_write(hcan, id);  // 保存设置
}
/**
 * @brief 将电机 RAM 中设置保存到 flash 中(使用此指令后建议给电机重新上电)
 * @param id 电机ID
 */
void conf_write(CAN_HandleTypeDef *hcan, uint8_t id)
{
    uint8_t tdata[] = {0x05, 0xb3, 0x02, 0x00, 0x00};

    can_send(hcan, 0x8000 | id, tdata, sizeof(tdata));
}
/**
 * @brief 周期返回电机位置、速度、力矩数据(返回数据格式和使用 0x17，0x01 指令获取的格式一样)
 * @param id 电机ID
 * @param t 返回周期（单位：ms）
 */
void timed_return_motor_status(CAN_HandleTypeDef *hcan, uint8_t id, int16_t t_ms)
{
    uint8_t tdata[] = {0x05, 0xb4, 0x02, 0x00, 0x00};

    *(int16_t *)&tdata[3] = t_ms;

    can_send(hcan, 0x8000 | id, tdata, sizeof(tdata));
}
/**
 * @brief 电机停止，注意：需让电机停止后再重置零位，否则无效
 * @param fdcanHandle &hfdcanx
 * @param motor id 电机ID
 */
void set_motor_stop(CAN_HandleTypeDef *fdcanHandle, uint8_t id)
{
    static uint8_t cmd[] = {0x01, 0x00, 0x00};

    can_send(fdcanHandle, 0x8000 | id, cmd, sizeof(cmd));
}
/**
 * @brief 电机刹车
 * @param fdcanHandle &hfdcanx
 * @param motor id 电机ID
 */
void set_motor_brake(CAN_HandleTypeDef *fdcanHandle, uint8_t id)
{
    static uint8_t cmd[] = {0x01, 0x00, 0x0f};

    can_send(fdcanHandle, 0x8000 | id, cmd, sizeof(cmd));
}
/**
 * @brief 读取电机位置、速度、力矩指令
 * @param id 电机ID
 */
void motor_read(CAN_HandleTypeDef *hcan, uint8_t id)
{
    static uint8_t tdata[8] = {0x17, 0x01};

    can_send(hcan, 0x8000 | id, tdata, sizeof(tdata));
}
