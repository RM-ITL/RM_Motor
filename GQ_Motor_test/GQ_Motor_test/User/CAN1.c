#include "stm32f4xx.h"                  // Device header
#include "gpio.h"
#include "can.h"
#include "CAN1.h"
#include "string.h"

/**
 * @brief ��ʼ��CAN����
 *
 * @param hcan CAN���
 * @param Callback_Function ����ص�����
 */
void CAN_Init(CAN_HandleTypeDef *hcan)
{
  HAL_CAN_Start(hcan);
  __HAL_CAN_ENABLE_IT(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
  __HAL_CAN_ENABLE_IT(hcan, CAN_IT_RX_FIFO1_MSG_PENDING);
}


//��������
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
 * @brief λ�ÿ���
 * @param id  ���ID
 * @param pos λ�ã���λ 0.0001 Ȧ���� pos = 5000 ��ʾת�� 0.5 Ȧ��λ�á�
 * @param ���أ���λ��0.01 NM���� torque = 110 ��ʾ�������Ϊ 1.1NM
 */
void motor_control_pos(CAN_HandleTypeDef *hcan, uint8_t id, int32_t pos, int16_t tqe)
{
    uint8_t tdata[8] = {0x07, 0x07, 0x0A, 0x05, 0x00, 0x00, 0x80, 0x00};

    *(int16_t *)&tdata[2] = pos;
    *(int16_t *)&tdata[6] = tqe;

    can_send(hcan, 0x8000 | id, tdata, 8);
}
/**
 * @brief �ٶȿ���
 * @param id ���ID
 * @param vel �ٶȣ���λ 0.00025 ת/�룬�� val = 1000 ��ʾ 0.25 ת/��
 * @param tqe ���أ���λ��0.01 NM���� torque = 110 ��ʾ�������Ϊ 1.1NM
 */
void motor_control_vel(CAN_HandleTypeDef *hcan, uint8_t id, int16_t vel, int16_t tqe)
{
    uint8_t tdata[8] = {0x07, 0x07, 0x00, 0x80, 0x20, 0x00, 0x80, 0x00};

    *(int16_t *)&tdata[4] = vel;
    *(int16_t *)&tdata[6] = tqe;

    can_send(hcan, 0x8000 | id, tdata, 8);
}
/**
 * @brief ����ģʽ
 * @param id ���ID
 * @param tqe ���أ���λ��0.01 NM���� torque = 110 ��ʾ�������Ϊ 1.1NM
 */
void motor_control_tqe(CAN_HandleTypeDef *hcan, uint8_t id, int32_t tqe)
{
    uint8_t tdata[8] = {0x05, 0x13, 0x00, 0x80, 0x20, 0x00, 0x80, 0x00};

    *(int16_t *)&tdata[2] = tqe;

    can_send(hcan, 0x8000 | id, tdata, 4);
}
/**
 * @brief ���λ��-�ٶ�-������ؿ��ƣ�int16��
 * @param id  ���ID
 * @param pos λ�ã���λ 0.0001 Ȧ���� pos = 5000 ��ʾת�� 0.5 Ȧ��λ�á�
 * @param vel �ٶȣ���λ 0.00025 ת/�룬�� val = 1000 ��ʾ 0.25 ת/��
 * @param tqe ������أ���λ��0.01 NM���� torque = 110 ��ʾ�������Ϊ 1.1NM
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
 * @brief ����ǰλ����Ϊ�����λ(��ָ��ֻ���� RAM ���޸ģ�������� `conf write` ָ��浽 flash ��)
 * @param id ���ID
 */
void rezero_pos(CAN_HandleTypeDef *hcan, uint8_t id)
{
    uint8_t tdata[] = {0x40, 0x01, 0x04, 0x64, 0x20, 0x63, 0x0a};

    can_send(hcan, 0x8000 | id, tdata, sizeof(tdata));
    HAL_Delay(1000);  // ������ʱ1s

    conf_write(hcan, id);  // ��������
}
/**
 * @brief ����� RAM �����ñ��浽 flash ��(ʹ�ô�ָ��������������ϵ�)
 * @param id ���ID
 */
void conf_write(CAN_HandleTypeDef *hcan, uint8_t id)
{
    uint8_t tdata[] = {0x05, 0xb3, 0x02, 0x00, 0x00};

    can_send(hcan, 0x8000 | id, tdata, sizeof(tdata));
}
/**
 * @brief ���ڷ��ص��λ�á��ٶȡ���������(�������ݸ�ʽ��ʹ�� 0x17��0x01 ָ���ȡ�ĸ�ʽһ��)
 * @param id ���ID
 * @param t �������ڣ���λ��ms��
 */
void timed_return_motor_status(CAN_HandleTypeDef *hcan, uint8_t id, int16_t t_ms)
{
    uint8_t tdata[] = {0x05, 0xb4, 0x02, 0x00, 0x00};

    *(int16_t *)&tdata[3] = t_ms;

    can_send(hcan, 0x8000 | id, tdata, sizeof(tdata));
}
/**
 * @brief ���ֹͣ��ע�⣺���õ��ֹͣ����������λ��������Ч
 * @param fdcanHandle &hfdcanx
 * @param motor id ���ID
 */
void set_motor_stop(CAN_HandleTypeDef *fdcanHandle, uint8_t id)
{
    static uint8_t cmd[] = {0x01, 0x00, 0x00};

    can_send(fdcanHandle, 0x8000 | id, cmd, sizeof(cmd));
}
/**
 * @brief ���ɲ��
 * @param fdcanHandle &hfdcanx
 * @param motor id ���ID
 */
void set_motor_brake(CAN_HandleTypeDef *fdcanHandle, uint8_t id)
{
    static uint8_t cmd[] = {0x01, 0x00, 0x0f};

    can_send(fdcanHandle, 0x8000 | id, cmd, sizeof(cmd));
}
/**
 * @brief ��ȡ���λ�á��ٶȡ�����ָ��
 * @param id ���ID
 */
void motor_read(CAN_HandleTypeDef *hcan, uint8_t id)
{
    static uint8_t tdata[8] = {0x17, 0x01};

    can_send(hcan, 0x8000 | id, tdata, sizeof(tdata));
}
