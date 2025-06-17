#ifndef __BSP_CAN_H__
#define __BSP_CAN_H__

/* NAN 表示在电机控制指令中，表示无限 */
#define  INI8_NAN   0x80
#define  INT16_NAN  0x8000
#define  INT32_NAN  0x80000000

typedef struct
{
    uint32_t id;
    float position;
    float velocity;
    float torque;
} motor_state_s;

typedef struct
{
    union
    {
        motor_state_s motor;
        uint8_t data[16];
    }motor_data;
} motor_state_t;

extern motor_state_t motor_state;
extern uint8_t motor_read_flag;

void CAN_Init(CAN_HandleTypeDef *hcan);
uint8_t can_send(CAN_HandleTypeDef* hcan,uint16_t id,uint8_t *msg,uint16_t len);
void can_filter_init(CAN_HandleTypeDef *hcan);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
void motor_control_tqe(CAN_HandleTypeDef *hcan, uint8_t id, int32_t tqe);
void motor_control_vel(CAN_HandleTypeDef *hcan, uint8_t id, int32_t vel, int16_t tqe);
void motor_angleControl(CAN_HandleTypeDef *hcan, uint8_t id, int32_t angle, uint16_t speed);
void motor_angleControl1(CAN_HandleTypeDef *hcan, uint8_t id, uint32_t angle, uint8_t spinDirection, uint16_t speed);
void motor_angleControl2(CAN_HandleTypeDef *hcan, uint8_t id, int32_t anglelncrement, uint32_t speed);

#endif
