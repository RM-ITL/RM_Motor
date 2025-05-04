#ifndef __CAN1_H__
#define __CAN1_H__

//高擎历程
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

uint8_t can_send(CAN_HandleTypeDef* hcan,uint16_t id,uint8_t *msg,uint16_t len);
void can_filter_init(CAN_HandleTypeDef *hcan);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
void motor_control_pos(CAN_HandleTypeDef *hcan, uint8_t id, int32_t pos, int16_t tqe);
void motor_control_vel(CAN_HandleTypeDef *hcan, uint8_t id, int16_t vel, int16_t tqe);
void motor_control_tqe(CAN_HandleTypeDef *hcan, uint8_t id, int32_t tqe);
void motor_control_pos_vel_tqe(CAN_HandleTypeDef *hcan, uint8_t id, int16_t pos, int16_t vel, int16_t tqe);
void rezero_pos(CAN_HandleTypeDef *hcan, uint8_t id);
void conf_write(CAN_HandleTypeDef *hcan, uint8_t id);
void timed_return_motor_status(CAN_HandleTypeDef *hcanx, uint8_t id, int16_t t_ms);
void set_motor_stop(CAN_HandleTypeDef *fdcanHandle, uint8_t id);
void set_motor_brake(CAN_HandleTypeDef *fdcanHandle, uint8_t id);
void motor_read(CAN_HandleTypeDef *hcanx, uint8_t id);

#endif
