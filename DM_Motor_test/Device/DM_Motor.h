#ifndef __DM_MOTOR_H__
#define __DM_MOTOR_H__
#include "stm32h7xx.h"
#include "bsp_can.h"

//电机KP,KD范围
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
// 电机参数设置结构体
typedef struct
{
    float    PMAX;		// 位置映射范围
    float    VMAX;		// 速度映射范围
    float    TMAX;		// 扭矩映射范围
} motor_t;

int float_to_uint(float x_float, float x_min, float x_max, int bits);
float uint_to_float(int x_int, float x_min, float x_max, int bits);
void DM_Motor_Init(void);
void motor_enble(FDCAN_HandleTypeDef *hfdcan, uint8_t id);
void motor_disable(FDCAN_HandleTypeDef *hfdcan, uint8_t id);
void Mit_mode(FDCAN_HandleTypeDef *hfdcan, uint8_t id,float pos, float vel,float kp, float kd, float tor);
void Pos_mode(FDCAN_HandleTypeDef *hfdcan, uint8_t id,float pos, float vel);
void Spd_mode(FDCAN_HandleTypeDef *hfdcan, uint8_t id, float vel);
void Psi_mode(FDCAN_HandleTypeDef *hfdcan, uint8_t id, float pos, float vel, float cur);
#endif
