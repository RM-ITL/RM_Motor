#include "DM_Motor.h"

//电机参数
motor_t MOTOR;
/**
 * 将浮点数转换为无符号整数，基于给定的范围和位数。
 *
 * 参数:
 *   x_float: 要转换的浮点数值
 *   x_min:   输入范围的最小值
 *   x_max:   输入范围的最大值
 *   bits:    目标整数的位数（必须大于等于1，且bits位数对应的范围不超过整型限制）
 *
 * 返回值:
 *   转换后的无符号整数值，范围在0到(1<<bits)-1之间。若输入超出范围可能导致结果溢出。
 */
int float_to_uint(float x_float, float x_min, float x_max, int bits)
{
    /* Converts a float to an unsigned int, given range and number of bits */
    float span = x_max - x_min;
    float offset = x_min;
    return (int) ((x_float-offset)*((float)((1<<bits)-1))/span);
}
/**
 * 将无符号整数转换为指定范围内的浮点数
 * 
 * 参数说明：
 * x_int  - 输入的无符号整数（0 <= x_int < (1<<bits)）
 * x_min  - 输出浮点数的最小值（对应x_int=0时的输出）
 * x_max  - 输出浮点数的最大值（对应x_int=(1<<bits)-1时的输出）
 * bits   - 量化使用的位数（决定输入整数的精度）
 * 
 * 返回值：
 * 转换后的浮点数，均匀分布在[x_min, x_max]区间内
 */
float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    /* converts unsigned int to float, given range and number of bits */
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}
//电机初始化
void DM_Motor_Init(void)
{
    MOTOR.PMAX = 12.5f;
    MOTOR.TMAX = 10.0f;
    MOTOR.VMAX = 30.0f;
}
//电机使能
void motor_enble(FDCAN_HandleTypeDef *hfdcan, uint8_t id)
{
    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};

    fdcan_send(hfdcan, id, data, 8);
}
//电机失能
void motor_disable(FDCAN_HandleTypeDef *hfdcan, uint8_t id)
{
    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};

    fdcan_send(hfdcan, id, data, 8);
}
/**
 * @brief 发送MIT模式控制指令到指定CAN节点
 * 
 * 该函数将位置、速度、力控参数打包为CAN帧格式并发送，用于控制支持MIT协议的电机驱动器。
 * 参数范围需符合电机驱动器规格，超出范围的值会被裁剪。
 * 
 * @param hfdcan FDCAN句柄指针
 * @param id CAN节点ID（0x00-0x0F）
 * @param pos 目标位置(rad)，范围[-MOTOR.PMAX, MOTOR.PMAX]
 * @param vel 目标速度(rad/s)，范围[-MOTOR.VMAX, MOTOR.VMAX]
 * @param kp 位置环增益，范围[KP_MIN, KP_MAX]
 * @param kd 微分增益，范围[KD_MIN, KD_MAX]
 * @param tor 输出扭矩限制(Nm)，范围[-MOTOR.TMAX, MOTOR.TMAX]
 * 
 * @return 无
 */
void Mit_mode(FDCAN_HandleTypeDef *hfdcan, uint8_t id,float pos, float vel,float kp, float kd, float tor)
{
    uint8_t data[8];
    uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;

    pos_tmp = float_to_uint(pos, -MOTOR.PMAX, MOTOR.PMAX, 16);
    vel_tmp = float_to_uint(vel, -MOTOR.VMAX, MOTOR.VMAX, 12);
    tor_tmp = float_to_uint(tor, -MOTOR.TMAX, MOTOR.TMAX, 12);
    kp_tmp  = float_to_uint(kp,  KP_MIN, KP_MAX, 12);
    kd_tmp  = float_to_uint(kd,  KD_MIN, KD_MAX, 12);

    data[0] = (pos_tmp >> 8);
    data[1] = pos_tmp;
    data[2] = (vel_tmp >> 4);
    data[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>>8);
    data[4] = kp_tmp;
    data[5] = (kd_tmp >> 4);
    data[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>>8);
    data[7] = tor_tmp;

    fdcan_send(hfdcan, 0x00 | id, data, 8);
}
/**
 * @brief  通过FDCAN总线发送位置和速度控制模式数据
 * @param  hfdcan：FDCAN外设句柄指针
 * @param  id：CAN消息标识符低4位（扩展帧）
 * @param  pos：位置目标值（float类型32位存储格式）
 * @param  vel：速度目标值（float类型32位存储格式）
 * @retval 无
 */
void Pos_mode(FDCAN_HandleTypeDef *hfdcan, uint8_t id,float pos, float vel)
{
    uint8_t *pbuf, *vbuf;
	uint8_t data[8];
	
	pbuf=(uint8_t*)&pos;
	vbuf=(uint8_t*)&vel;
	
	data[0] = *pbuf;
	data[1] = *(pbuf+1);
	data[2] = *(pbuf+2);
	data[3] = *(pbuf+3);

	data[4] = *vbuf;
	data[5] = *(vbuf+1);
	data[6] = *(vbuf+2);
	data[7] = *(vbuf+3);

    fdcan_send(hfdcan, 0x100 | id, data, 8);
}
/**
 * @brief  发送速度模式指令到指定的FDCAN节点
 * @param  hfdcan FDCAN外设句柄指针
 * @param  id 目标节点ID（0-255）
 * @param  vel 浮点型速度值（单位由应用层定义）
 * @retval None
 */
void Spd_mode(FDCAN_HandleTypeDef *hfdcan, uint8_t id, float vel)
{
    uint8_t *vbuf;
    uint8_t data[4];
    vbuf=(uint8_t*)&vel;
    
    data[0] = *vbuf;
    data[1] = *(vbuf+1);
    data[2] = *(vbuf+2);
    data[3] = *(vbuf+3);

    fdcan_send(hfdcan, 0x200 | id, data, 4);
}
/**
  * @brief  通过CAN总线发送指定ID设备的力位混控模式数据
  * @param  hfdcan: FDCAN句柄指针
  * @param  id: 设备ID（0-127）
  * @param  pos: 位置值（float类型）
  * @param  vel: 速度值（float类型，单位0.01 LSB）
  * @param  cur: 电流值（float类型，单位0.0001 LSB）
  * @retval 无
  */
void Psi_mode(FDCAN_HandleTypeDef *hfdcan, uint8_t id, float pos, float vel, float cur)
{
    uint8_t *pbuf, *vbuf, *ibuf;
    uint8_t data[8];
    
    uint16_t u16_vel = vel*100;
    uint16_t u16_cur  = cur*10000;
    
    pbuf=(uint8_t*)&pos;
    vbuf=(uint8_t*)&u16_vel;
    ibuf=(uint8_t*)&u16_cur;
    
    data[0] = *pbuf;
    data[1] = *(pbuf+1);
    data[2] = *(pbuf+2);
    data[3] = *(pbuf+3);

    data[4] = *vbuf;
    data[5] = *(vbuf+1);
    
    data[6] = *ibuf;
    data[7] = *(ibuf+1);

    fdcan_send(hfdcan, 0x300 | id, data, 8);
}

