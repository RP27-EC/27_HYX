#include "communicate.h"
#include "imu_sensor.h"
#include "gimbal_yaw.h"
#include "can.h"
#include "motor.h"

C_Board_Tx_t C_Board_Tx;
C_Board_Rx_t C_Board_Rx;

static uint8_t tx_buf[8];

static uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits);
static float uint_to_float(uint16_t x_int, float x_min, float x_max, uint8_t bits);

//发送云台当前角度
void C_Board_Send(void)
{
    C_Board_Tx.yaw_mec=Yaw_Motor.rx_info->motor_angle;
	C_Board_Tx.yaw_imu=imu_sensor.info->base_info.yaw*DEG2RAD;  
    uint16_t t1=float_to_uint(C_Board_Tx.yaw_mec,-PI,PI,16);
    uint16_t t2=float_to_uint(C_Board_Tx.yaw_imu,-PI,PI,16);
    tx_buf[0]=(t1>>8)&0xFF;
    tx_buf[1]=t1&0xFF;
    tx_buf[2]=(t2>>8)&0xFF;
    tx_buf[3]=t2&0xFF;
    CAN_SendData(&hcan2,ID_BOARD_TX1,tx_buf);
}
//接收云台模式和遥控器状态
void C_Board_Rx_01(uint8_t *rxbuf)
{
    C_Board_Rx.gimbal_mode=rxbuf[0];
    C_Board_Rx.rc_online=rxbuf[1];
    C_Board_Rx.heartbeat=0;
}
//接收目标角度
void C_Board_Rx_02(uint8_t *rxbuf)
{
    uint16_t t1=((uint16_t)rxbuf[0]<<8)|rxbuf[1];
    uint16_t t2=((uint16_t)rxbuf[2]<<8)|rxbuf[3];
    C_Board_Rx.yaw_mec_tar=uint_to_float(t1,-PI,PI,16);
    C_Board_Rx.pitch_mec_tar=uint_to_float(t2,-PI,PI,16);
    C_Board_Rx.heartbeat=0;
}

static uint16_t float_to_uint(float x,float x_min,float x_max,uint8_t bits)
{
    float span=x_max-x_min;
    float offset=x_min;
    return (uint16_t)((x-offset)*((float)((1<<bits)-1))/span);
}

static float uint_to_float(uint16_t x_int,float x_min,float x_max,uint8_t bits)
{
    float span=x_max-x_min;
    float offset=x_min;
    return((float)x_int)*span/((float)((1<<bits)-1))+offset;
}
