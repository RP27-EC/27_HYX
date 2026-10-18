#include "communicate.h"
#include "gimbal_motor.h"

Board_Tx_t Board_Tx;
Board_Rx_t Board_Rx;

static uint8_t tx_buf1[8];
static uint8_t tx_buf2[8];

static uint16_t float_to_uint(float x,float x_min,float x_max,uint8_t bits);
static float    uint_to_float(uint16_t x_int,float x_min,float x_max,uint8_t bits);

void Board_Send_To_C(void)
{
    //发模式和状态
    tx_buf1[0]=Board_Tx.gimbal_mode;
    tx_buf1[1]=Board_Tx.rc_online;
    CAN_SendData(&hfdcan2,ID_BOARD_TX1,tx_buf1);
    //发目标值
    uint16_t t1=float_to_uint(Board_Tx.yaw_mec_tar,-PI,PI,16);
    uint16_t t2=float_to_uint(Board_Tx.pitch_mec_tar,-PI,PI,16);
    tx_buf2[0]=(t1>>8)&0xFF;
    tx_buf2[1]=t1&0xFF;
    tx_buf2[2]=(t2>>8)&0xFF;
    tx_buf2[3]=t2&0xFF;
    CAN_SendData(&hfdcan2,ID_BOARD_TX2,tx_buf2);
}

void Board_Rx_01(uint8_t *rxbuf)
{
	Board_Rx.yaw_imu=0;
	Board_Rx.heartbeat=0;
    uint16_t t1=((uint16_t)rxbuf[0]<<8)|rxbuf[1];
    Board_Rx.yaw_mec=uint_to_float(t1,-PI,PI,16);
}

void Board_Rx_02(uint8_t *rxbuf)
{
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
