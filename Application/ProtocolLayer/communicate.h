#ifndef __COMMUNICATE_H
#define __COMMUNICATE_H

#include "rp_math.h"

#define ID_BOARD_TX1 0x0C1   // 云台反馈
#define ID_BOARD_TX2 0x0C2   // 保留
#define ID_BOARD_RX1 0x0D1   // 云台模式+状态
#define ID_BOARD_RX2 0x0D2   // 云台目标角

typedef struct {
    float yaw_mec;           // 云台电机相对角度(rad)
    float yaw_imu;           // IMU 绝对 Yaw(rad)
} C_Board_Tx_t;

typedef struct {
    uint8_t gimbal_mode;     // 0->机械 1->陀螺 2->睡眠
    uint8_t rc_online;       // 遥控器状态
    float yaw_mec_tar;       // 机械模式目标(rad)
    float pitch_mec_tar;     // Pitch 目标(rad)
    uint16_t heartbeat;      // 跨板通信心跳
} C_Board_Rx_t;

extern C_Board_Tx_t C_Board_Tx;
extern C_Board_Rx_t C_Board_Rx;

void C_Board_Send(void);
void C_Board_Rx_01(uint8_t *rxbuf);
void C_Board_Rx_02(uint8_t *rxbuf);

#endif
