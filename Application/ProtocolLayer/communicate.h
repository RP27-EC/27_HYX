#ifndef __COMMUNICATE_H
#define __COMMUNICATE_H

#include "rp_math.h"

#define ID_BOARD_TX1 0xD1
#define ID_BOARD_TX2 0xD2
#define ID_BOARD_RX1 0xC1
#define ID_BOARD_RX2 0xC2

typedef struct {
    uint8_t gimbal_mode;  // 0->机械 1->陀螺 2->睡眠
    uint8_t rc_online;    // 遥控器状态
    float yaw_mec_tar;    // 机械模式目标角(rad)
    float pitch_mec_tar;  // Pitch 目标角(rad)
}Board_Tx_t;

typedef struct {
    float yaw_mec;        // 云台电机相对角度(rad)
    float yaw_imu;        // IMU 绝对 Yaw (rad)
	uint16_t heartbeat;
} Board_Rx_t;

extern Board_Tx_t Board_Tx;
extern Board_Rx_t Board_Rx;

void Board_Send_To_C(void);
void Board_Rx_01(uint8_t *rxbuf);
void Board_Rx_02(uint8_t *rxbuf);

#endif
