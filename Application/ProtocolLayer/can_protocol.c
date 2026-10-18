#include "can_protocol.h"
#include "cap_protocol.h"
#include "Chassis_Motor.h"
#include "communicate.h"
#include "judge.h"
#include "cap.h"

/**
 *  @brief  CAN1 接收数据
 */
void CAN1_rxDataHandler(uint32_t rxId, uint8_t *rxBuf)
{
	switch (rxId)
	{
        case 0x201: R_F_Wheel.rx(&R_F_Wheel,rxBuf); break; //右前反馈
        case 0x202: R_B_Wheel.rx(&R_B_Wheel,rxBuf); break; //右后反馈
        case 0x203: L_F_Wheel.rx(&L_F_Wheel,rxBuf); break; //左前反馈
        case 0x204: L_B_Wheel.rx(&L_B_Wheel,rxBuf); break; //左后反馈
		default: break;
	}
}

/**
 *  @brief  CAN2 接收数据
 */
void CAN2_rxDataHandler(uint32_t rxId, uint8_t *rxBuf)
{
	switch (rxId)
	{
		case 0x012: Yaw_Motor.rx(&Yaw_Motor,rxBuf); break;
		case 0x0C1: Board_Rx_01(rxBuf); break;
		default: break;
	}
}

/**
 *  @brief  CAN3 接收数据
 */
void CAN3_rxDataHandler(uint32_t rxId, uint8_t *rxBuf)
{
	switch (rxId)
	{
		default: break;
	}
}
