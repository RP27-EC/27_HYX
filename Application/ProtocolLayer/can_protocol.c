#include "can_protocol.h"
#include "communicate.h"

/**
 *  @brief  CAN1 接收数据
 */
void CAN1_rxDataHandler(uint32_t rxId, uint8_t *rxBuf)
{
	switch (rxId)
	{
		case 0x00B: L_Wheel.rx(&L_Wheel,rxBuf); break;
		case 0x205: R_Fric.rx(&R_Fric,rxBuf);   break;
		default: break;
	}
}
/**
 *  @brief  CAN2 接收数据
 */
void CAN2_rxDataHandler(uint32_t canId, uint8_t *rxBuf)
{
	
	switch (canId)
	{
		case 0x012: Yaw_Motor.rx(&Yaw_Motor,rxBuf); break;
		case 0x0D1: C_Board_Rx_01(rxBuf); break;
		case 0x0D2: C_Board_Rx_02(rxBuf); break;
		default: break;
	}
}
