/**
  ******************************************************************************
  * @file    control_task.c
  * @brief   ��������,����������̨�ͷ������
  ******************************************************************************
  */
#include "control_task.h"
#include "gimbal_motor.h"
#include "imu_sensor.h"
#include "gimbal_yaw.h"
#include "chassis.h"
#include "Judge.h"

void StartCtrlTask(void const * argument)
{
	Gimbal_Yaw_Init(&Gimbal_Yaw);
	for(;;)
	{
		Gimbal_Yaw.update(&Gimbal_Yaw);
		Chassis_ctrl();
		osDelay(1);
	}
}
