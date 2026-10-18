/**
  ******************************************************************************
  * @file    control_task.c
  * @brief   
  ******************************************************************************
  */
#include "control_task.h"
#include "gimbal_yaw.h"
#include "communicate.h"

void StartControlTask(void const * argument)
{
	Gimbal_Yaw_Init(&Gimbal_Yaw);
	for(;;) 
	{
		if ((imu_sensor.work_state.err_code==IMU_NONE_ERR)||
			(imu_sensor.work_state.err_code==IMU_DATA_CALI))
			imu_sensor.update(&imu_sensor);
		Gimbal_Yaw.update(&Gimbal_Yaw);
		C_Board_Send();
		osDelay(1);
	}
}



