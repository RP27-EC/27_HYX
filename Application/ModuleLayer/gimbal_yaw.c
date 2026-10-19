#include "gimbal_yaw.h"
#include "communicate.h"
#include "pid.h"
#include "motor.h"
#include "imu_sensor.h"

static void  Gimbal_Yaw_Update(Gimbal_Yaw_t *Gimbal_Yaw);
static float Wrap180(float angle);

Gimbal_Yaw_t Gimbal_Yaw;
static uint8_t last_mode=0xFF; 	//识别模式切换  
static float gyro_ofs=0.0f;

static float Wrap180(float angle)
{
    while(angle> 180.0f) angle-=360.0f;
    while(angle<-180.0f) angle+=360.0f;
    return angle;
}

void Gimbal_Yaw_Init(Gimbal_Yaw_t *Gimbal_Yaw)
{
    Yaw_Motor.single_init(&Yaw_Motor);
	Gimbal_Yaw->update=Gimbal_Yaw_Update;
    //位置外环
    Gimbal_Yaw->yaw_outer.kp=MEC_OUTER_KP;
    Gimbal_Yaw->yaw_outer.ki=
    Gimbal_Yaw->yaw_outer.kd=
    Gimbal_Yaw->yaw_outer.integral_max=0.0f;
    Gimbal_Yaw->yaw_outer.out_max=500.0f;
    Gimbal_Yaw->yaw_outer.a=1;
    //速度内环
    Gimbal_Yaw->yaw_inner.kp=MEC_INNER_KP;
    Gimbal_Yaw->yaw_inner.ki=
    Gimbal_Yaw->yaw_inner.kd=
    Gimbal_Yaw->yaw_inner.integral_max=0.0f;
    Gimbal_Yaw->yaw_inner.out_max=10.0f;
    Gimbal_Yaw->yaw_inner.a=1;
    C_Board_Rx.heartbeat=200;
    last_mode=0xFF;
	//单发力矩
    Yaw_Motor.tx_info->Kp=
    Yaw_Motor.tx_info->Kd=
    Yaw_Motor.tx_info->target_angle=
    Yaw_Motor.tx_info->target_speed=
    Yaw_Motor.tx_info->torque=0.0f;
	Yaw_Motor.single_set_torque(&Yaw_Motor);
}

static void Gimbal_Yaw_Update(Gimbal_Yaw_t *gimbal)
{
    float dead_deg;
	uint8_t mode=YAW_MEC;
	// YAW 保护
    if(Yaw_Motor.state->status!=DEV_ONLINE||
       C_Board_Rx.heartbeat>100           ||
       C_Board_Rx.gimbal_mode==YAW_SLEEP)
	{
        pid_clear(&gimbal->yaw_outer);
        pid_clear(&gimbal->yaw_inner);
        last_mode=0xFF;
        Yaw_Motor.tx_info->Kp=
        Yaw_Motor.tx_info->Kd=
        Yaw_Motor.tx_info->target_angle=
        Yaw_Motor.tx_info->target_speed=
        Yaw_Motor.tx_info->torque=0.0f;
        Yaw_Motor.single_set_torque(&Yaw_Motor);
        C_Board_Rx.heartbeat++;
        return;
    }
	//必须陀螺仪零偏校准完才能进陀螺仪模式
    if(C_Board_Rx.gimbal_mode==YAW_IMU&& 
	   imu_sensor.work_state.cali_end)
        mode=YAW_IMU;
    if(mode==YAW_IMU)
    {
         //切陀螺仪模式->进场对齐
        if(last_mode!=YAW_IMU)
            gyro_ofs=Wrap180(imu_sensor.info->base_info.yaw-C_Board_Rx.yaw_mec_tar*RAD2DEG);
        gimbal->target_angle=Wrap180(C_Board_Rx.yaw_mec_tar*RAD2DEG+gyro_ofs);
        gimbal->current_angle=imu_sensor.info->base_info.yaw;      
        gimbal->ang_vel=imu_sensor.info->base_info.ave_rate_yaw;     
        gimbal->yaw_outer.kp=GYRO_OUTER_KP;
        gimbal->yaw_inner.kp=GYRO_INNER_KP;
        dead_deg=GYRO_DEAD_DEG;
    }
    else
    {
        gimbal->target_angle=C_Board_Rx.yaw_mec_tar*RAD2DEG;
        gimbal->current_angle=Yaw_Motor.rx_info->motor_angle*RAD2DEG;
        gimbal->ang_vel=imu_sensor.info->base_info.ave_rate_yaw;
        gimbal->yaw_outer.kp=MEC_OUTER_KP;
        gimbal->yaw_inner.kp=MEC_INNER_KP;
        dead_deg=YAW_DEAD_DEG;
    }
	//位置外环
    gimbal->yaw_outer.target =gimbal->target_angle;
    gimbal->yaw_outer.measure=gimbal->current_angle;
    pid_err_cal(&gimbal->yaw_outer);
    while(gimbal->yaw_outer.err> 180.0f) gimbal->yaw_outer.err-=360.0f;
    while(gimbal->yaw_outer.err<-180.0f) gimbal->yaw_outer.err+=360.0f;
    if (fabsf(gimbal->yaw_outer.err)<dead_deg)
	{
		gimbal->yaw_inner.target=0.0f;
	}
    else
    {
        single_pid_ctrl(&gimbal->yaw_outer);
        gimbal->yaw_inner.target=gimbal->yaw_outer.out;
    }
	//速度内环
    gimbal->yaw_inner.measure=gimbal->ang_vel;
    pid_err_cal(&gimbal->yaw_inner);
    single_pid_ctrl(&gimbal->yaw_inner);
    gimbal->output_torque=gimbal->yaw_inner.out;
	//单发力矩
    Yaw_Motor.tx_info->Kp=
    Yaw_Motor.tx_info->Kd=
    Yaw_Motor.tx_info->target_angle=
    Yaw_Motor.tx_info->target_speed=0.0f;
    Yaw_Motor.tx_info->torque=gimbal->output_torque;
    Yaw_Motor.single_set_torque(&Yaw_Motor);
	last_mode=mode;
    C_Board_Rx.heartbeat++;
}
