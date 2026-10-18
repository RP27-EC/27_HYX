#ifndef __GIMBAL_YAW_H
#define __GIMBAL_YAW_H

#include "pid.h"

#define RAD2DEG          (180.0f/PI)   //弧度转度
#define DEG2RAD          (PI/180.0f)   //度转弧度
#define YAW_MEC          0		       //机械模式
#define YAW_IMU          1		       //陀螺仪模式
#define YAW_SLEEP        2		       //关控模式
#define YAW_DEAD_DEG     0.2f		   //机械模式死区角
#define GYRO_DEAD_DEG    1.0f		   //陀螺仪模式死区角
#define GYRO_OUTER_KP    40.0f
#define GYRO_INNER_KP    0.025f
#define MEC_OUTER_KP     15.0f	   
#define MEC_INNER_KP     0.12f	

typedef struct Gimbal_Yaw_struct_t
{
    float current_angle;               //电机相对角度(度)
    float ang_vel;                     //角速度(度/秒)
    float target_angle;                //目标角度(度)
    float output_torque;               //输出力矩
    pid_ctrl_t yaw_outer;              //位置环
    pid_ctrl_t yaw_inner;              //速度环
    void (*update)(struct Gimbal_Yaw_struct_t *gimbal);
} Gimbal_Yaw_t;

extern Gimbal_Yaw_t Gimbal_Yaw;

void Gimbal_Yaw_Init(Gimbal_Yaw_t *Gimbal_Yaw);

#endif
