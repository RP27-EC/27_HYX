#include "gimbal_yaw.h"
#include "rc_sensor.h"
#include "gimbal_motor.h"
#include "chassis.h"
								   
Gimbal_Yaw_t Gimbal_Yaw;           
								   
static float gyro_tar_deg=0.0f;    //陀螺仪模式目标角(度) 
static uint8_t up_step_last=0;     //上一次的拨轮上拨状态
static uint8_t u_turn_hold=0;      //掉头进行标识->防自动双稳态翻回
static uint16_t u_turn_tick=0;	   //掉头超时计数

static void Yaw_Mec_Target_Config(void);
static void Yaw_Gyro_Target_Config(void);
static void Gimbal_Yaw_Update(Gimbal_Yaw_t *Gimbal_Yaw);

void Gimbal_Yaw_Init(Gimbal_Yaw_t *Gimbal_Yaw)
{
    Gimbal_Yaw->update=Gimbal_Yaw_Update;
	Gimbal_Yaw->mode=YAW_MEC;
    Gimbal_Yaw->posi=YAW_FRONT;
    gyro_tar_deg=0.0f;
    Yaw_Mec_Target_Config();
}

static void Yaw_Mec_Target_Config(void)
{
    float err;
    if(Gimbal_Yaw.posi==YAW_BACK)
        err= YAW_BACK_RAD-Board_Rx.yaw_mec;
    else
        err=YAW_FRONT_RAD-Board_Rx.yaw_mec;
    while(err> PI) err-=2*PI;
    while(err<-PI) err+=2*PI;
    if(fabsf(err)>YAW_FLIP_RAD)
        Gimbal_Yaw.posi=(Gimbal_Yaw.posi==YAW_BACK)?YAW_FRONT:YAW_BACK;
    Gimbal_Yaw.target_rad=(Gimbal_Yaw.posi==YAW_BACK)?YAW_BACK_RAD:YAW_FRONT_RAD;
}

static void Yaw_Gyro_Target_Config(void)
{
    float stick=rc_sensor.info->ch0/RC_MAX;
	stick=DeathZoom(stick,0.0f,0.1f);
    gyro_tar_deg+=-stick*GYRO_YAW_SENS*CTRL_DT;
    while(gyro_tar_deg> 180.0f) gyro_tar_deg-=360.0f;
    while(gyro_tar_deg<-180.0f) gyro_tar_deg+=360.0f;
    Gimbal_Yaw.target_rad=gyro_tar_deg*PI/180.0f;
}

static void Gimbal_Yaw_Update(Gimbal_Yaw_t *Gimbal_Yaw)
{
    if(rc_sensor.errno==DEV_ID_ERR||
	   rc_sensor.work_state==DEV_OFFLINE||
	   RC_SW1_VALUE==RC_SW_UP||Board_Rx.heartbeat>100)
         Gimbal_Yaw->mode=YAW_SLEEP;
    else if(RC_SW2_VALUE==RC_SW_UP)		//右拨上位->机械
         Gimbal_Yaw->mode=YAW_MEC;      //    中位->陀螺仪 
    else Gimbal_Yaw->mode=YAW_GYRO;     //    下位->小陀螺 
	//机械模式拨轮上拨实现掉头
	if(rc_sensor.info->thumbwheel.step[RC_TB_UP]!=up_step_last)
    {
        up_step_last=rc_sensor.info->thumbwheel.step[RC_TB_UP];
        if(Gimbal_Yaw->mode==YAW_MEC)
        {
            Gimbal_Yaw->posi=(Gimbal_Yaw->posi==YAW_BACK)?YAW_FRONT:YAW_BACK;
            u_turn_hold=1;
            u_turn_tick=0;
        }
    }
    if(Gimbal_Yaw->mode!=YAW_MEC) u_turn_hold=0;   
	switch (Gimbal_Yaw->mode)
	{
		case YAW_SLEEP:
			Gimbal_Yaw->target_rad=Board_Rx.yaw_mec;
		    Gimbal_Yaw->posi=YAW_FRONT;
			break;
		case YAW_MEC:
			if(u_turn_hold)
			{
				Gimbal_Yaw->target_rad=
				(Gimbal_Yaw->posi==YAW_BACK)?YAW_BACK_RAD:YAW_FRONT_RAD;
				{
					float u_err=Gimbal_Yaw->target_rad-Board_Rx.yaw_mec;
					while(u_err> PI) u_err-=2*PI;
					while(u_err<-PI) u_err+=2*PI;
					//掉头成功->进行标识位置零
					if(fabsf(u_err)<3.0f*PI/180.0f) u_turn_hold=0;
				}
				//过2秒掉头失败->放弃
				if(++u_turn_tick>2000) u_turn_hold=0;               
			}
			else
			{
				Yaw_Mec_Target_Config();
			}
			break;
		case YAW_GYRO:
			Yaw_Gyro_Target_Config();
			Gimbal_Yaw->posi=YAW_FRONT;
			break;
		default:
			break;
	}
	Board_Rx.heartbeat++;
    Board_Tx.gimbal_mode=(uint8_t)Gimbal_Yaw->mode;
    Board_Tx.rc_online=(rc_sensor.errno==NONE_ERR)?1:0;
    Board_Tx.yaw_mec_tar=Gimbal_Yaw->target_rad;
    Board_Send_To_C();
}
