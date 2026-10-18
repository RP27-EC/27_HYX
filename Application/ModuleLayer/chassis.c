#include "chassis.h"
#include "rc_sensor.h"
#include "rp_math.h"
#include "Chassis_motor.h"
#include "gimbal_yaw.h"									  

static float fol_tgt_deg=FOLLOW_TARGET_DEG;   //当前对齐位
static float follow_wz=0.0f; 				  //陀螺仪底盘跟随输出
static float spin_wz=0.0f;  			      //小陀螺自旋输出

static void  chassis_vector_to_rpm(float vx,float vy,float wz,float rpm[4]);
static float chassis_follow_gimbal(void);
static float chassis_spin(void);

//底盘运动学逆解算
static void chassis_vector_to_rpm(float vx,float vy,float wz,float rpm[4])
{
    float v1=-vx-vy+wz;
    float v2=vx-vy+wz;
    float v3=-vx+vy+wz;
    float v4=vx+vy+wz;
	rpm[0]=v1*CHASSIS_REDUCTION*30.0f/(PI*R_CHASSIS_WHEEL);
	rpm[1]=v2*CHASSIS_REDUCTION*30.0f/(PI*R_CHASSIS_WHEEL);
	rpm[2]=v3*CHASSIS_REDUCTION*30.0f/(PI*R_CHASSIS_WHEEL);
	rpm[3]=v4*CHASSIS_REDUCTION*30.0f/(PI*R_CHASSIS_WHEEL);
}

//陀螺仪模式底盘自转量计算
static float chassis_follow_gimbal(void)
{
    float rel=Board_Rx.yaw_mec*(180.0f/PI); 
    float err=rel-fol_tgt_deg;
    float want,step;
    while(err> 180.0f) err-=360.0f;
    while(err<-180.0f) err+=360.0f;
    if(fabsf(err)>FOLLOW_SWITCH_DEG)
    {
        fol_tgt_deg=(fol_tgt_deg==FOLLOW_TARGET_DEG)?FOLLOW_BACK_DEG:FOLLOW_TARGET_DEG;
        err=rel-fol_tgt_deg;
        while(err> 180.0f) err-=360.0f;
        while(err<-180.0f) err+=360.0f;
    }
    if(fabsf(err)<FOLLOW_DEAD_DEG) err=0.0f;
    want=-FOLLOW_WZ_MAX*(err/FOLLOW_FULL_DEG);
    //斜坡限速+限幅
    step=FOLLOW_WZ_MAX*(CTRL_DT/FOLLOW_RAMP_S);
    if(want> follow_wz+step) want= follow_wz+step;
    if(want< follow_wz-step) want= follow_wz-step;
    if(want> FOLLOW_WZ_MAX)  want= FOLLOW_WZ_MAX;
    if(want<-FOLLOW_WZ_MAX)  want=-FOLLOW_WZ_MAX;
    follow_wz=want;
    return follow_wz;
}

//小陀螺模式底盘自转量计算
static float chassis_spin(void)
{
    float want=SPIN_WZ_MAX;
    float step=fabsf(SPIN_WZ_MAX)*(CTRL_DT/FOLLOW_RAMP_S);
    if(want>spin_wz+step) want=spin_wz+step;
    if(want<spin_wz-step) want=spin_wz-step;
    spin_wz=want;
    return spin_wz;
}

void Chassis_ctrl(void)
{
	uint8_t tx_buf[8];
	float rpm_target[4];
	if(rc_sensor.errno==DEV_ID_ERR||
	   rc_sensor.work_state==DEV_OFFLINE||
	   rc_sensor.info->s1==RC_SW_UP){
		   //电流直接全发零防疯转
		   tx_buf[0]=tx_buf[1]=
		   tx_buf[2]=tx_buf[3]=
		   tx_buf[4]=tx_buf[5]=
		   tx_buf[6]=tx_buf[7]=0;
		   CAN_SendData(&hfdcan1,0x200,tx_buf);
		   //积分清零
		   pid_clear(L_F_Wheel.ctrl->speed_ctrl);
		   pid_clear(R_F_Wheel.ctrl->speed_ctrl);
		   pid_clear(L_B_Wheel.ctrl->speed_ctrl);
		   pid_clear(R_B_Wheel.ctrl->speed_ctrl);
		   //防下次控车飙速
		   L_F_Wheel.ctrl->speed_ctrl->target=
		   R_F_Wheel.ctrl->speed_ctrl->target=
		   L_B_Wheel.ctrl->speed_ctrl->target=
		   R_B_Wheel.ctrl->speed_ctrl->target=0;
		   follow_wz=spin_wz=0.0f;              
	}else{
		//归一化处理+死区
		float vx= DeathZoom(rc_sensor.info->ch3/RC_MAX,0.0f,0.1f)*MAX_SPEED;
		float vy=-DeathZoom(rc_sensor.info->ch2/RC_MAX,0.0f,0.1f)*MAX_SPEED; 
		float wz= DeathZoom(rc_sensor.info->ch0/RC_MAX,0.0f,0.1f)*MAX_SPIN_SPEED;
		if(Gimbal_Yaw.mode==YAW_GYRO)
		{
			if(RC_SW2_VALUE==RC_SW_DOWN) wz=chassis_spin();
			else
			{
				wz=chassis_follow_gimbal();
				spin_wz=0.0f; 
			}
		}
		else
		{
			follow_wz=0.0f;
			spin_wz=0.0f;
		}
		//小陀螺速度分解->合速度指向对齐位置
		if(Gimbal_Yaw.mode==YAW_GYRO && RC_SW2_VALUE==RC_SW_DOWN)
		{
			float a=Board_Rx.yaw_mec;
			float cosa=arm_cos_f32(a);
			float sina=arm_sin_f32(a);
			float vx_r=vx*cosa-vy*sina;
			float vy_r=vx*sina+vy*cosa;
			vx=vx_r;
			vy=vy_r;
		}
		if((Gimbal_Yaw.mode==YAW_GYRO)?((RC_SW2_VALUE!=RC_SW_DOWN)&&fol_tgt_deg>0.0f)
		                              :(Gimbal_Yaw.posi==YAW_BACK))
		{
			vx=-vx;
			vy=-vy;
		}
		chassis_vector_to_rpm(vx,vy,wz,rpm_target);
		//设定目标转速
		L_F_Wheel.ctrl->speed_ctrl->target=rpm_target[0];
		R_F_Wheel.ctrl->speed_ctrl->target=rpm_target[1];
		L_B_Wheel.ctrl->speed_ctrl->target=rpm_target[2];
		R_B_Wheel.ctrl->speed_ctrl->target=rpm_target[3];
		//PID执行
		L_F_Wheel.single_set_speed(&L_F_Wheel);
		R_F_Wheel.single_set_speed(&R_F_Wheel);
		L_B_Wheel.single_set_speed(&L_B_Wheel);
		R_B_Wheel.single_set_speed(&R_B_Wheel);
		Chassis_Group.group_set_torque(&Chassis_Group);
	}
}
