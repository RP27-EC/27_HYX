#ifndef __CHASSIS_H
#define __CHASSIS_H

#define R_CHASSIS_WHEEL       0.07625f            //单位:米
#define CHASSIS_REDUCTION     19.2f               //减速比
#define RC_MAX                660.0f              //遥控器最大控幅
#define YAW_FLIP_RAD          (95.0f*PI)/180.0f	  //掉头临界角(rad)
#define GYRO_YAW_SENS         180.0f              //陀螺仪 Yaw 最大角速度(度/秒)      
#define CTRL_DT               0.001f              //控制周期(s)
/*------------陀螺仪模式参数------------*/
#define FOLLOW_TARGET_DEG     -22.316185f  
#define FOLLOW_BACK_DEG       157.683815f  
#define FOLLOW_SWITCH_DEG     95.0f     		  //临界偏离角(超过->掉头)
#define FOLLOW_DEAD_DEG       3.0f      		  //陀螺仪死区角
#define FOLLOW_FULL_DEG       30.0f     		  //满速跟云台触发角
#define FOLLOW_RAMP_S         0.1f      		  //加到满速所用时间
/*------------小陀螺模式参数------------*/
#define FOLLOW_WZ_MAX         1.5f      	      //跟随最大自转量
#define SPIN_WZ_MAX           1.5f 	 		      //最大自旋量

void Chassis_ctrl(void);

#endif
