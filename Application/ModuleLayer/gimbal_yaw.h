#ifndef __GIMBAL_YAW_H
#define __GIMBAL_YAW_H

#include "communicate.h"

#define YAW_FRONT_RAD   (-22.316185f*PI/180.0f) //µç³Ø²Ö½Ç¶È
#define YAW_BACK_RAD    (157.683815f*PI/180.0f)

typedef enum {
    YAW_MEC,
    YAW_GYRO,
    YAW_SLEEP,
} Yaw_Mode_e;

typedef enum{
	YAW_FRONT,
	YAW_BACK,
}Posi_e;

typedef struct Gimbal_Yaw_struct_t{
    Yaw_Mode_e mode;
	Posi_e posi;
    float target_rad;
	void (*update)(struct Gimbal_Yaw_struct_t *Gimbal_Yaw);
} Gimbal_Yaw_t;

extern Gimbal_Yaw_t Gimbal_Yaw;

void Gimbal_Yaw_Init(Gimbal_Yaw_t *Gimbal_Yaw);

#endif
