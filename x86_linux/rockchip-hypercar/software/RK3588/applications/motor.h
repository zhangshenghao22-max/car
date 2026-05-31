#ifndef __MOTOR_H_
#define __MOTOR_H_
#include <rtthread.h>
#include <rtdevice.h>
#include "robot_init.h"
#include "system.h"
#include <rt_encoder_soft.h>
#include "rk_gpio_ctrl.h"

void Set_Pwm(int motor_a,int motor_b,int motor_c,int motor_d,int servo);
void Limit_Pwm(int amplitude);
void Drive_Motor(float Vx,float Vy,float Vz);
float target_limit_float(float insert,float low,float high);
int target_limit_int(int insert,int low,int high);

int chassis_pwm_init(void);

int Incremental_PI(int motor_id, float Encoder, float Target,const char * pwm_dev_name);

int Incremental_PI_A(int motor_id, float Encoder, float Target,const char * pwm_dev_name);
int Incremental_PI_B(int motor_id, float Encoder, float Target,const char * pwm_dev_name);
int Incremental_PI_C(int motor_id, float Encoder, float Target,const char * pwm_dev_name);
int Incremental_PI_D(int motor_id, float Encoder, float Target,const char * pwm_dev_name);

#endif