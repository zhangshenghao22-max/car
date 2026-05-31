#ifndef __GPIO_INIT_H_
#define __GPIO_INIT_H_

#include <rtthread.h>
#include <rtdevice.h>
#include "robot_init.h"
#include "system.h"

//原本定义
#define PIN_AIN1    RK_GPIO4_B3
#define PIN_AIN2    RK_GPIO4_B2
#define PIN_BIN1    RK_GPIO3_B7
#define PIN_BIN2    RK_GPIO1_B1
#define PIN_CIN1    RK_GPIO1_B4
#define PIN_CIN2    RK_GPIO1_B5
#define PIN_DIN1    RK_GPIO4_C6
#define PIN_DIN2    RK_GPIO3_B3 

/* 电机编号枚举（清晰易读） */
typedef enum
{
    MOT_A = 0,  /* left rear */
    MOT_B = 1,  /* left front */
    MOT_C = 2,  /* right front */
    MOT_D = 3,  /* right rear */
    MOT_MAX
} motor_id_t;

/* 方向枚举（正转、反转、刹车） */
/* 此处的正转为电机逆时针转动，并非小车前进方向*/
typedef enum
{
    MOTOR_STOP    = 0,   // 刹车（AIN1=AIN2=0 或 1 都行，推荐全 0）  
    MOTOR_REVERSE = 1,   // 反转
    MOTOR_FORWARD = 2    // 正转
} motor_dir_t;

void init_io(void);

void run_on(void);

void motor_set_direction(motor_id_t id, motor_dir_t dir);
void motor_apply_pwm_dir(motor_id_t id, int signed_pwm);

#endif
