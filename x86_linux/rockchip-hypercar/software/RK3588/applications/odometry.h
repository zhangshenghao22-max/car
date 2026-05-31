#ifndef ODOMETRY_H
#define ODOMETRY_H
#include <rtthread.h>
#include <rtdevice.h>
#include <stdint.h>
#include "robot_init.h"
#include "system.h"
#include <rt_encoder_soft.h>
#include "rk_gpio_ctrl.h"
#include "rk_encoder.h"

typedef struct {
    rt_uint64_t us_timestamp;          // 微秒时间戳
    float s_timestamp;          // 秒时间戳
    /* Pose */
    float pose_x;
    float pose_y;
    float pose_z;                   // 固定为 0

    float orientation_x;
    float orientation_y;
    float orientation_z;
    float orientation_w;

    /* Twist */
    float twist_linear_x;
    float twist_linear_y;
    float twist_linear_z;           // 固定为 0

    float twist_angular_x;          // 固定为 0
    float twist_angular_y;          // 固定为 0
    float twist_angular_z;
} odometry_data_t;

/* 初始化/重置里程计 */
void odometry_init(void);
void odometry_reset(void);

/**
 * @brief 主更新函数
 *        自动读取全局变量 motor_rpm[4]（单位：m/s）
 */
void odometry_update(void);

void odometry_update_no_lock(void);

/* 线程安全获取当前里程计 */
const odometry_data_t* odometry_get(void);

/* 调试打印（FinSH/mshell 可用） */
void odometry_print(void);


extern odometry_data_t g_odom;

#endif