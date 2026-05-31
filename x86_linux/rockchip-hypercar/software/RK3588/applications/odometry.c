#include "odometry.h"
#include <math.h>

// static odometry_data_t g_odom;
odometry_data_t g_odom;
static rt_uint64_t     g_last_time_us = 0;
static rt_mutex_t      g_odom_mutex;


/* 四元数归一化 */
static void quat_normalize(void)
{
    float norm = sqrtf(g_odom.orientation_x * g_odom.orientation_x +
                       g_odom.orientation_y * g_odom.orientation_y +
                       g_odom.orientation_z * g_odom.orientation_z +
                       g_odom.orientation_w * g_odom.orientation_w);
    if (norm > 1e-8f)
    {
        float inv = 1.0f / norm;
        g_odom.orientation_x *= inv;
        g_odom.orientation_y *= inv;
        g_odom.orientation_z *= inv;
        g_odom.orientation_w *= inv;
    }
}

extern int ktime_odometry_init();
void odometry_init(void)
{
    g_odom = (odometry_data_t){
        .us_timestamp        = 0,
        .s_timestamp         =0.0,
        .pose_x = 0, .pose_y = 0, .pose_z = 0,
        .orientation_x = 0, .orientation_y = 0, .orientation_z = 0, .orientation_w = 1.0f,
        .twist_linear_x = 0, .twist_linear_y = 0, .twist_linear_z = 0,
        .twist_angular_x = 0, .twist_angular_y = 0, .twist_angular_z = 0
    };
    g_last_time_us = 0;

    g_odom_mutex = rt_mutex_create("odom_mtx", RT_IPC_FLAG_FIFO);
    RT_ASSERT(g_odom_mutex != RT_NULL);

    ktime_odometry_init();//初始化里程计专用定时器

}

void odometry_reset(void)
{
    rt_mutex_take(g_odom_mutex, RT_WAITING_FOREVER);
    g_odom.pose_x = g_odom.pose_y = g_odom.pose_z = 0.0f;
    g_odom.orientation_x = g_odom.orientation_y = g_odom.orientation_z = 0.0f;
    g_odom.orientation_w = 1.0f;
    g_last_time_us = 0;
    rt_mutex_release(g_odom_mutex);
}


void odometry_update(void)
{
    rt_mutex_take(g_odom_mutex, RT_WAITING_FOREVER);

    // 计算 dt
    float dt = 0.02f;

    // g_odom.timestamp = now_us;

    //定义为逆时针旋转方向为轮速正方向，即左右轮速极性相反
    float v_LB = motor_rpm[0];   // 左后
    float v_LF = motor_rpm[1];   // 左前
    float v_RF = -motor_rpm[2];   // 右前
    float v_RB = -motor_rpm[3];   // 右后

/* 标准X型麦轮正向运动学 —— 对应 LB, LF, RF, RB 顺序 */
    float vx = ( v_LF + v_RF + v_RB + v_LB) / 4.0f;
    float vy = ( v_LF - v_RF + v_RB - v_LB) / 4.0f;
    float wz = (-v_LF - v_RF + v_RB + v_LB) / (4.0f * (Axle_spacing+Wheel_spacing));

    // 积分位姿
    g_odom.pose_x += vx * dt;
    g_odom.pose_y += vy * dt;

// 姿态积分（绕Z轴）
    float theta = wz * dt;
    float half_theta = theta * 0.5f;
    float sin_half = sinf(half_theta);
    float cos_half = cosf(half_theta);

    float qx = g_odom.orientation_x;
    float qy = g_odom.orientation_y;
    float qz = g_odom.orientation_z;
    float qw = g_odom.orientation_w;

    g_odom.orientation_x = qx * cos_half + qy * sin_half;
    g_odom.orientation_y = qy * cos_half - qx * sin_half;
    g_odom.orientation_z = qz * cos_half + qw * sin_half;
    g_odom.orientation_w = qw * cos_half - qz * sin_half;

    quat_normalize();

    // 速度直接赋值
    g_odom.twist_linear_x  = vx;
    g_odom.twist_linear_y  = vy;
    g_odom.twist_angular_z = wz;

    rt_mutex_release(g_odom_mutex);
}

void odometry_update_no_lock(void)
{
    static rt_uint64_t last_time_us = 0;
    rt_uint64_t now_us = rt_tick_get_millisecond();

    //dt只用作计算里程计位姿计算 不更新时间戳
    float dt;
    if (last_time_us == 0) {
        dt = 0.02f;                                      // 第一次默认 20ms
    } else {
        dt = (float)(now_us - last_time_us) * 0.001f;  // 微秒 → 秒
        // 防抖动保护
        if (dt < 0.005f || dt > 0.1f) dt = 0.02f;
    }
    last_time_us = now_us;

    // 直接读 motor_rpm（只读，不涉及锁）
    //定义为逆时针旋转方向为轮速正方向，即左右轮速极性相反
    float v_LB = motor_rpm[0];
    float v_LF = motor_rpm[1];
    float v_RF = -motor_rpm[2];
    float v_RB = -motor_rpm[3];

    float vx = ( v_LF + v_RF + v_RB + v_LB) / 4.0f;
    float vy = ( v_LF - v_RF + v_RB - v_LB) / 4.0f;
    float wz = (-v_LF - v_RF + v_RB + v_LB) / (4.0f * (Axle_spacing+Wheel_spacing));

    g_odom.pose_x += vx * dt;
    g_odom.pose_y += vy * dt;

    // 姿态积分（同之前）
    float theta = wz * dt;
    float s = sinf(theta * 0.5f);
    float c = cosf(theta * 0.5f);
    float qx = g_odom.orientation_x;
    float qy = g_odom.orientation_y;
    float qz = g_odom.orientation_z;
    float qw = g_odom.orientation_w;

    g_odom.orientation_x = qx * c + qy * s;
    g_odom.orientation_y = qy * c - qx * s;
    g_odom.orientation_z = qz * c + qw * s;
    g_odom.orientation_w = qw * c - qz * s;

    quat_normalize();

    g_odom.twist_linear_x  = vx;
    g_odom.twist_linear_y  = vy;
    g_odom.twist_angular_z = wz;
    // g_odom.s_timestamp += dt;
}

/* 获取当前里程计（线程安全） */
const odometry_data_t* odometry_get(void)
{
    return &g_odom;
}


void odometry_print(void)
{
    rt_kprintf("=== Odometry ===\r\n");
    // rt_kprintf("time : %.4f s\r\n", g_odom.s_timestamp);
    rt_kprintf("pose : x=%.4f  y=%.4f\r\n", g_odom.pose_x, g_odom.pose_y);
    rt_kprintf("quat : [%.4f, %.4f, %.4f, %.4f]\r\n",
               g_odom.orientation_x, g_odom.orientation_y,
               g_odom.orientation_z, g_odom.orientation_w);
    rt_kprintf("vel  : vx=%.3f  vy=%.3f  wz=%.3f\r\n",
               g_odom.twist_linear_x, g_odom.twist_linear_y, g_odom.twist_angular_z);
    rt_kprintf("================\r\n");
}
