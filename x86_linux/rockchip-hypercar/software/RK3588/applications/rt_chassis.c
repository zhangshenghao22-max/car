#include <rtthread.h>

#include "micro_ros_rtt.h"
#include "system.h"
#include "odometry.h"
#include "servo_bus.h"
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <std_msgs/msg/string.h>
#include <stdbool.h>

#define PERFORMANCE_STATS

// 性能统计
#ifdef PERFORMANCE_STATS
#include <rtdevice.h>
static uint32_t cmd_vel_count = 0;
static uint32_t max_latency_us = 0;
static uint64_t total_latency_us = 0;
static rt_tick_t last_cmd_time = 0;
#endif

// 错误检查宏
#define RCCHECK(fn) { \
    rcl_ret_t temp_rc = fn; \
    if((temp_rc != RCL_RET_OK)){\
        rt_kprintf("Failed status on line %d: %d. Aborting.\n", __LINE__, (int)temp_rc); \
    }\
}

#define RCSOFTCHECK(fn) { \
    rcl_ret_t temp_rc = fn; \
    if((temp_rc != RCL_RET_OK)){\
        rt_kprintf("Failed status on line %d: %d. Continuing.\n", __LINE__, (int)temp_rc);\
    }\
}

// 全局变量
static rcl_subscription_t cmd_sub;
static rcl_subscription_t arm_cmd_sub;
static rcl_publisher_t odom_pub;
static rcl_timer_t odom_timer;

static geometry_msgs__msg__Twist cmd_vel;
static std_msgs__msg__String arm_cmd_msg;
static nav_msgs__msg__Odometry odom_msg;
static char arm_cmd_buffer[512];

// 关键变量声明为volatile，确保实时更新
static volatile float target_vx = 0.0f;
static volatile float target_vy = 0.0f;
static volatile float target_wz = 0.0f;

// 用于存储传输配置
static char remote_addr[20] = {0};
static char remote_port[10] = {0};

// 快速发布标志
static bool odom_updated = false;

// 保存传输配置
static void save_transports(char *addr, char *port)
{
    if(addr != RT_NULL)
    {
        rt_strncpy(remote_addr, addr, sizeof(remote_addr) - 1);
        remote_addr[sizeof(remote_addr) - 1] = '\0';
    }

    if(port != RT_NULL)
    {
        rt_strncpy(remote_port, port, sizeof(remote_port) - 1);
        remote_port[sizeof(remote_port) - 1] = '\0';
    }
}

static char *get_remote_addr()
{
    return remote_addr;
}

static char *get_serial_name()
{
    return remote_addr;
}

static uint32_t get_remote_port()
{
    return (uint32_t)atoi(remote_port);
}

// 命令速度回调函数
void cmd_vel_callback(const void *msgin)
{
    const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
    
    target_vx = msg->linear.x;
    target_vy = msg->linear.y;
    target_wz = msg->angular.z;
    
    set_chassis_target(target_vx, target_vy, target_wz);
    
#ifdef PERFORMANCE_STATS
    // 性能统计
    rt_tick_t current_time = rt_tick_get();
    if (last_cmd_time > 0) {
        uint32_t latency = RT_TICK_PER_SECOND * (current_time - last_cmd_time) / 1000; // 转换为us
        if (latency > max_latency_us) max_latency_us = latency;
        total_latency_us += latency;
    }
    last_cmd_time = current_time;
    cmd_vel_count++;
    
    // 每100次消息打印一次统计
    if (cmd_vel_count % 100 == 0) {
        uint32_t avg_latency = total_latency_us / cmd_vel_count;
        rt_kprintf("Cmd_vel stats: count=%d, avg_latency=%uus, max_latency=%uus\n", 
              cmd_vel_count, avg_latency, max_latency_us);
    }
#endif
    
    // 标志里程计需要更新
    odom_updated = true;
#ifdef DEBUG_CMD_VEL
    LOG_D("Cmd_vel: vx=%.2f, vy=%.2f, wz=%.2f\n", target_vx, target_vy, target_wz);
#endif
}

static void arm_cmd_callback(const void *msgin)
{
    const std_msgs__msg__String *msg = (const std_msgs__msg__String *)msgin;

    if (!msg || !msg->data.data || msg->data.size == 0)
    {
        return;
    }

    servo_bus_send_raw_len(msg->data.data, msg->data.size);
}

// 快速里程计更新函数
static void publish_odometry_fast(void)
{
    // 获取当前时间 - 使用最快的方法
    int64_t time_stamp = rmw_uros_epoch_millis();
    odom_msg.header.stamp.sec = time_stamp / 1000;
    odom_msg.header.stamp.nanosec = (time_stamp % 1000) * 1000000;
    
    // 设置位姿
    odom_msg.pose.pose.position.x = g_odom.pose_x;
    odom_msg.pose.pose.position.y = g_odom.pose_y;
    odom_msg.pose.pose.position.z = 0.0;
    
    // 设置速度
    odom_msg.twist.twist.linear.x = g_odom.twist_linear_x;
    odom_msg.twist.twist.linear.y = g_odom.twist_linear_y;
    odom_msg.twist.twist.linear.z = 0.0;
    odom_msg.twist.twist.angular.z = g_odom.twist_angular_z;
    
    // 协方差保持为0
    
    // 发布里程计
    rcl_publish(&odom_pub, &odom_msg, NULL);
}

// 里程计定时器回调
void odom_timer_callback(rcl_timer_t *timer, int64_t last_time)
{
    RCLC_UNUSED(last_time);
    
    if(timer != RT_NULL)
    {
        publish_odometry_fast();
    }
}

// 事件驱动的里程计发布线程
static void odom_publish_thread_entry(void *parameter)
{
    (void)parameter;
    
    while(1)
    {
        // 事件驱动：当里程计数据更新时才发布
        if (odom_updated)
        {
            publish_odometry_fast();
            odom_updated = false;
        }
        rt_thread_mdelay(5);
    }
}

static void microros_chassis_thread_entry(void *parameter)
{
    RCLC_UNUSED(parameter);
    
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    
    // 初始化支持结构
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    
    // 创建节点
    rcl_node_t node;
    RCCHECK(rclc_node_init_default(&node, "rt_chassis", "", &support));
    
    // 初始化命令速度订阅器 - 使用最佳效果QoS（BEST_EFFORT）
    rcl_subscription_options_t sub_options = rcl_subscription_get_default_options();
    sub_options.qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;  // 降低延迟
    
    RCCHECK(rcl_subscription_init(
        &cmd_sub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/cmd_vel",
        &sub_options));

    RCCHECK(rcl_subscription_init(
        &arm_cmd_sub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "/arm_cmd",
        &sub_options));
    
    // 初始化里程计发布器 - 使用最佳效果QoS
    rcl_publisher_options_t pub_options = rcl_publisher_get_default_options();
    pub_options.qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    
    RCCHECK(rcl_publisher_init(
        &odom_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        "/odom",
        &pub_options));
    
    // 初始化里程计定时器（100Hz - 10ms周期）
    RCCHECK(rclc_timer_init_default(
        &odom_timer,
        &support,
        RCL_MS_TO_NS(10),  // 10ms = 100Hz，提高发布频率
        odom_timer_callback));
    
    // 初始化执行器 - 使用更小的处理窗口
    rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
    
    // 添加订阅到执行器 - 使用快速回调
    RCCHECK(rclc_executor_add_subscription(
        &executor, 
        &cmd_sub, 
        &cmd_vel, 
        &cmd_vel_callback, 
        ON_NEW_DATA));

    RCCHECK(rclc_executor_add_subscription(
        &executor,
        &arm_cmd_sub,
        &arm_cmd_msg,
        &arm_cmd_callback,
        ON_NEW_DATA));
    
    // 添加定时器到执行器
    RCCHECK(rclc_executor_add_timer(&executor, &odom_timer));
    
    // 初始化消息
    memset(&cmd_vel, 0, sizeof(cmd_vel));
    
    memset(&arm_cmd_msg, 0, sizeof(arm_cmd_msg));
    arm_cmd_msg.data.data = arm_cmd_buffer;
    arm_cmd_msg.data.size = 0;
    arm_cmd_msg.data.capacity = sizeof(arm_cmd_buffer);

    // 初始化里程计消息
    memset(&odom_msg, 0, sizeof(odom_msg));
    
    // 初始化里程计消息的字符串字段
    static char frame_id[5] = "odom";
    static char child_frame_id[10] = "base_link";
    
    odom_msg.header.frame_id.data = frame_id;
    odom_msg.header.frame_id.size = 4;
    odom_msg.header.frame_id.capacity = 5;
    
    odom_msg.child_frame_id.data = child_frame_id;
    odom_msg.child_frame_id.size = 9;
    odom_msg.child_frame_id.capacity = 10;
    
    // 初始化姿态四元数为单位四元数
    odom_msg.pose.pose.orientation.w = 1.0;
    
    rt_kprintf("RT-Thread chassis node started (optimized for low latency)\n");
    rt_kprintf("Subscribing to /cmd_vel with BEST_EFFORT QoS\n");
    rt_kprintf("Subscribing to /arm_cmd with BEST_EFFORT QoS\n");
    rt_kprintf("Publishing /odom at 100Hz\n");
    
#ifdef PERFORMANCE_STATS
    rt_kprintf("Performance statistics enabled\n");
#endif
    
    while(1)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(0.5));
        rt_thread_mdelay(1);
    }
    
    // 清理资源
    RCCHECK(rcl_subscription_fini(&cmd_sub, &node));
    RCCHECK(rcl_subscription_fini(&arm_cmd_sub, &node));
    RCCHECK(rcl_publisher_fini(&odom_pub, &node));
    RCCHECK(rcl_timer_fini(&odom_timer));
    RCCHECK(rclc_executor_fini(&executor));
    RCCHECK(rcl_node_fini(&node));
    RCCHECK(rclc_support_fini(&support));
}

// 帮助信息
static void help()
{
    rt_kprintf("Usage:\n");
    rt_kprintf("\tmicroros_chassis <communication> [params]\n");
    rt_kprintf("\nCommunication options:\n");
    rt_kprintf("\tserial <device>     - Serial communication\n");
    rt_kprintf("\tudp <ip> <port>     - UDP communication\n");
    rt_kprintf("\ttcp <ip> <port>     - TCP communication\n");
    rt_kprintf("\nExamples:\n");
    rt_kprintf("\tmicroros_chassis serial uart2\n");
    rt_kprintf("\tmicroros_chassis udp 10.10.10.31 8888\n");
    rt_kprintf("\tmicroros_chassis tcp 10.10.10.31 8888\n");
}

#if defined(PKG_MICRO_ROS_USE_UDP) || defined(PKG_MICRO_ROS_USE_TCP)
struct netdev;
extern struct netdev *netdev_list;

static rt_bool_t microros_network_ready(void)
{
    if (netdev_list == RT_NULL)
    {
        rt_kprintf("micro-ROS network not ready: no netdev. Run ifconfig first.\n");
        return RT_FALSE;
    }

    rt_kprintf("micro-ROS network ready\n");
    return RT_TRUE;
}
#endif

static void microros_chassis(int argc, char* argv[])
{
    if(argc < 2)
    {
        help();
        return;
    }

    if(rt_strcmp(argv[1], "serial") == 0)
    {
#if defined(PKG_MICRO_ROS_USE_SERIAL)
        if(argc == 2)
        {
            if(MICRO_ROS_SERIAL_NAME == RT_NULL)
            {
                rt_kprintf("'MICRO_ROS_SERIAL_NAME' macro is not defined in rtconfig.h");
                return;
            }
            set_microros_transports(MICRO_ROS_SERIAL_NAME);
            rt_kprintf("Using serial: %s\n", MICRO_ROS_SERIAL_NAME);
        }
        else
        {
            save_transports(argv[2], RT_NULL);
            set_microros_transports(get_serial_name());
            rt_kprintf("Using serial: %s\n", argv[2]);
        }
#else
        rt_kprintf("Serial communication is disabled\n");
        return;
#endif
    }
    else if(rt_strcmp(argv[1], "udp") == 0)
    {
#if defined(PKG_MICRO_ROS_USE_UDP)
        if (!microros_network_ready())
        {
            return;
        }

        if(argc == 2)
        {
            if((MICRO_ROS_UDP_IP == RT_NULL) || (MICRO_ROS_UDP_PORT == RT_NULL))
            {
                rt_kprintf("'MICRO_ROS_UDP_IP' and 'MICRO_ROS_UDP_PORT' macros are not defined in rtconfig.h\n");
                return;
            }
            set_microros_udp_transports(MICRO_ROS_UDP_IP, atoi(MICRO_ROS_UDP_PORT));
            rt_kprintf("Using UDP: %s:%s\n", MICRO_ROS_UDP_IP, MICRO_ROS_UDP_PORT);
        }
        else if(argc == 4)
        {
            save_transports(argv[2], argv[3]);
            set_microros_udp_transports(get_remote_addr(), get_remote_port());
            rt_kprintf("Using UDP: %s:%s\n", argv[2], argv[3]);
        }
        else
        {
            rt_kprintf("Invalid UDP parameters\n");
            help();
            return;
        }
#else
        rt_kprintf("UDP communication is disabled\n");
        return;
#endif
    }
    else if(rt_strcmp(argv[1], "tcp") == 0)
    {
#if defined(PKG_MICRO_ROS_USE_TCP)
        if (!microros_network_ready())
        {
            return;
        }

        if(argc == 2)
        {
            if((MICRO_ROS_TCP_IP == RT_NULL) || (MICRO_ROS_TCP_PORT == RT_NULL))
            {
                rt_kprintf("'MICRO_ROS_TCP_IP' and 'MICRO_ROS_TCP_PORT' macros are not defined in rtconfig.h\n");
                return;
            }
            set_microros_tcp_transports(MICRO_ROS_TCP_IP, atoi(MICRO_ROS_TCP_PORT));
            rt_kprintf("Using TCP: %s:%s\n", MICRO_ROS_TCP_IP, MICRO_ROS_TCP_PORT);
        }
        else if(argc == 4)
        {
            save_transports(argv[2], argv[3]);
            set_microros_tcp_transports(get_remote_addr(), get_remote_port());
            rt_kprintf("Using TCP: %s:%s\n", argv[2], argv[3]);
        }
        else
        {
            rt_kprintf("Invalid TCP parameters\n");
            help();
            return;
        }
#else
        rt_kprintf("TCP communication is disabled\n");
        return;
#endif
    }
    else
    {
        help();
        return;
    }

    rt_thread_t thread = rt_thread_create("chassis_node", 
                                          microros_chassis_thread_entry, 
                                          RT_NULL, 
                                          8192, 
                                          20, 
                                          10);
    if(thread != RT_NULL)
    {
        rt_thread_startup(thread);
        rt_kprintf("Micro-ROS chassis node thread created\n");
    }
    else
    {
        rt_kprintf("Failed to create chassis node thread\n");
    }
}
MSH_CMD_EXPORT(microros_chassis, Micro-ROS chassis controller (cmd_vel subscriber, odom publisher));
