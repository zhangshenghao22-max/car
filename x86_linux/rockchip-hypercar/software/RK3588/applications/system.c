#include "system.h"

//Vehicle three-axis target moving speed, unit: m/s
//小车三轴目标运动速度，单位：m/s
float Move_X, Move_Y, Move_Z;

static rt_mutex_t g_move_lock = RT_NULL;

//PID parameters of Speed control
//速度控制PID参数
float Velocity_KP=200.0,Velocity_KI=300;

//Smooth control of intermediate variables, dedicated to omni-directional moving cars
//平滑控制中间变量，全向移动小车专用
Smooth_Control smooth_control;

//The parameter structure of the motor
//电机的参数结构体
Motor_parameter MOTOR_A,MOTOR_B,MOTOR_C,MOTOR_D;

/************ 小车型号相关变量 **************************/
/************ Variables related to car model ************/
//Encoder accuracy
////电机(车轮)转1圈对应的编码器数值
float Encoder_precision;
//Wheel circumference, unit: m
//轮子周长，单位：m
float Wheel_perimeter;
//Drive wheel base, unit: m
//主动轮轮距，单位：m
float Wheel_spacing;
//The wheelbase of the front and rear axles of the trolley, unit: m
//小车前后轴的轴距，单位：m
float Axle_spacing;
//All-directional wheel turning radius, unit: m
//全向轮转弯半径，单位：m
float Omni_turn_radiaus;
/************ 小车型号相关变量 **************************/
/************ Variables related to car model ************/

//系统上电计时
rt_uint32_t sys_time_count=0;

//小车刹停标志位
bool STOP_FLAG=1;

//初始化system
void systemInit(void)
{
    //初始化小车
    Robot_Select();
    init_io();
    odometry_init();
    chassis_target_init();
    rt_thread_mdelay(100);
}

void chassis_target_init(void)
{
    g_move_lock = rt_mutex_create("move_lock", RT_IPC_FLAG_FIFO);
    RT_ASSERT(g_move_lock != RT_NULL);
}

void set_chassis_target(float vx, float vy, float wz)
{
    rt_mutex_take(g_move_lock, RT_WAITING_FOREVER);
    Move_X = vx;
    Move_Y = vy;
    Move_Z = wz;
    rt_mutex_release(g_move_lock);
    if(!(vx==0&&vy==0&&wz==0))STOP_FLAG=0;
    else STOP_FLAG=1;
}
