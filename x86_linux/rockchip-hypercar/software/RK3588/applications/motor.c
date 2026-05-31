#include "motor.h"

void Drive_Motor(float Vx, float Vy, float Vz)//只有一处调用
{
    float amplitude = 0.9; //Wheel target speed limit //车轮目标速度限幅 max ±0.9m/s
    float v[4]={0.0,0.0,0.0,0.0};
    // Wheel order: A=left rear, B=left front, C=right front, D=right rear.
    // ROS convention: +Vx forward, +Vy left, +Vz counter-clockwise.
    v[0] = Vx + Vy - Vz * (Axle_spacing + Wheel_spacing);
    v[1] = Vx - Vy - Vz * (Axle_spacing + Wheel_spacing);
    v[2] = Vx + Vy + Vz * (Axle_spacing + Wheel_spacing);
    v[3] = Vx - Vy + Vz * (Axle_spacing + Wheel_spacing);

    
    //Wheel (motor) target speed limit //车轮(电机)目标速度限幅
    MOTOR_A.Target = target_limit_float(v[0], -amplitude, amplitude);
    MOTOR_B.Target = target_limit_float(v[1], -amplitude, amplitude);
    MOTOR_C.Target = target_limit_float(v[2], -amplitude, amplitude);
    MOTOR_D.Target = target_limit_float(v[3], -amplitude, amplitude);

}
/**************************************************************************
 Function: Incremental PI controller
 Input   : Encoder measured value (actual speed), target speed
 Output  : Motor PWM
 According to the incremental discrete PID formula
 pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
 e(k) represents the current deviation
 e(k-1) is the last deviation and so on
 PWM stands for incremental output
 In our speed control closed loop system, only PI control is used
 pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)

 函数功能：增量式PI控制器
 入口参数：编码器测量值(实际速度)，目标速度
 返回  值：电机PWM
 根据增量式离散PID公式
 pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
 e(k)代表本次偏差
 e(k-1)代表上一次的偏差  以此类推
 pwm代表增量输出
 在我们的速度控制闭环系统里面，只使用PI控制
 pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)
 **************************************************************************/

int Incremental_PI(int motor_id, float Encoder, float Target,const char * pwm_dev_name)
{
    static float Bias[MOT_MAX]     = {0};
    static float Last_bias[MOT_MAX] = {0};
    static float Integral[MOT_MAX] = {0};  

    if(motor_id==2||motor_id==3)Encoder=-Encoder;//C,D编码器极性同A,B相反

    Bias[motor_id] = Target - Encoder;

    // 增量式 PI
    float Pwm_inc = Velocity_KP * (Bias[motor_id] - Last_bias[motor_id]) 
          + Velocity_KI * Bias[motor_id];

    // 关键的抗积分饱和！！！这四行是灵魂
    if (Integral[motor_id] >= 10000 && Pwm_inc > 0)  Pwm_inc = 0;   // 已正满，禁止继续加
    if (Integral[motor_id] <= -10000 && Pwm_inc < 0) Pwm_inc = 0;  // 已负满，禁止继续减

    // 累加到输出
    Integral[motor_id] += Pwm_inc;


    // 限幅 0~10000（对应 0~100.0% 占空比）
    if (Integral[motor_id] > 10000)  Integral[motor_id] = 10000;
    if (Integral[motor_id] < -10000)     Integral[motor_id] = -10000;

    Last_bias[motor_id] = Bias[motor_id];
    rt_kprintf("%s → 10kHz, duty ratio:%.1f%%  wheel speed:%10.3f  target velocity:%.3f deviation:%.3f integral:%.3f\n", pwm_dev_name,Integral[motor_id]/100,Encoder,Target,Bias[motor_id],Pwm_inc);
    return (int)Integral[motor_id];
}

int Incremental_PI_A(int motor_id, float Encoder, float Target,const char * pwm_dev_name)
{
    static float Bias     = 0;
    static float Last_bias = 0;
    static float Integral = 0;  


    Bias= Target - Encoder;

    // 增量式 PI
    float Pwm_inc = Velocity_KP * (Bias - Last_bias) 
          + Velocity_KI * Bias;

    // rt_kprintf("KP %.3f KI%.3f\n",Velocity_KP * (Bias - Last_bias) ,Velocity_KI * Bias);
    // 关键的抗积分饱和！！！这四行是灵魂
    if (Integral >= 1000 && Pwm_inc > 0)  Pwm_inc = 0;   // 已正满，禁止继续加
    if (Integral <= -1000 && Pwm_inc < 0) Pwm_inc = 0;  // 已负满，禁止继续减

    // 累加到输出
    Integral += Pwm_inc;


    // 限幅 0~1000（对应 0~100.0% 占空比）
    if (Integral > 1000)  Integral = 1000;
    if (Integral < -1000)     Integral = -1000;

    Last_bias = Bias;
    // rt_kprintf("%s → 10kHz, 占空比 %.1f%%  轮速%10.3f  目标速度 %.3f  \n", pwm_dev_name,Integral/10,Encoder,Target);
    // rt_kprintf("%.1f,%10.3f,%.3f  \n", Integral/10,Encoder,Target);
    return (int)Integral;
}
int Incremental_PI_B(int motor_id, float Encoder, float Target,const char * pwm_dev_name)
{
    static float Bias     = 0;
    static float Last_bias = 0;
    static float Integral = 0;  


    Bias= Target - Encoder;

    // 增量式 PI
    float Pwm_inc = Velocity_KP * (Bias - Last_bias) 
          + Velocity_KI * Bias;

    // 关键的抗积分饱和！！！这四行是灵魂
    if (Integral >= 1000 && Pwm_inc > 0)  Pwm_inc = 0;   // 已正满，禁止继续加
    if (Integral <= -1000 && Pwm_inc < 0) Pwm_inc = 0;  // 已负满，禁止继续减

    // 累加到输出
    Integral += Pwm_inc;


    // 限幅 0~10000（对应 0~100.0% 占空比）
    if (Integral > 1000)  Integral = 1000;
    if (Integral < -1000)     Integral = -1000;

    Last_bias = Bias;
    // rt_kprintf("%s → 10kHz, 占空比 %.1f%%  轮速%10.3f  目标速度 %.3f 偏差%.3f 积分%.3f\n", pwm_dev_name,Integral/10,Encoder,Target,Bias,Pwm_inc);
    return (int)Integral;
}
int Incremental_PI_C(int motor_id, float Encoder, float Target,const char * pwm_dev_name)
{
    static float Bias     = 0;
    static float Last_bias = 0;
    static float Integral = 0;  

    Encoder=-Encoder;//C,D编码器极性同A,B相反

    Bias= Target - Encoder;

    // 增量式 PI
    float Pwm_inc = Velocity_KP * (Bias - Last_bias) 
          + Velocity_KI * Bias;

    // 关键的抗积分饱和！！！这四行是灵魂
    if (Integral >= 1000 && Pwm_inc > 0)  Pwm_inc = 0;   // 已正满，禁止继续加
    if (Integral <= -1000 && Pwm_inc < 0) Pwm_inc = 0;  // 已负满，禁止继续减

    // 累加到输出
    Integral += Pwm_inc;


    // 限幅 0~10000（对应 0~100.0% 占空比）
    if (Integral > 1000)  Integral = 1000;
    if (Integral < -1000)     Integral = -1000;

    Last_bias = Bias;
    // rt_kprintf("%s → 10kHz, 占空比 %.1f%%  轮速%10.3f  目标速度 %.3f 偏差%.3f 积分%.3f\n", pwm_dev_name,Integral/10,Encoder,Target,Bias,Pwm_inc);
    return (int)Integral;
}
int Incremental_PI_D(int motor_id, float Encoder, float Target,const char * pwm_dev_name)
{
    static float Bias     = 0;
    static float Last_bias = 0;
    static float Integral = 0;  

    Encoder=-Encoder;//C,D编码器极性同A,B相反

    Bias= Target - Encoder;

    // 增量式 PI
    float Pwm_inc = Velocity_KP * (Bias - Last_bias) 
          + Velocity_KI * Bias;

    // 关键的抗积分饱和！！！这四行是灵魂
    if (Integral >= 1000 && Pwm_inc > 0)  Pwm_inc = 0;   // 已正满，禁止继续加
    if (Integral <= -1000 && Pwm_inc < 0) Pwm_inc = 0;  // 已负满，禁止继续减

    // 累加到输出
    Integral += Pwm_inc;


    // 限幅 0~10000（对应 0~100.0% 占空比）
    if (Integral > 1000)  Integral = 1000;
    if (Integral < -1000)     Integral = -1000;

    Last_bias = Bias;
    // rt_kprintf("%s → 10kHz, 占空比 %.1f%%  轮速%10.3f  目标速度 %.1f 偏差%.1f 积分%.1f\n", pwm_dev_name,Integral/10,Encoder,Target,Bias,Pwm_inc);
    return (int)Integral;
}

/**************************************************************************
 Function: Limiting function
 Input   : Value
 Output  : none
 函数功能：限幅函数
 入口参数：幅值
 返回  值：无
 **************************************************************************/
float target_limit_float(float insert, float low, float high)
{
    if (insert < low)
        return low;
    else if (insert > high)
        return high;
    else
        return insert;
}
int target_limit_int(int insert, int low, int high)
{
    if (insert < low)
        return low;
    else if (insert > high)
        return high;
    else
        return insert;
}
