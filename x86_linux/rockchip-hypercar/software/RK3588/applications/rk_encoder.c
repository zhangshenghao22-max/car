#include <ktime.h>
#include "rk_encoder.h"

#define SAMPLE_PERIOD_MS   20    
#define PPR                500   // 编码器每转500个脉冲对（lines）
#define REDU_RATIO         30    // 减速比（电机轴编码器）

rt_int32_t last_count[4] = {0};
double motor_rpm[4] = {0.0};

static struct rt_ktime_hrtimer rk_timer;
static int rpm_calculation(wheel_pos_t pos,rt_int32_t count)
{
    double dt = SAMPLE_PERIOD_MS / 1000.0; 
    // motor_rpm[pos] = (double)count  / ((double)PPR * dt);
    motor_rpm[pos] = (double)count  *  Wheel_perimeter / ((double)Encoder_precision * dt);//  m/s
    return RT_EOK;
}

static void speed_calc_timeout(void *parameter)
{
    rt_int32_t current_count = 0;
    int ret = 0;

    rk_soft_encoder_update();

    for (int i = 0; i < WHEEL_MAX; i++)
    {
        ret = rk_soft_encoder_pulse(i, &current_count);
        rpm_calculation(i, current_count);
        last_count[i] = current_count;
    }
}

#define ENCODER_TEST_THREAD_PRIORITY     20
#define ENCODER_TEST_THREAD_STACK_SIZE   8192
#define ENCODER_TEST_THREAD_TIMESLICE    10

static const char *wheel_names[4] = {"LF (Left Front)", "RF (Right Front)", "LB (Left Back)", "RB (Right Back)"};
static void encoder_print_thread_entry(void *parameter)
{
    int ret = 0;
    encoder_dir_t dir = STOP_WHEEL;
    const char *dir_str = "Stop";

    // rt_kprintf("\n=== 四轮编码器实时测试开始（2x模式） ===\n");
    // rt_kprintf("PPR=%d  |  减速比=1:%d  |  采样周期=500ms \n\n", PPR, REDU_RATIO);

    // rt_kprintf("%-12s %8s %10s %12s\n", "轮子", "方向", "RPM", "累计脉冲");
    // rt_kprintf("---------------------------------------------------------\n");

    while (1)
    {
        for (int i = 0; i < WHEEL_MAX; i++)
        {
            ret = rk_soft_encoder_get_dir(i, &dir);
            if(ret < 0) continue;

            if (dir > 0) dir_str = "Forward↑";
            else if (dir < 0) dir_str = "Reverse↓";
            else dir_str = "Stop";

        //     rt_kprintf("%-12s %8s %10.3f %12ld\n",
        //             wheel_names[i],
        //             dir_str,
        //             motor_rpm[i],
        //             (long)last_count[i]);
        // rt_kprintf("---------------------------------------------------------\n");
        }
        rt_thread_mdelay(500);
    }
}

static struct rt_timer timer1;
void encoder_init(void)
{
    rt_thread_t tid;
    unsigned long cnt;
    int64_t time_timer;
    unsigned long res = rt_ktime_cputimer_getres();

    rt_ktime_hrtimer_delay_init(&rk_timer);
    rt_ktime_hrtimer_init(&rk_timer, "encoder_hrtimer", RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_HARD_TIMER, speed_calc_timeout, RT_NULL);

    time_timer = SAMPLE_PERIOD_MS * MICROSECOND_PER_SECOND;//20ms
    cnt = (time_timer * RT_KTIME_RESMUL) / res;
    rt_ktime_hrtimer_start(&rk_timer, cnt);

    tid = rt_thread_create("enc_test",
                encoder_print_thread_entry,
                RT_NULL,
                ENCODER_TEST_THREAD_STACK_SIZE,
                ENCODER_TEST_THREAD_PRIORITY,
                ENCODER_TEST_THREAD_TIMESLICE);
    if (tid != RT_NULL)
    {
        rt_thread_startup(tid);
    }
    else
    {
        rt_kprintf("[encoder_test] create thread failed!\n");
    }
}
