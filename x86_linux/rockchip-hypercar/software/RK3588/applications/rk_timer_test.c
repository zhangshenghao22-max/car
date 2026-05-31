#include <rtthread.h>
#include "ktime.h"
#include <rtdevice.h>
#include <dt-bindings-pinctrl.h>
#include "odometry.h"

#define PERIOD_MS  20

static struct rt_ktime_hrtimer odometry_update_timer;

// static rt_uint8_t ain1_state = 0;
// static rt_uint8_t ain2_state = 0;
// static rt_uint64_t flag = 0;

static void hrtimer_callback(void *parameter)
{

    odometry_update_no_lock();

}

static void ktime_odometry_entry(void *parameter)
{
    unsigned long cnt;
    unsigned long res = rt_ktime_cputimer_getres();

    rt_ktime_hrtimer_delay_init(&odometry_update_timer);
    /* 初始化定时器 */
    rt_ktime_hrtimer_init(&odometry_update_timer,
                          "odometry_update_hrtimer", 
                          RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_HARD_TIMER, 
                          hrtimer_callback,
                          RT_NULL);

    int64_t ns = PERIOD_MS * MICROSECOND_PER_SECOND;//20ms
    cnt = (ns * RT_KTIME_RESMUL) / res;

    rt_ktime_hrtimer_start(&odometry_update_timer, cnt);  //启动定时器，20ms触发一次。
}

/* 导出为 shell 命令 */
int ktime_odometry_init(void)
{
    rt_thread_t tid = rt_thread_create("ktime_odometry", ktime_odometry_entry, RT_NULL, 2048, 20, 10);
    if (tid)
        rt_thread_startup(tid);
    return 0;
}
MSH_CMD_EXPORT(ktime_odometry_init, ktime test);
