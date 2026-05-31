#include <rtthread.h>
#include <rtdevice.h>
#include <stdlib.h>
#include "rk_gpio_ctrl.h"
#include "motor.h"
#include "rk_encoder.h"
#include "odometry.h"
#include "system.h"

#define PWM_DEV_NUM        4
#define PWM_CHANNEL        0
#define PERIOD_10KHZ_NS    100000ULL
#define MOTOR_PWM_MAX      1000

static const char *pwm_dev_name[PWM_DEV_NUM] = {
    "pwm8", "pwm13", "pwm14", "pwm15"
};

static struct rt_device_pwm *pwm_dev[PWM_DEV_NUM] = {RT_NULL};
static rt_int16_t pwm_Integral[PWM_DEV_NUM] = {0, 0, 0, 0};
static rt_uint16_t motor_duty[PWM_DEV_NUM] = {0, 0, 0, 0};
static const rt_int8_t motor_output_sign[PWM_DEV_NUM] = {
    1,   /* A: left rear */
    1,   /* B: left front */
    -1,  /* C: right front */
    -1   /* D: right rear */
};

static volatile rt_bool_t raw_mode = RT_FALSE;
static rt_bool_t raw_hw_ready = RT_FALSE;
static rt_int16_t raw_pwm[PWM_DEV_NUM] = {0, 0, 0, 0};

static int clamp_pwm(int pwm)
{
    if (pwm > MOTOR_PWM_MAX) return MOTOR_PWM_MAX;
    if (pwm < -MOTOR_PWM_MAX) return -MOTOR_PWM_MAX;
    return pwm;
}

static int abs_pwm(int pwm)
{
    return pwm < 0 ? -pwm : pwm;
}

static void apply_signed_motor_pwm(int id, int signed_pwm)
{
    if (id < 0 || id >= PWM_DEV_NUM) return;

    signed_pwm = clamp_pwm(signed_pwm);
    motor_duty[id] = (rt_uint16_t)abs_pwm(signed_pwm);
    motor_apply_pwm_dir((motor_id_t)id, signed_pwm);

    if (pwm_dev[id] != RT_NULL)
    {
        rt_pwm_set(pwm_dev[id],
                   PWM_CHANNEL,
                   PERIOD_10KHZ_NS,
                   PERIOD_10KHZ_NS * motor_duty[id] / MOTOR_PWM_MAX);
    }
}

static void apply_all_stop(void)
{
    for (int i = 0; i < PWM_DEV_NUM; i++)
    {
        apply_signed_motor_pwm(i, 0);
    }
}

static void raw_hw_init_once(void)
{
    if (raw_hw_ready)
    {
        return;
    }

    init_io();

    for (int i = 0; i < PWM_DEV_NUM; i++)
    {
        if (pwm_dev[i] == RT_NULL)
        {
            pwm_dev[i] = (struct rt_device_pwm *)rt_device_find(pwm_dev_name[i]);
        }

        if (pwm_dev[i] == RT_NULL)
        {
            rt_kprintf("Warning: not found %s\n", pwm_dev_name[i]);
            continue;
        }

        rt_pwm_enable(pwm_dev[i], PWM_CHANNEL);
        rt_pwm_set(pwm_dev[i], PWM_CHANNEL, PERIOD_10KHZ_NS, 0);
    }

    raw_hw_ready = RT_TRUE;
}

static void set_raw_pwm_all(int pwm_a, int pwm_b, int pwm_c, int pwm_d)
{
    raw_hw_init_once();

    raw_pwm[MOT_A] = (rt_int16_t)clamp_pwm(pwm_a);
    raw_pwm[MOT_B] = (rt_int16_t)clamp_pwm(pwm_b);
    raw_pwm[MOT_C] = (rt_int16_t)clamp_pwm(pwm_c);
    raw_pwm[MOT_D] = (rt_int16_t)clamp_pwm(pwm_d);

    raw_mode = RT_TRUE;
    STOP_FLAG = 0;
    Move_X = 0;
    Move_Y = 0;
    Move_Z = 0;

    for (int i = 0; i < PWM_DEV_NUM; i++)
    {
        apply_signed_motor_pwm(i, raw_pwm[i]);
    }
}

static void stop_raw_pwm_all(void)
{
    raw_mode = RT_FALSE;
    for (int i = 0; i < PWM_DEV_NUM; i++)
    {
        raw_pwm[i] = 0;
    }

    Move_X = 0;
    Move_Y = 0;
    Move_Z = 0;
    STOP_FLAG = 1;

    raw_hw_init_once();
    apply_all_stop();
}

static int speed_control(int argc, char *argv[])
{
    if (argc != 4)
    {
        rt_kprintf("Usage: speed_control <x_vel> <y_vel> <z_vel>\n");
        rt_kprintf("Example: speed_control 0.5 0.0 1.57\n");
        return -1;
    }

    float x_vecl = atof(argv[1]);
    float y_vecl = atof(argv[2]);
    float z_vecl = atof(argv[3]);

    raw_mode = RT_FALSE;
    set_chassis_target(x_vecl, y_vecl, z_vecl);

    rt_kprintf("[speed] Set chassis target velocity:\n");
    rt_kprintf("        x_vel = %.3f m/s\n", x_vecl);
    rt_kprintf("        y_vel = %.3f m/s\n", y_vecl);
    rt_kprintf("        z_vel = %.3f rad/s\n", z_vecl);

    return 0;
}
MSH_CMD_EXPORT(speed_control, speed_control);

static int motor_raw(int argc, char **argv)
{
    if (argc != 3)
    {
        rt_kprintf("Usage: motor_raw <id:0-3> <pwm:-1000..1000>\n");
        rt_kprintf("Example: motor_raw 0 200\n");
        return -1;
    }

    int id = atoi(argv[1]);
    int pwm = clamp_pwm(atoi(argv[2]));

    if (id < 0 || id >= PWM_DEV_NUM)
    {
        rt_kprintf("Invalid motor id, expected 0..3\n");
        return -1;
    }

    set_raw_pwm_all(0, 0, 0, 0);
    raw_pwm[id] = (rt_int16_t)pwm;
    apply_signed_motor_pwm(id, raw_pwm[id]);

    rt_kprintf("motor_raw: id=%d signed_pwm=%d\n", id, pwm);
    return 0;
}
MSH_CMD_EXPORT(motor_raw, raw single motor PWM+IN1/IN2 test);

static int motor_raw4(int argc, char **argv)
{
    if (argc != 5)
    {
        rt_kprintf("Usage: motor_raw4 <A_PWM> <B_PWM> <C_PWM> <D_PWM>\n");
        rt_kprintf("Wheel order: A=left rear, B=left front, C=right front, D=right rear\n");
        rt_kprintf("Example: motor_raw4 150 150 -150 -150\n");
        return -1;
    }

    int pwm_a = atoi(argv[1]);
    int pwm_b = atoi(argv[2]);
    int pwm_c = atoi(argv[3]);
    int pwm_d = atoi(argv[4]);

    set_raw_pwm_all(pwm_a, pwm_b, pwm_c, pwm_d);
    rt_kprintf("motor_raw4: A=%d B=%d C=%d D=%d\n",
               raw_pwm[MOT_A], raw_pwm[MOT_B], raw_pwm[MOT_C], raw_pwm[MOT_D]);
    return 0;
}
MSH_CMD_EXPORT(motor_raw4, raw four motor PWM+IN1/IN2 test);

static int chassis_raw_pwm(int argc, char **argv)
{
    if (argc != 4)
    {
        rt_kprintf("Usage: chassis_raw_pwm <x_pwm> <y_pwm> <z_pwm>\n");
        rt_kprintf("Wheel order: A=left rear, B=left front, C=right front, D=right rear\n");
        rt_kprintf("Example: chassis_raw_pwm 120 0 0\n");
        return -1;
    }

    int x_pwm = clamp_pwm(atoi(argv[1]));
    int y_pwm = clamp_pwm(atoi(argv[2]));
    int z_pwm = clamp_pwm(atoi(argv[3]));

    if (x_pwm == 0 && y_pwm == 0 && z_pwm == 0)
    {
        stop_raw_pwm_all();
        rt_kprintf("chassis_raw_pwm: stop\n");
        return 0;
    }

    int pwm_a = x_pwm + y_pwm - z_pwm;
    int pwm_b = x_pwm - y_pwm - z_pwm;
    int pwm_c = x_pwm + y_pwm + z_pwm;
    int pwm_d = x_pwm - y_pwm + z_pwm;

    set_raw_pwm_all(pwm_a * motor_output_sign[MOT_A],
                    pwm_b * motor_output_sign[MOT_B],
                    pwm_c * motor_output_sign[MOT_C],
                    pwm_d * motor_output_sign[MOT_D]);

    rt_kprintf("chassis_raw_pwm: x=%d y=%d z=%d -> A=%d B=%d C=%d D=%d\n",
               x_pwm, y_pwm, z_pwm,
               raw_pwm[MOT_A], raw_pwm[MOT_B], raw_pwm[MOT_C], raw_pwm[MOT_D]);
    return 0;
}
MSH_CMD_EXPORT(chassis_raw_pwm, raw chassis PWM control);

static int motor_raw_stop(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    stop_raw_pwm_all();

    rt_kprintf("motor_raw_stop: all motors stopped\n");
    return 0;
}
MSH_CMD_EXPORT(motor_raw_stop, stop raw motor test);

static void chassis_pwm_init_thread(void *parameter)
{
    (void)parameter;
    static rt_uint32_t print_cnt = 0;

    for (int i = 0; i < PWM_DEV_NUM; i++)
    {
        pwm_dev[i] = (struct rt_device_pwm *)rt_device_find(pwm_dev_name[i]);
        if (pwm_dev[i] == RT_NULL)
        {
            rt_kprintf("Warning: not found %s\n", pwm_dev_name[i]);
            continue;
        }
        rt_pwm_enable(pwm_dev[i], PWM_CHANNEL);
        rt_pwm_set(pwm_dev[i], PWM_CHANNEL, PERIOD_10KHZ_NS, 0);
    }

    while (1)
    {
        if (STOP_FLAG)
        {
            raw_mode = RT_FALSE;
            apply_all_stop();
            rt_thread_mdelay(50);
            continue;
        }

        if (raw_mode)
        {
            for (int j = 0; j < PWM_DEV_NUM; j++)
            {
                apply_signed_motor_pwm(j, raw_pwm[j]);
            }
            rt_thread_mdelay(50);
            continue;
        }

        Drive_Motor(Move_X, Move_Y, Move_Z);

        pwm_Integral[0] = Incremental_PI_A(0, motor_rpm[0], MOTOR_A.Target, pwm_dev_name[0]);
        pwm_Integral[1] = Incremental_PI_B(1, motor_rpm[1], MOTOR_B.Target, pwm_dev_name[1]);
        pwm_Integral[2] = Incremental_PI_C(2, motor_rpm[2], MOTOR_C.Target, pwm_dev_name[2]);
        pwm_Integral[3] = Incremental_PI_D(3, motor_rpm[3], MOTOR_D.Target, pwm_dev_name[3]);

        for (int j = 0; j < MOT_MAX; j++)
        {
            apply_signed_motor_pwm(j, pwm_Integral[j] * motor_output_sign[j]);
        }

        print_cnt++;
        if (print_cnt >= 2)
        {
            print_cnt = 0;
            odometry_update_no_lock();
        }

        rt_thread_mdelay(50);
    }
}

int chassis_pwm_init(void)
{
    rt_thread_t tid = rt_thread_create("chassis_pwm_init",
                                       chassis_pwm_init_thread,
                                       RT_NULL,
                                       4096,
                                       5,
                                       10);
    if (tid)
    {
        rt_thread_startup(tid);
    }
    else
    {
        rt_kprintf("Failed to create chassis_pwm_init thread!\n");
    }

    return RT_EOK;
}

static int motor_stop(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    stop_raw_pwm_all();

    rt_kprintf("All motors have been stopped!\n");
    return 0;
}
MSH_CMD_EXPORT(motor_stop, emergency stop all motors);
