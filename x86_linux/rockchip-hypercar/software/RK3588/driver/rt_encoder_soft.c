#include <rtthread.h>
#include <rtdevice.h>
#include <rt_encoder_soft.h>
#include <dt-bindings-pinctrl.h>

#define E1_A      RK_GPIO1_B2
#define E1_B      RK_GPIO1_B3
#define E2_A      RK_GPIO1_B7
#define E2_B      RK_GPIO1_D7
#define E3_A      RK_GPIO3_A4
#define E3_B      RK_GPIO3_B5
#define E4_A      RK_GPIO4_C4
#define E4_B      RK_GPIO4_C5

/*-----------------------------------------------
 * 车轮结构体
 *----------------------------------------------*/
struct wheel_condition
{
    char          name[8];         // 名字：A/B/C/D （实际打印时会映射为左右前后）
    rt_uint16_t   pin[2];          // pin[0]=A相, pin[1]=B相
    rt_int32_t    count;           // 实时脉冲计数（2倍频后）
    volatile long long temp_count; // 临时脉冲计数（2倍频后）
    encoder_dir_t direction;       // 方向: 1 正转, -1 反转, 0 停止
};

static struct wheel_condition car_wheel[WHEEL_MAX] =
{
    { .name = "A", .pin = { E1_A, E1_B }, .count = 0, .temp_count = 0, .direction = STOP_WHEEL },
    { .name = "B", .pin = { E2_A, E2_B }, .count = 0, .temp_count = 0, .direction = STOP_WHEEL },
    { .name = "C", .pin = { E3_A, E3_B }, .count = 0, .temp_count = 0, .direction = STOP_WHEEL },
    { .name = "D", .pin = { E4_A, E4_B }, .count = 0, .temp_count = 0, .direction = STOP_WHEEL },
};

static int cnt = 0;

static void encoder_gpio_irq(void *args)
{
    struct wheel_condition *w = (struct wheel_condition *)args;

    rt_uint8_t b = rt_pin_read(w->pin[1]);
    rt_uint8_t a = rt_pin_read(w->pin[0]);
    
    if (a == b) {
        w->temp_count--;
    } else {
        w->temp_count++;
    }

    // rt_kprintf("[%d]irq!\n", cnt++);
}

void rk_soft_encoder_update(void)
{
    for (int i = 0; i < WHEEL_MAX; i++)
    {
        car_wheel[i].count = car_wheel[i].temp_count;
        car_wheel[i].direction =  ( car_wheel[i].count > 0 ) ? FORWARD : REVERSAL;
        if( car_wheel[i].count == 0 ) car_wheel[i].direction = STOP_WHEEL;
        car_wheel[i].temp_count = 0;
    }
}

rt_err_t rk_soft_encoder_pulse(wheel_pos_t pos, rt_int32_t *pulse)
{
    rt_base_t leave;
    if (pos >= WHEEL_MAX || pulse == RT_NULL)
        return RT_ERROR;

    leave = rt_hw_interrupt_disable();
    *pulse = car_wheel[pos].count;
    rt_hw_interrupt_enable(leave);
    return RT_EOK;
}

rt_err_t rk_soft_encoder_get_dir(wheel_pos_t pos, encoder_dir_t *dir)
{
    rt_base_t leave;
    if (pos >= WHEEL_MAX || dir == RT_NULL)
        return RT_ERROR;

    leave = rt_hw_interrupt_disable();
    *dir = car_wheel[pos].direction;
    rt_hw_interrupt_enable(leave);
    return RT_EOK;
}

int rk_soft_encoder_init(void)
{
    for (int i = 0; i < WHEEL_MAX; i++)
    {
        struct wheel_condition *w = &car_wheel[i];

        rt_pin_mode(w->pin[0], PIN_MODE_INPUT_PULLDOWN);
        rt_pin_mode(w->pin[1], PIN_MODE_INPUT_PULLDOWN);

        rt_pin_attach_irq(w->pin[0], PIN_IRQ_MODE_RISING, encoder_gpio_irq, (void *)w);
        rt_pin_irq_enable(w->pin[0], RT_TRUE);
    }

    rt_kprintf("2x Quadrature Encoder (A phase bilateral edge interruption) + Speed Timer initialized OK.\n");

    return RT_EOK;
}
INIT_COMPONENT_EXPORT(rk_soft_encoder_init);
