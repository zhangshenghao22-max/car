#ifndef __RT_ENCODER_H__
#define __RT_ENCODER_H__

#include <rtthread.h>
#include <system.h>

typedef enum
{
    WHEEL_LF, //左前
    WHEEL_RF, //右前
    WHEEL_LB, //左后
    WHEEL_RB, //右后
    WHEEL_MAX
}wheel_pos_t;

typedef enum {
    STOP_WHEEL, // 停转
    FORWARD,    // 正向
    REVERSAL    // 反向
} encoder_dir_t;

int rk_soft_encoder_init(void);
void rk_soft_encoder_update(void);
rt_err_t rk_soft_encoder_pulse(wheel_pos_t pos, rt_int32_t *pulse);
rt_err_t rk_soft_encoder_get_dir(wheel_pos_t pos, encoder_dir_t *dir);

#endif /* __RT_ENCODER_H__ */
