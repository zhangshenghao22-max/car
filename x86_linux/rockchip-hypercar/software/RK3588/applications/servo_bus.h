#ifndef __SERVO_BUS_H__
#define __SERVO_BUS_H__

#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

rt_err_t servo_bus_init(void);
rt_err_t servo_bus_send_raw(const char *payload);
rt_err_t servo_bus_send_raw_len(const char *payload, rt_size_t len);
rt_err_t servo_bus_send_set(int id, int pwm, int duration_ms);
rt_err_t servo_bus_home(void);
void servo_bus_diag(void);

#ifdef __cplusplus
}
#endif

#endif /* __SERVO_BUS_H__ */
