#include <rtthread.h>
#include "system.h"
#include "servo_bus.h"

#include "rt_encoder_soft.h"
#include "motor.h"

int main(int argc, char** argv)
{
    rt_kprintf("Hi, this is RT-Thread!!\n");

    return 0;
}

void chassis_car_app()
{
    systemInit();
    servo_bus_init();

    rt_kprintf("Initial chassis gpio mode!\n");
    rk_soft_encoder_init();

    rt_kprintf("Initial chassis pwm!\n");
    chassis_pwm_init();
    rt_kprintf("Initial chassis encoder!\n");
    encoder_init();

    rt_kprintf("Initial chassis speed by default!\n");
    set_chassis_target(0, 0, 0);

    rt_kprintf("ROS CAR IS STARTTING...\n");
}
MSH_CMD_EXPORT(chassis_car_app, chassis car app init);
