#include <rtthread.h>
#include <rtdevice.h>
#include <dt-bindings-pinctrl.h>
#include "rk_gpio_ctrl.h"

/*
 * Wheel position mapping used by the RT chassis layer:
 * A = left rear, B = left front, C = right front, D = right rear.
 * The motor driver board uses PWM + two direction inputs for each wheel.
 */
static const rt_base_t motor_pins[MOT_MAX][2] = {
    {PIN_AIN1, PIN_AIN2},
    {PIN_BIN2, PIN_BIN1},
    {PIN_CIN1, PIN_CIN2},
    {PIN_DIN2, PIN_DIN1}
};

void motor_set_direction(motor_id_t id, motor_dir_t dir)
{
    if (id >= MOT_MAX) return;

    rt_base_t in1 = motor_pins[id][0];
    rt_base_t in2 = motor_pins[id][1];

    switch (dir)
    {
        case MOTOR_FORWARD:
            rt_pin_write(in1, PIN_HIGH);
            rt_pin_write(in2, PIN_LOW);
            break;
        case MOTOR_REVERSE:
            rt_pin_write(in1, PIN_LOW);
            rt_pin_write(in2, PIN_HIGH);
            break;
        case MOTOR_STOP:
        default:
            rt_pin_write(in1, PIN_LOW);
            rt_pin_write(in2, PIN_LOW);
            break;
    }
}

void motor_apply_pwm_dir(motor_id_t id, int signed_pwm)
{
    if (id >= MOT_MAX) return;

    if (signed_pwm > 0)
    {
        motor_set_direction(id, MOTOR_FORWARD);
    }
    else if (signed_pwm < 0)
    {
        motor_set_direction(id, MOTOR_REVERSE);
    }
    else
    {
        motor_set_direction(id, MOTOR_STOP);
    }
}

void init_io(void)
{
    for (int i = 0; i < MOT_MAX; i++)
    {
        rt_pin_mode(motor_pins[i][0], PIN_MODE_OUTPUT);
        rt_pin_mode(motor_pins[i][1], PIN_MODE_OUTPUT);
        motor_set_direction((motor_id_t)i, MOTOR_STOP);
    }
}
