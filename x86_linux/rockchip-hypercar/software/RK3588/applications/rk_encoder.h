#ifndef __ENCODER_H_
#define __ENCODER_H_

#include <rtthread.h>
#include <rtdevice.h>
#include <rt_encoder_soft.h>
#include "robot_init.h"
#include "system.h"
#include <dt-bindings-pinctrl.h>

extern rt_int32_t last_count[4];
extern double motor_rpm[4];

void encoder_init(void);

#endif