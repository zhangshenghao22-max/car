/*
 * Copyright (c) 2006-2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-3-08      GuEe-GUI     the first version
 */
#include <setup.h>
#include <board.h>
#include <psci.h>

void rt_hw_board_init(void)
{
    rt_hw_common_setup();
}

void reboot(void)
{
    psci_system_reboot();
}
MSH_CMD_EXPORT(reboot, reboot...);

