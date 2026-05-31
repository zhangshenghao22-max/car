/*
 * Copyright (c) 2006-2024, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-10-27     GuEe-GUI     first version
 */

#ifndef __IVSHMEM_H__
#define __IVSHMEM_H__

#include <rtdef.h>

#include <drivers/pci.h>

/* Registers */
#define IVSHMEM_INTR_MASK       0
#define IVSHMEM_INTR_STATUS     4
#define IVSHMEM_IV_POSITION     8
#define IVSHMEM_DOORBELL        12
#define IVSHMEM_REG_2_ID        0x10
#define IVSHMEM_REG_2_MAX_PEERS 0x14
#define IVSHMEM_REG_2_INT_CTRL  0x18
#define IVSHMEM_REG_2_DOORBELL  0x1c
#define IVSHMEM_REG_2_STATE     0x20
#define IVSHMEM_REG_2_STATE_TBL 0x24

/* BAR */
#define IVSHMEM_REG_BAR         0
#define IVSHMEM_MSIX_BAR        1
#define IVSHMEM_SHARE_MEM_BAR   2

#define IVSHMEM_MAX_VECTORS     255

/* JailHouse support */
#define IVSHMEM_JAILHOUSE_SHMEM_ADDR_LO 0x40
#define IVSHMEM_JAILHOUSE_SHMEM_ADDR_HI 0x44
#define IVSHMEM_JAILHOUSE_SHMEM_SIZE_LO 0x48
#define IVSHMEM_JAILHOUSE_SHMEM_SIZE_HI 0x4c

#define RT_DEVICE_CTRL_IVSHMEM_FLUSH            (0x40)
#define RT_DEVICE_CTRL_IVSHMEM_INVALIDATE       (0x41)
#define RT_DEVICE_CTRL_IVSHMEM_SEND_NOTIFY      (0x42)
#define RT_DEVICE_CTRL_IVSHMEM_WAIT_NOTIFY      (0x43)
#define RT_DEVICE_CTRL_IVSHMEM_SIZE             (0x44)

struct ivshmem_cmd
{
    rt_uint32_t is_read;
    rt_uint32_t reg_off;
    rt_uint32_t value;
};

struct ivshmem_device
{
    struct rt_device parent;
    struct rt_pci_device *dev;

    void *reg;
    int irq;

    void *shmem_pa;
    void *shmem;
    rt_size_t shmem_size;

    int nvectors;
    struct rt_pci_msix_entry *msix_entries;

    rt_err_t (*handle_irq)(struct ivshmem_device *, int irq);
};

#define raw_to_ivshmem(raw) rt_container_of(raw, struct ivshmem_device, parent)

rt_inline rt_uint32_t ivshmem_doorbell(int peer_id, int vector_index)
{
    return (rt_uint32_t)((peer_id << 16) | (vector_index & IVSHMEM_MAX_VECTORS));
}

rt_err_t ivshmem_install_msix_vectors(struct ivshmem_device *ivshmem_dev, int nvectors, char *name);
rt_err_t ivshmem_install_intx_vector(struct ivshmem_device *ivshmem_dev, char *name);

rt_err_t ivshmem_pci_probe(struct ivshmem_device *ivshmem_dev,
        struct rt_pci_device *dev);
struct ivshmem_device *ivshmem_pci_remove(struct rt_pci_device *dev);

#endif /* __IVSHMEM_H__ */
