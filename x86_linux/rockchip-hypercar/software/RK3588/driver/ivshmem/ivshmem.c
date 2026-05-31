/*
 * Copyright (c) 2006-2024, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-10-27     GuEe-GUI     first version
 */

#include <rtthread.h>

#define DBG_TAG "drv.ivshmem"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#include <rthw.h>
#include <cpuport.h>
#include <ioremap.h>

#include <ivshmem.h>
#include "ivshmem_pci_class.h"

static void ivshmem_isr(int irqno, void *param)
{
    rt_uint32_t status;
    struct ivshmem_device *ivshmem_dev = param;

    status = HWREG32(ivshmem_dev->reg + IVSHMEM_INTR_STATUS);

    if ((status && status != 0xffffffff) || ivshmem_dev->nvectors)
    {
        if (ivshmem_dev->handle_irq)
        {
            ivshmem_dev->handle_irq(ivshmem_dev, ivshmem_dev->nvectors ? irqno : status);
        }
        else
        {
            LOG_E("IVSHMEM<%p> have a irq = %d, by no have irq handers", ivshmem_dev, irqno);
        }
    }
}

#if 0
rt_err_t ivshmem_install_msix_vectors(struct ivshmem_device *ivshmem_dev, int nvectors, char *name)
{
    rt_err_t ret = RT_EOK;

    do {
        char irq_name[RT_NAME_MAX];

        if (!ivshmem_dev || !name)
        {
            ret = -RT_EINVAL;
            break;
        }

        if (ivshmem_dev->nvectors != 0)
        {
            LOG_E("IVSHMEM<%p> '%s' have already install msix", ivshmem_dev, name);
            ret = -RT_ERROR;
            break;
        }

        if (!ivshmem_dev->handle_irq)
        {
            LOG_E("IVSHMEM<%p> '%s' no have irq handers", ivshmem_dev, name);
            ret = -RT_EINVAL;
            break;
        }

        ivshmem_dev->msix_entries = rt_malloc(nvectors * sizeof(*ivshmem_dev->msix_entries));

        if (!ivshmem_dev->msix_entries)
        {
            ret = -RT_ENOMEM;
            break;
        }

        ivshmem_dev->nvectors = nvectors;

        for (int i = 0; i < nvectors; ++i)
        {
            ivshmem_dev->msix_entries[i].entry = i;
        }

        ret = pci_enable_msix_exact(ivshmem_dev->dev, ivshmem_dev->msix_entries, ivshmem_dev->nvectors);

        if (ret)
        {
            rt_free(ivshmem_dev->msix_entries);
            ivshmem_dev->nvectors = 0;

            break;
        }

        for (int i = 0; i < nvectors; i++)
        {
            int irq = ivshmem_dev->msix_entries[i].vector;

            rt_snprintf(irq_name, RT_NAME_MAX, "%s-%d", name, irq);

            rt_hw_interrupt_install(irq, ivshmem_isr, ivshmem_dev, irq_name);
            rt_hw_interrupt_umask(irq);
        }
    } while (0);

    return ret;
}
#else
rt_err_t ivshmem_install_msix_vectors(struct ivshmem_device *ivshmem_dev, int nvectors, char *name)
{
    return -RT_ENOSYS;

}
#endif

rt_err_t ivshmem_install_intx_vector(struct ivshmem_device *ivshmem_dev, char *name)
{
    rt_err_t ret = RT_EOK;

    do {
        if (!ivshmem_dev || !name)
        {
            ret = -RT_EINVAL;
            break;
        }

        if (ivshmem_dev->nvectors != 0)
        {
            LOG_E("IVSHMEM<%p> '%s' have already install msix", ivshmem_dev, name);
            ret = -RT_ERROR;
            break;
        }

        if (!ivshmem_dev->handle_irq)
        {
            LOG_E("IVSHMEM<%p> '%s' no have irq handers", ivshmem_dev, name);
            ret = -RT_EINVAL;
            break;
        }

        ivshmem_dev->nvectors = 0;
        ivshmem_dev->msix_entries = RT_NULL;

        rt_hw_interrupt_install(ivshmem_dev->irq, ivshmem_isr, ivshmem_dev, name);
        rt_pci_intx(ivshmem_dev->dev, RT_TRUE);
        rt_hw_interrupt_umask(ivshmem_dev->irq);
    } while (0);

    return ret;
}

static void ivshmem_free_resource(struct ivshmem_device *ivshmem_dev)
{
    if (ivshmem_dev->dev)
    {
        #if 0
        ivshmem_dev->dev->dev = RT_NULL;
        #endif
        ivshmem_dev->dev->sysdata = RT_NULL;
    }
    if (ivshmem_dev->reg)
    {
        rt_iounmap(ivshmem_dev->reg);
    }
    if (ivshmem_dev->shmem)
    {
        rt_iounmap(ivshmem_dev->shmem);
    }
}

rt_err_t ivshmem_pci_probe(struct ivshmem_device *ivshmem_dev,
        struct rt_pci_device *dev)
{
    rt_err_t ret = RT_EOK;

    do {
        if (!ivshmem_dev || !dev || dev->irq == -1)
        {
            ret = -RT_EINVAL;
            break;
        }

        rt_memset(ivshmem_dev, 0, sizeof(*ivshmem_dev));

        ivshmem_dev->dev = dev;
        ivshmem_dev->irq = dev->irq;

        #if 0
        dev->dev = &ivshmem_dev->parent;
        #endif
        dev->sysdata = ivshmem_dev;

        ivshmem_dev->reg = rt_ioremap((void *)dev->resource[IVSHMEM_REG_BAR].base,
                dev->resource[IVSHMEM_REG_BAR].size);

        if (!ivshmem_dev->reg)
        {
            ret = -RT_ERROR;
            break;
        }

        /* Set all masks to on */
        HWREG32(ivshmem_dev->reg + IVSHMEM_INTR_MASK) = 0xffffffff;

        ivshmem_dev->shmem_size = dev->resource[IVSHMEM_SHARE_MEM_BAR].size;
        ivshmem_dev->shmem = (void *)dev->resource[IVSHMEM_SHARE_MEM_BAR].base;

        if (!ivshmem_dev->shmem_size)
        {
            rt_pci_read_config_u32(dev, IVSHMEM_JAILHOUSE_SHMEM_ADDR_LO, (rt_uint32_t *)&ivshmem_dev->shmem_pa);
            rt_pci_read_config_u32(dev, IVSHMEM_JAILHOUSE_SHMEM_ADDR_HI, (rt_uint32_t *)&ivshmem_dev->shmem_pa + 1);
            rt_pci_read_config_u32(dev, IVSHMEM_JAILHOUSE_SHMEM_SIZE_LO, (rt_uint32_t *)&ivshmem_dev->shmem_size);
            rt_pci_read_config_u32(dev, IVSHMEM_JAILHOUSE_SHMEM_SIZE_HI, (rt_uint32_t *)&ivshmem_dev->shmem_size + 1);
        }

        if (dev->class == (PCI_BASE_CLASS_NETWORK << 16))
        {
#ifdef RT_USING_IVSHMEM_NET_CACHE
            ivshmem_dev->shmem = rt_ioremap_cached(ivshmem_dev->shmem_pa, ivshmem_dev->shmem_size);
#else
            ivshmem_dev->shmem = rt_ioremap(ivshmem_dev->shmem_pa, ivshmem_dev->shmem_size);
#endif
        }
        else
        {
#ifdef RT_USING_IVSHMEM_RAW_CACHE
            ivshmem_dev->shmem = rt_ioremap_cached(ivshmem_dev->shmem_pa, ivshmem_dev->shmem_size);
#else
            ivshmem_dev->shmem = rt_ioremap(ivshmem_dev->shmem_pa, ivshmem_dev->shmem_size);
#endif
        }

        if (!ivshmem_dev->shmem)
        {
            ret = -RT_ERROR;
            break;
        }
    } while (0);

    if (ret && ivshmem_dev)
    {
        ivshmem_free_resource(ivshmem_dev);
    }

    return ret;
}

extern void rt_hw_interrupt_uninstall(int vector, rt_isr_handler_t handler, void *param);
struct ivshmem_device *ivshmem_pci_remove(struct rt_pci_device *dev)
{
    struct ivshmem_device *ret = RT_NULL;

    if (dev && dev->sysdata)
    {
        ret = dev->sysdata;

        if (ret->msix_entries)
        {
            for (int i = 0; i < ret->nvectors; ++i)
            {
                int irq = ret->msix_entries[i].irq;

                rt_hw_interrupt_umask(irq);
                rt_hw_interrupt_uninstall(irq, ivshmem_isr, ret);
            }
        }

        rt_hw_interrupt_umask(ret->irq);
        rt_hw_interrupt_uninstall(ret->irq, ivshmem_isr, ret);

        if (ret->nvectors)
        {
            #if 0
            pci_disable_msix(ret->dev);
            #endif
            rt_free(ret->msix_entries);
        }
        else
        {
            rt_pci_intx(ret->dev, RT_FALSE);
        }
        ivshmem_free_resource(ret);
    }

    return ret;
}
