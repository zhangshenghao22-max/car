/*
 * Copyright (c) 2006-2024, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-10-27     GuEe-GUI     first version
 */

#include <rtthread.h>

#define DBG_TAG "ivshmem.net"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#include <cpuport.h>

/* #include <qrand.h> */
#include <rtatomic.h>
#include <ivshmem.h>
/* #include <libdriver.h> */
#include <ipc/workqueue.h>
#include <dt-bindings/size.h>

#ifdef RT_USING_LWIP
#include <lwip/inet.h>
#include <netif/ethernetif.h>
#include <lwip/netifapi.h>
#endif

#include "ivshmem_pci_class.h"
#include "ivshmem_virtio_queue.h"

#define IVSHMEM_NET_STATE_RESET 1
#define IVSHMEM_NET_STATE_BCST  2
#define IVSHMEM_NET_STATE_LINK  3
#define IVSHMEM_NET_STATE_INIT  4
#define IVSHMEM_NET_STATE_READY 5
#define IVSHMEM_NET_STATE_RUN   6

#define IVSHMEM_NET_MTU_MIN     256
#define IVSHMEM_NET_MTU_MAX     65535
#define IVSHMEM_NET_MTU_DEF     16384

#define IVSHMEM_NET_STATE_IRQ   0
#define IVSHMEM_NET_RX_IRQ      0
#define IVSHMEM_NET_TX_IRQ      0

#define IVSHMEM_NET_VQ_ALIGN    64

#define IVSHMEM_NET_STUB_MAX_NR RT_UINT16_MAX
#define IVSHMEM_NET_STUB_MAGIC  0x54454e76  /* "vNET" */

#ifndef SMP_CACHE_BYTES
#define L1_CACHE_SHIFT  6
#define L1_CACHE_BYTES  (1 << L1_CACHE_SHIFT)
#define SMP_CACHE_BYTES L1_CACHE_BYTES
#endif

#define IVSHMEM_NET_FRAME_SIZE(s)   RT_ALIGN(18 + (s), SMP_CACHE_BYTES)

#define _KB 1024

#define _min(x, y) ({        \
    typeof(x) _x = (x);     \
    typeof(y) _y = (y);     \
    (void) (&_x == &_y);    \
    _x < _y ? _x : _y; })

#define _max(x, y) ({        \
    typeof(x) _x = (x);     \
    typeof(y) _y = (y);     \
    (void) (&_x == &_y);    \
    _x > _y ? _x : _y; })

#define _swap(a, b) do { typeof(a) __tmp = (a); (a) = (b); (b) = __tmp; } while (0)

struct ivshmem_net_queue
{
    struct virtq queue;
    rt_uint32_t free_head;
    rt_uint32_t num_free;
    rt_uint16_t last_avail_idx;
    rt_uint16_t last_used_idx;

    void *data;
    void *end;
    rt_uint32_t size;
    rt_uint32_t head;
    rt_uint32_t tail;
};

struct ivshmem_net_stub
{
    rt_uint32_t magic;
    rt_uint32_t ack_ivpos;
    rt_uint32_t org_state;
    rt_uint32_t pad;
} rt_packed;

struct ivshmem_net
{
    struct ivshmem_device parent;
    struct eth_device ndev;

    struct rt_workqueue *state_wq;
    struct rt_work state_work;

    struct
    {
        rt_uint32_t intxctrl;
        rt_uint32_t istat;
        rt_uint32_t ivpos;
        rt_uint32_t doorbell;
    } *regs;

    struct ivshmem_net_stub *self, *peer;

    rt_uint32_t vqsize;
    rt_uint32_t qlen;
    rt_uint32_t qsize;

    rt_bool_t broken;
    rt_atomic_t lstate;
    rt_atomic_t rstate;

    void *shm;
    rt_size_t shm_size;

    rt_uint32_t mtu;
    rt_uint8_t mac[6];

    rt_uint32_t ivpos;
    rt_uint32_t peer_id;

    struct ivshmem_net_queue rx;
    struct ivshmem_net_queue tx;

    struct rt_spinlock tx_free_lock;
    struct rt_spinlock tx_clean_lock;

    rt_uint32_t received;
};

#define raw_to_ivshmem_net(raw) rt_container_of(raw, struct ivshmem_net, ndev.parent)

static void ivshmem_net_set_state(struct ivshmem_net *in, rt_uint32_t state);

rt_inline rt_base_t ivshmem_spin_lock_irqsave(struct rt_spinlock *lock)
{
    rt_base_t level = rt_hw_local_irq_disable();
    rt_hw_spin_lock(&lock->lock);

    return level;
}

rt_inline void ivshmem_spin_unlock_irqrestore(struct rt_spinlock *lock, rt_base_t level)
{
    rt_hw_spin_unlock(&lock->lock);
    rt_hw_local_irq_enable(level);
}

rt_inline void ivshmem_flush_dcache_range(void *addr, int len)
{
    rt_hw_cpu_dcache_ops(RT_HW_CACHE_FLUSH, addr, len);
}

rt_inline void ivshmem_inval_dcache_range(void *addr, int len)
{
    rt_hw_cpu_dcache_ops(RT_HW_CACHE_INVALIDATE, addr, len);
}

static rt_uint32_t qrand32(rt_uint32_t seed)
{
    rt_uint32_t ret;
    static rt_uint32_t next = 1;

    next = next * 1103515245 + 12345;

    ret = (rt_uint32_t)(next / 65536) % 32768;

    if (seed)
    {
        ret *= seed;
    }

    return ret;
}

rt_inline void ivshmem_net_notify_vector(struct ivshmem_net *in, int vector)
{
    HWREG32(&in->regs->doorbell) = ivshmem_doorbell(in->peer_id, vector);
}

static struct ivshmem_net_stub *ivshmem_net_get_my_stub(struct ivshmem_net *in)
{
    if (in == RT_NULL)
    {
        return RT_NULL;
    }

    if ((in->ivpos != 0) && (in->ivpos != 1))
{
        return RT_NULL;
    }

    return (struct ivshmem_net_stub *)((rt_uint8_t *)in->shm + in->shm_size + sizeof(struct ivshmem_net_stub) * in->ivpos);
}

static struct ivshmem_net_stub *ivshmem_net_get_remote_stub(struct ivshmem_net *in)
{
    if (in == RT_NULL)
    {
        return RT_NULL;
    }

    if ((in->ivpos != 0) && (in->ivpos != 1))
    {
        return RT_NULL;
    }

    return (struct ivshmem_net_stub *)((rt_uint8_t *)in->shm + in->shm_size + sizeof(struct ivshmem_net_stub) * (1- in->ivpos));
}

static void ivshmem_net_init_stub(struct ivshmem_net *in)
{
    /* save the net stub for a long time */
    in->self->magic = IVSHMEM_NET_STUB_MAGIC;

    in->self->org_state = rt_atomic_load(&in->lstate);

}

static void ivshmem_net_check_remote(struct ivshmem_net *in)
{
#define IVSHMEM_NET_CHECK_REMOTE_TIMES    (10)
    rt_uint32_t peer_id = in->peer_id;
    int i;

    rt_hw_dmb();

    in->peer_id = 1 - in->ivpos;

    rt_hw_dmb();

    ivshmem_net_set_state(in, IVSHMEM_NET_STATE_RESET);

    rt_hw_dmb();

    in->peer_id = peer_id;

    for (i = 0; i < IVSHMEM_NET_CHECK_REMOTE_TIMES; i++)
{
        struct ivshmem_net_stub *peer_stub
            = ivshmem_net_get_remote_stub(in);

        if (peer_stub->org_state == IVSHMEM_NET_STATE_RESET)
        {
            break;
        }

        rt_thread_mdelay(1);
    }

    return;
#undef IVSHMEM_NET_CHECK_REMOTE_TIMES
}

static void ivshmem_net_reset(struct ivshmem_net *in)
{
    struct ivshmem_net_stub *self = ivshmem_net_get_my_stub(in);

    self->magic = IVSHMEM_NET_STUB_MAGIC;
    self->ack_ivpos = RT_UINT16_MAX;

    in->self = self;

    rt_atomic_store(&in->lstate, IVSHMEM_NET_STATE_RESET);
    rt_atomic_store(&in->rstate, IVSHMEM_NET_STATE_RESET);

    in->peer_id = RT_UINT16_MAX;
}

static void ivshmem_net_broadcast(struct ivshmem_net *in)
{
    struct ivshmem_net_stub *peer = ivshmem_net_get_remote_stub(in);

    /* now we are in broadcast state */
    rt_atomic_store(&in->lstate, IVSHMEM_NET_STATE_BCST);

    if (peer->magic == IVSHMEM_NET_STUB_MAGIC)
    {
        /* ack the peer */
        in->self->ack_ivpos = in->ivpos;

        in->peer = peer;
        in->peer_id = 1 - in->ivpos;

        rt_atomic_store(&in->lstate, IVSHMEM_NET_STATE_LINK);

        rt_hw_wmb();

        ivshmem_net_notify_vector(in, IVSHMEM_NET_STATE_IRQ);
    }
    else
    {
        /* cancel ack */
        in->self->ack_ivpos = RT_UINT16_MAX;

        rt_atomic_store(&in->lstate, IVSHMEM_NET_STATE_RESET);
        rt_hw_wmb();
    }
}

static void *ivshmem_net_desc_data(struct ivshmem_net *in, struct ivshmem_net_queue *q,
        struct virtq_desc *desc, rt_uint32_t *len)
{
    void *data = RT_NULL;
    rt_uint64_t offs = 0UL;
    rt_uint32_t dlen = 0;
    rt_uint16_t flags = 0;

    offs = desc->addr;
    dlen = desc->len;
    flags = desc->flags;

    do {
        if (flags)
        {
            break;
        }

        if (offs >= in->shm_size)
        {
            break;
        }

        data = (rt_uint8_t *)in->shm + offs;

        if (data < q->data || data >= q->end)
        {
            data = RT_NULL;
            break;
        }

        if (dlen > q->end - data)
        {
            data = RT_NULL;
            break;
        }

        *len = dlen;
    } while (0);

    return data;
}

static void ivshmem_net_init_queue(struct ivshmem_net *in, struct ivshmem_net_queue *q, void *mem, rt_uint32_t len)
{
    struct virtq *vq = &q->queue;

    rt_memset(q, 0, sizeof(*q));

    vq->num = len;
    vq->desc = mem;
    vq->avail = (struct virtq_avail *)((char *)mem + VIRTQ_DESC_TOTAL_SIZE(len));
    vq->used = (void *)RT_ALIGN((rt_ubase_t)&vq->avail->ring[len] + VIRTQ_AVAIL_RES_SIZE, IVSHMEM_NET_VQ_ALIGN);

    q->data = (rt_uint8_t *)mem + in->vqsize;
    q->end = (rt_uint8_t *)q->data + in->qsize;
    q->size = in->qsize;
}

static void ivshmem_net_init_queues(struct ivshmem_net *in)
{
    void *tx, *rx;

    if (in->ivpos < in->peer_id)
    {
        tx = in->shm;
        rx = (rt_uint8_t *)in->shm + in->shm_size / 2;
    }
    else
    {
        rx = in->shm;
        tx = (rt_uint8_t *)in->shm + in->shm_size / 2;
    }

    rt_memset(tx, 0, in->shm_size / 2);

    ivshmem_net_init_queue(in, &in->rx, rx, in->qlen);
    ivshmem_net_init_queue(in, &in->tx, tx, in->qlen);

    _swap(in->rx.queue.used, in->tx.queue.used);

    in->tx.num_free = in->tx.queue.num;

    for (int i = 0; i < in->tx.queue.num - 1; ++i)
    {
        in->tx.queue.desc[i].next = i + 1;
    }
}

static rt_err_t ivshmem_net_calc_qsize(struct ivshmem_net *in)
{
    rt_err_t ret = RT_EOK;

    do {
        rt_uint32_t qlen, qsize, vqsize;

        for (qlen = 4096; qlen > 32; qlen >>= 1)
        {
            #if 1
            vqsize = RT_ALIGN(VIRTQ_DESC_TOTAL_SIZE(qlen) + VIRTQ_AVAIL_TOTAL_SIZE(qlen),
                    IVSHMEM_NET_VQ_ALIGN) + VIRTQ_USED_TOTAL_SIZE(qlen);
            vqsize = RT_ALIGN(vqsize, IVSHMEM_NET_VQ_ALIGN);
            #else
            vqsize = virtq_size(RT_NULL, qlen, IVSHMEM_NET_VQ_ALIGN);
            #endif

            if (vqsize < in->shm_size / 16)
            {
                break;
            }
        }

        if (vqsize > in->shm_size / 2)
        {
            ret = -RT_EINVAL;
            break;
        }

        qsize = in->shm_size / 2 - vqsize;

        if (qsize < 4 * IVSHMEM_NET_MTU_MIN)
        {
            ret = -RT_EINVAL;
            break;
        }

        in->vqsize = vqsize;
        in->qlen = qlen;
        in->qsize = qsize;
    } while (0);

    return ret;
}

static void ivshmem_net_enable_tx_irq(struct ivshmem_net *in)
{
    struct virtq *vq = &in->tx.queue;

    vq->avail->ring[vq->num] = in->tx.last_used_idx;
    rt_hw_wmb();
}

static void ivshmem_net_notify_tx(struct ivshmem_net *in, unsigned int num)
{
    rt_uint16_t evt, old, new;
    struct virtq *vq = &in->tx.queue;
    rt_uint16_t *pevt = RT_NULL;

    rt_hw_dmb();
    pevt = (rt_uint16_t *)&(vq->used->ring[vq->num]);
    evt = *pevt;
    old = in->tx.last_avail_idx - num;
    new = in->tx.last_avail_idx;

    if (virtq_need_event(evt, new, old))
    {
        ivshmem_net_notify_vector(in, IVSHMEM_NET_RX_IRQ);
    }
}

static void ivshmem_net_enable_rx_irq(struct ivshmem_net *in)
{
    struct virtq *vq = &in->rx.queue;
    rt_uint16_t *pevt = (rt_uint16_t *)&(vq->used->ring[vq->num]);

    *pevt = in->rx.last_avail_idx;
    rt_hw_wmb();
}

static void ivshmem_net_notify_rx(struct ivshmem_net *in, unsigned int num)
{
    rt_uint16_t evt, old, new;
    struct virtq *vq = &in->rx.queue;

    rt_hw_dmb();

    evt = vq->avail->ring[vq->num];
    old = in->rx.last_used_idx - num;
    new = in->rx.last_used_idx;

    if (virtq_need_event(evt, new, old))
    {
        ivshmem_net_notify_vector(in, IVSHMEM_NET_TX_IRQ);
    }
}

rt_used
static rt_bool_t ivshm_net_rx_avail(struct ivshmem_net *in)
{
    struct ivshmem_net_queue *rx = &in->rx;
    struct virtq *vq = &rx->queue;
    rt_uint16_t avail_idx;

    avail_idx = vq->avail->idx;

    rt_hw_dmb();

    return avail_idx != rx->last_avail_idx;
}

static rt_size_t ivshmem_net_tx_space(struct ivshmem_net *in)
{
    rt_uint32_t space, tail, head;
    struct ivshmem_net_queue *tx = &in->tx;

    tail = tx->tail;
    head = tx->head;

    if (head < tail)
    {
        space = tail - head;
    }
    else
    {
        space = _max(tx->size - head, tail);
    }

    return space;
}

static rt_bool_t ivshmem_net_tx_ok(struct ivshmem_net *in, rt_uint32_t mtu)
{
    return in->tx.num_free >= 2 && ivshmem_net_tx_space(in) >= 2 * IVSHMEM_NET_FRAME_SIZE(mtu);
}

static rt_uint32_t ivshmem_net_tx_advance(struct ivshmem_net_queue *q, rt_uint32_t *pos, rt_uint32_t len)
{
    rt_uint32_t p = *pos;

    len = IVSHMEM_NET_FRAME_SIZE(len);

    if (q->size - p < len)
    {
        p = 0;
    }

    *pos = p + len;

    return p;
}

static void ivshmem_net_tx_clean(struct ivshmem_net *in)
{
    rt_uint16_t used_idx, last;
    rt_uint32_t num = 0, fhead = 0;

    struct virtq *vq;
    struct virtq_used_elem *used;
    struct virtq_desc *desc = RT_NULL;
    struct virtq_desc *fdesc = RT_NULL;
    struct ivshmem_net_queue *tx = &in->tx;
    rt_base_t level = ivshmem_spin_lock_irqsave(&in->tx_clean_lock);

    vq = &tx->queue;

    used_idx = vq->used->idx;
    rt_hw_dmb();

    last = tx->last_used_idx;

    while (last != used_idx)
    {
        void *data;
        rt_uint32_t len, tail;

        used = vq->used->ring + (last % vq->num);
        if (used->id >= vq->num || used->len != 1)
        {
            LOG_E("%s: invalid tx used->id %d ->len %d", in->ndev.parent.parent.name, used->id, used->len);
            break;
        }

        desc = &vq->desc[used->id];
        data = ivshmem_net_desc_data(in, &in->tx, desc, &len);

        if (!data)
        {
            LOG_E("%s: bad tx descriptor, data = NULL", in->ndev.parent.parent.name);
            break;
        }

        tail = ivshmem_net_tx_advance(tx, &tx->tail, len);

        if (data != ((rt_uint8_t *)tx->data + tail))
        {
            LOG_E("%s: bad tx descriptor", in->ndev.parent.parent.name);
            break;
        }

        if (!num)
        {
            fdesc = desc;
        }
        else
        {
            desc->next = fhead;
        }

        fhead = used->id;
        last++;
        num++;
    }

    tx->last_used_idx = last;

    ivshmem_spin_unlock_irqrestore(&in->tx_clean_lock, level);

    if (num)
    {
        level = ivshmem_spin_lock_irqsave(&in->tx_free_lock);

        fdesc->next = tx->free_head;
        tx->free_head = fhead;
        tx->num_free += num;
        RT_ASSERT(tx->num_free <= vq->num);

        ivshmem_spin_unlock_irqrestore(&in->tx_free_lock, level);
    }
}

static struct virtq_desc *ivshmem_net_rx_desc(struct ivshmem_net *in)
{
    rt_uint32_t avail;
    rt_uint16_t avail_idx;
    struct ivshmem_net_queue *rx = &in->rx;
    struct virtq *vq = &rx->queue;
    struct virtq_desc *ret = RT_NULL;

    avail_idx = vq->avail->idx;
    rt_hw_dmb();

    if (avail_idx != rx->last_avail_idx)
    {
        avail = vq->avail->ring[rx->last_avail_idx++ & (vq->num - 1)];

        if (avail >= vq->num)
        {
            LOG_E("%s: invalid rx avail %d", in->ndev.parent.parent.name, avail);
        }
        else
        {
            ret = &vq->desc[avail];
        }
    }

    return ret;
}

static void ivshmem_net_rx_finish(struct ivshmem_net *in, struct virtq_desc *desc)
{
    struct ivshmem_net_queue *rx = &in->rx;
    struct virtq *vq = &rx->queue;
    rt_uint32_t desc_id = desc - vq->desc, used;

    used = rx->last_used_idx++ & (vq->num - 1);
    vq->used->ring[used].id = desc_id;
    vq->used->ring[used].len = 1;

    rt_hw_dmb();
    vq->used->idx = rx->last_used_idx;
    rt_hw_dmb();
}

static void ivshmem_net_set_state(struct ivshmem_net *in, rt_uint32_t state)
{
    if (in->self)
    {
        rt_atomic_store(&in->lstate, state);
        in->self->org_state = state;

        rt_hw_wmb();

        ivshmem_net_notify_vector(in, IVSHMEM_NET_STATE_IRQ);
    }
}

rt_inline rt_uint32_t ivshmem_net_get_state(struct ivshmem_net *in)
{
    rt_uint32_t ret = RT_UINT32_MAX;

    if (in->peer)
    {
        ret = in->peer->org_state;
    }

    return ret;
}

static void ivshmem_net_state_change(struct rt_work *work, void *work_data)
{
    rt_uint32_t rstate, lstate;
    struct ivshmem_net *in = rt_container_of(work, struct ivshmem_net, state_work);

    rstate = ivshmem_net_get_state(in);
    lstate = rt_atomic_load(&in->lstate);

    switch (lstate)
    {
    case IVSHMEM_NET_STATE_INIT:
        ivshmem_net_init_queues(in);

        if (rstate >= IVSHMEM_NET_STATE_INIT)
        {
            ivshmem_net_set_state(in, IVSHMEM_NET_STATE_READY);
        }
        break;
    case IVSHMEM_NET_STATE_READY:
        /*
         * link is up and we are running once the remote is in READY or RUN.
         */
        if (rstate >= IVSHMEM_NET_STATE_READY)
        {
            struct ip4_addr ipaddr, netmask, gw;

            ipaddr.addr = inet_addr("10.10.10.30");
            gw.addr = inet_addr("10.10.10.1");
            netmask.addr = inet_addr("255.255.255.0");

            netifapi_netif_set_addr(in->ndev.netif, &ipaddr, &netmask, &gw);
            eth_device_linkchange(&in->ndev, RT_TRUE);

            ivshmem_net_set_state(in, IVSHMEM_NET_STATE_RUN);
        }
        else if (rstate == IVSHMEM_NET_STATE_RESET)
        {
            ivshmem_net_set_state(in, IVSHMEM_NET_STATE_RESET);

            rt_hw_dmb();

            ivshmem_net_reset(in);

            eth_device_linkchange(&in->ndev, RT_FALSE);
        }
        else
        {
            ;
        }

        break;

    case IVSHMEM_NET_STATE_RUN:
        if (rstate == IVSHMEM_NET_STATE_RESET)
        {
            ivshmem_net_set_state(in, IVSHMEM_NET_STATE_RESET);

            rt_hw_dmb();

            ivshmem_net_reset(in);

            eth_device_linkchange(&in->ndev, RT_FALSE);
        }
        else
        {
            eth_device_ready(&in->ndev);
        }
        break;
    default:
        break;
    }

    rt_hw_wmb();
    rt_atomic_store(&in->rstate, rstate);
}

static rt_err_t ivshmem_net_tx(rt_device_t dev, struct pbuf *p)
{
    struct ivshmem_net *in = raw_to_ivshmem_net(dev);

    if (rt_atomic_load(&in->lstate) >= IVSHMEM_NET_STATE_READY)
    {
        void *buf;
        struct virtq *vq;
        struct virtq_desc *desc;
        struct ivshmem_net_queue *tx;
        rt_uint32_t desc_idx, avail, head;
        rt_base_t level;

        tx = &in->tx;
        vq = &tx->queue;

        ivshmem_net_tx_clean(in);

        if (!ivshmem_net_tx_ok(in, in->ndev.netif->mtu))
        {
            ivshmem_net_enable_tx_irq(in);
        }

        /* start of tx frame */
        RT_ASSERT(tx->num_free >= 1);

        level = ivshmem_spin_lock_irqsave(&in->tx_free_lock);
        desc_idx = tx->free_head;
        desc = &vq->desc[desc_idx];
        tx->free_head = desc->next;
        tx->num_free--;
        ivshmem_spin_unlock_irqrestore(&in->tx_free_lock, level);

        head = ivshmem_net_tx_advance(tx, &tx->head, p->tot_len);

        buf = (rt_uint8_t *)tx->data + head;
        pbuf_copy_partial(p, buf, p->tot_len, 0);

        desc->addr = buf - in->shm;
        desc->len = p->tot_len;
        desc->flags = 0;

        avail = tx->last_avail_idx++ & (vq->num - 1);
        vq->avail->ring[avail] = desc_idx;

        vq->avail->idx = tx->last_avail_idx;
        rt_hw_dmb();

        ivshmem_net_notify_tx(in, 1);
        /* end of tx frame */
    }

    return RT_EOK;
}

static struct pbuf *ivshmem_net_rx(rt_device_t dev)
{
    /* int received = 0; */
    struct pbuf *ret = RT_NULL;
    struct ivshmem_net *in = raw_to_ivshmem_net(dev);

    if (in->received == 0)
    {
        ivshmem_net_tx_clean(in);
    }

    do {
        void *data;
        rt_uint32_t len;
        struct virtq_desc *desc;

        desc = ivshmem_net_rx_desc(in);
        if (!desc)
        {
            break;
        }

        data = ivshmem_net_desc_data(in, &in->rx, desc, &len);

        if (!data)
        {
            LOG_E("%s: bad rx descriptor", in->ndev.parent.parent.name);
            break;
        }

        ret = pbuf_alloc(PBUF_RAW, len, PBUF_RAM);
        if (!ret)
        {
            LOG_E("%s: pbuf alloc failed", in->ndev.parent.parent.name);
            break;
        }

        rt_memcpy(ret->payload, data, len);

        ivshmem_net_rx_finish(in, desc);
        in->received++;
    }while(0);

    #if 0
    if (!ret)
    {
        ivshmem_net_enable_rx_irq(in);
    }
    #else
    ivshmem_net_enable_rx_irq(in);
    #endif

    if ((!ret) && (in->received != 0))
    {
        ivshmem_net_notify_rx(in, in->received);
        rt_hw_rmb();
        in->received = 0;
    }

    return ret;
}

static rt_err_t ivshmem_net_init(rt_device_t dev)
{
    rt_err_t ret = RT_EOK;
    struct ivshmem_net *in = raw_to_ivshmem_net(dev);

    ivshmem_net_reset(in);

    ivshmem_net_check_remote(in);


    return ret;
}

static rt_err_t ivshmem_net_control(rt_device_t dev, int cmd, void *args)
{
    rt_err_t status = RT_EOK;
    struct ivshmem_net *in = raw_to_ivshmem_net(dev);

    switch (cmd)
    {
    case NIOCTL_GADDR:
        if (args == RT_NULL)
        {
            status = -RT_ERROR;
            break;
        }

        rt_memcpy(args, in->mac, sizeof(in->mac));
        break;
    default:
        status = -RT_EINVAL;
        break;
    }

    return status;
}

#ifdef RT_USING_DEVICE_OPS
const static struct rt_device_ops ivshmem_net_ops =
{
    .init = ivshmem_net_init,
    .control = ivshmem_net_control
};
#endif

static rt_err_t ivshmem_net_isr(struct ivshmem_device *ivshmem_dev, int irq)
{
    rt_uint32_t lstate;
    struct ivshmem_net *in = RT_NULL;

    if (ivshmem_dev == RT_NULL)
    {
        return RT_EOK;
    }

    in = rt_container_of(ivshmem_dev, struct ivshmem_net, parent);

    if (in->self == RT_NULL)
    {
        return RT_EOK;
    }
    lstate = rt_atomic_load(&in->lstate);

    if (lstate >= IVSHMEM_NET_STATE_INIT)
    {
        rt_workqueue_dowork(in->state_wq, &in->state_work);
    }
    else if (lstate == IVSHMEM_NET_STATE_RESET)
    {
        ivshmem_net_broadcast(in);
    }
    else if (lstate == IVSHMEM_NET_STATE_BCST)
    {
        /* ack the peer */
        in->self->ack_ivpos = in->ivpos;
        in->broken = RT_TRUE;
    }
    else if (lstate == IVSHMEM_NET_STATE_LINK)
    {
        /* link OK! */
        if (in->peer && in->peer->ack_ivpos != RT_UINT16_MAX &&
            in->peer->ack_ivpos == in->peer_id)
        {
            /* ack the peer when we are the sente */
            in->self->ack_ivpos = in->ivpos;

            /* to init state */
            rt_atomic_store(&in->rstate, IVSHMEM_NET_STATE_INIT);
            rt_atomic_store(&in->lstate, IVSHMEM_NET_STATE_INIT);
            ivshmem_net_init_stub(in);
            rt_hw_wmb();

            /* ask peer goto init */
            ivshmem_net_notify_vector(in, IVSHMEM_NET_STATE_IRQ);

            /*
             * if peer is sente, the magic area will set to the magic not zero,
             * when we are the sente, we must be wait for peer change state to
             * init, call workqueue maybe clear the ack stub info.
             */
            if (in->peer->magic == IVSHMEM_NET_STUB_MAGIC)
            {
                rt_workqueue_dowork(in->state_wq, &in->state_work);
            }
        }
    }

    return RT_EOK;
}

static rt_err_t ivshmem_net_probe(struct rt_pci_device *dev)
{
    static int dev_no = 0;
    rt_err_t ret = RT_EOK;
    char dev_name[RT_NAME_MAX];
    struct ivshmem_net *in = rt_calloc(1, sizeof(*in));

    rt_kprintf("ivshmem_net_probe: start\n");

    do {
        rt_uint32_t rand;
        rt_device_t parent;
        struct eth_device *ndev;

        if (!in)
        {
            ret = -RT_ENOMEM;
            break;
        }

        ret = ivshmem_pci_probe(&in->parent, dev);

        if (ret)
        {
            rt_kprintf("ivshmem_net_probe: ivshmem_pci_probe failed %d\n", ret);
            break;
        }

        if (in->parent.shmem_size <= (sizeof(*in->self) + sizeof(*in->peer)))
        {
            rt_kprintf("ivshmem_net_probe: shmem too small %d\n", (int)in->parent.shmem_size);
            return -ENOMEM;
        }

        ndev = &in->ndev;
        parent = &ndev->parent;

        #if 0
        in->parent.dev->dev = parent;
        #endif

        rt_snprintf(dev_name, RT_NAME_MAX, "vnet%d", dev_no++);

        in->parent.handle_irq = ivshmem_net_isr;
        in->regs = in->parent.reg;
        in->shm = in->parent.shmem;
        in->shm_size = in->parent.shmem_size - (sizeof(*in->self) + sizeof(*in->peer));

        rt_spin_lock_init(&in->tx_free_lock);
        rt_spin_lock_init(&in->tx_clean_lock);

        ret = ivshmem_net_calc_qsize(in);

        if (ret)
        {
            rt_kprintf("ivshmem_net_probe: calc qsize failed %d\n", ret);
            break;
        }

        in->ivpos = HWREG32(&in->regs->ivpos);

        in->state_wq = rt_workqueue_create(dev_name, RT_SYSTEM_WORKQUEUE_STACKSIZE, RT_THREAD_PRIORITY_MAX / 2);

        if (!in->state_wq)
        {
            rt_kprintf("ivshmem_net_probe: create workqueue failed\n");
            break;
        }

        rt_work_init(&in->state_work, ivshmem_net_state_change, RT_NULL);

        if (ivshmem_install_msix_vectors(&in->parent, 1, dev_name))
        {
            ivshmem_install_intx_vector(&in->parent, dev_name);
        }

        rand = (qrand32(0) << 16) | qrand32(0);

        in->mac[5] = ((rt_uint8_t *)&rand)[3];
        in->mac[4] = ((rt_uint8_t *)&rand)[2];
        /* we use ivpos as mac[2]-[3] */
        in->mac[3] = (in->ivpos >> 8) & 0xf;
        in->mac[2] = in->ivpos & 0xf;
        in->mac[1] = ((rt_uint8_t *)&rand)[1];
        in->mac[0] = ((rt_uint8_t *)&rand)[0];
        /* clear multicast bit */
        in->mac[0] &= 0xfe;
        /* set local assignment bit (IEEE802) */
        in->mac[0] |= 0x02;

        in->received = 0;

        parent->type = RT_Device_Class_NetIf;
    #ifdef RT_USING_DEVICE_OPS
        parent->ops = &ivshmem_net_ops;
    #else
        parent->init    = ivshmem_net_init;
        parent->control = ivshmem_net_init;
    #endif
        ndev->eth_tx    = ivshmem_net_tx;
        ndev->eth_rx    = ivshmem_net_rx;

        ret = eth_device_init(ndev, dev_name);
        rt_kprintf("ivshmem_net_probe: eth_device_init %s ret=%d ivpos=%d shm=%p size=%d\n",
                   dev_name, ret, (int)in->ivpos, in->shm, (int)in->shm_size);
    } while (0);

    if (ret && in)
    {
        if (in->state_wq)
        {
            rt_workqueue_destroy(in->state_wq);
        }

        rt_free(in);
    }

    return ret;
}

#if 0
static void ivshmem_net_remove(struct rt_pci_device *dev)
{
    struct ivshmem_device *ivshmem_dev = ivshmem_pci_remove(dev);

    if (ivshmem_dev)
    {
        struct ivshmem_net *in = rt_container_of(ivshmem_dev, struct ivshmem_net, parent);

        ivshmem_net_set_state(in, IVSHMEM_NET_STATE_RESET);

        eth_device_linkchange(&in->ndev, RT_FALSE);
        eth_device_deinit(&in->ndev);

        rt_workqueue_cancel_work_sync(in->state_wq, &in->state_work);
        rt_workqueue_destroy(in->state_wq);

        in->parent.handle_irq = RT_NULL;

        rt_hw_wmb();

        ivshmem_net_set_state(in, IVSHMEM_NET_STATE_RESET);

        rt_hw_wmb();

        rt_free(in);
    }
}
#endif

static struct rt_pci_device_id ivshmem_net_pci_ids[] =
{
    {
        RT_PCI_DEVICE_ID(PCI_VENDOR_ID_REDHAT_QUMRANET, 0x1110),
        .class = PCI_BASE_CLASS_NETWORK << 16,
        .class_mask = 0xff << 16,
    },
    { /* sentinel */ }
};

static struct rt_pci_driver ivshmem_net_drv =
{
    .name = "ivshmem-net",
    .ids = ivshmem_net_pci_ids,
    .probe = ivshmem_net_probe,
    /* .remove = ivshmem_net_remove, */
};

RT_PCI_DRIVER_EXPORT(ivshmem_net_drv);
