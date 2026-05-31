#include <rtthread.h>
#include <rthw.h>
#include <rtdevice.h>
#include <ioremap.h>
#include <stdlib.h>
#include <dt-bindings-pinctrl.h>

#include "servo_bus.h"

#define SERVO_UART_NAME "uart7"
#define SERVO_BAUD_RATE 115200
#define SERVO_TX_EN_PIN RK_GPIO0_A0       /* ROCK 5B pin37, GPIO0_A0 */
#define SERVO_MAX_PAYLOAD 512
#define SERVO_DEFAULT_DURATION_MS 400

#define SERVO_GPIO0_BASE 0xfd8a0000UL
#define SERVO_GPIO_MAP_SIZE 0x1000UL
#define SERVO_GPIO_V2_DR 0x00
#define SERVO_GPIO_V2_DDR 0x08
#define SERVO_GPIO_V2_EXT_PORT 0x70

#define SERVO_IOC_BASE 0xfd5f0000UL
#define SERVO_IOC_MAP_SIZE 0x10000UL
#define SERVO_GPIO0_A0_IOMUX 0x0010UL
#define SERVO_GPIO0_A0_IOMUX_MASK 0xfU
#define SERVO_GPIO0_A0_FUNC_GPIO 0U

#ifdef RT_USING_OFW
#include <drivers/ofw.h>
#endif

struct servo_bus_state
{
    rt_device_t uart;
    rt_mutex_t lock;
    rt_bool_t initialized;
    rt_bool_t oe_high;
    rt_bool_t oe_rt_pin_ok;
    rt_bool_t oe_mmio_ok;
    rt_bool_t oe_iomux_ok;
    rt_uint32_t tx_count;
    rt_uint32_t error_count;
    rt_size_t last_len;
    char last_cmd[SERVO_MAX_PAYLOAD];
};

static struct servo_bus_state g_servo_bus;
static void *g_servo_gpio0_base;
static void *g_servo_ioc_base;

static void servo_gpio0a0_mux_gpio(void)
{
    rt_uint32_t data;

    if (!g_servo_ioc_base)
    {
        g_servo_ioc_base = rt_ioremap((void *)SERVO_IOC_BASE, SERVO_IOC_MAP_SIZE);
    }

    if (!g_servo_ioc_base)
    {
        g_servo_bus.oe_iomux_ok = RT_FALSE;
        return;
    }

    /*
     * RK3588 GPIO0_A0 lives in PMU1 IOC. Function 0 is GPIO.
     * Use the Rockchip write-mask style: upper bits select which low bits update.
     */
    data = (SERVO_GPIO0_A0_IOMUX_MASK << 16) |
           (SERVO_GPIO0_A0_FUNC_GPIO << 0);
    HWREG32((rt_uint8_t *)g_servo_ioc_base + SERVO_GPIO0_A0_IOMUX) = data;
    g_servo_bus.oe_iomux_ok = RT_TRUE;
}

static rt_uint32_t servo_gpio0a0_iomux_read(void)
{
    if (!g_servo_ioc_base)
    {
        return 0xffffffffU;
    }

    return HWREG32((rt_uint8_t *)g_servo_ioc_base + SERVO_GPIO0_A0_IOMUX);
}

static void servo_gpio0_write_bit(rt_uint32_t offset, int bit, rt_bool_t value)
{
    void *reg;
    rt_uint32_t data;

    if (!g_servo_gpio0_base)
    {
        return;
    }

    reg = (rt_uint8_t *)g_servo_gpio0_base + offset + (bit >= 16 ? 0x4 : 0x0);
    data = RT_BIT((rt_uint32_t)bit % 16U + 16U);
    if (value)
    {
        data |= RT_BIT((rt_uint32_t)bit % 16U);
    }

    HWREG32(reg) = data;
}

static int servo_gpio0_read_bit(rt_uint32_t offset, int bit)
{
    rt_uint32_t data;

    if (!g_servo_gpio0_base)
    {
        return -1;
    }

    data = HWREG32((rt_uint8_t *)g_servo_gpio0_base + offset);
    return (data >> bit) & 1U;
}

static void servo_gpio0_init_oe(void)
{
    if (!g_servo_bus.oe_rt_pin_ok)
    {
        /*
         * Preferred path: let the Rockchip pin driver own GPIO0_A0.
         * This requires gpio0 to be present in the RT DTB.
         */
        rt_pin_mode(SERVO_TX_EN_PIN, PIN_MODE_OUTPUT);
        rt_pin_write(SERVO_TX_EN_PIN, PIN_LOW);
        if (rt_pin_read(SERVO_TX_EN_PIN) == PIN_LOW)
        {
            g_servo_bus.oe_rt_pin_ok = RT_TRUE;
        }
    }

    if (g_servo_bus.oe_rt_pin_ok)
    {
        return;
    }

    servo_gpio0a0_mux_gpio();

    if (!g_servo_gpio0_base)
    {
        g_servo_gpio0_base = rt_ioremap((void *)SERVO_GPIO0_BASE, SERVO_GPIO_MAP_SIZE);
    }

    if (!g_servo_gpio0_base)
    {
        g_servo_bus.oe_mmio_ok = RT_FALSE;
        return;
    }

    g_servo_bus.oe_mmio_ok = RT_TRUE;
    servo_gpio0a0_mux_gpio();
    servo_gpio0_write_bit(SERVO_GPIO_V2_DDR, SERVO_TX_EN_PIN, RT_TRUE);
}

static void servo_bus_set_oe(rt_bool_t enable)
{
    servo_gpio0_init_oe();
    if (g_servo_bus.oe_rt_pin_ok)
    {
        rt_pin_write(SERVO_TX_EN_PIN, enable ? PIN_HIGH : PIN_LOW);
    }
    else if (g_servo_bus.oe_mmio_ok)
    {
        servo_gpio0_write_bit(SERVO_GPIO_V2_DR, SERVO_TX_EN_PIN, enable);
    }
    g_servo_bus.oe_high = enable;
}

static void servo_bus_apply_uart7_pinctrl(void)
{
#ifdef RT_USING_OFW
    rt_device_t dev;
    rt_err_t ret;
    struct rt_ofw_node *np;

    dev = rt_device_find(SERVO_UART_NAME);
    if (dev && dev->ofw_node)
    {
        ret = rt_pin_ctrl_confs_apply_by_name(dev, RT_NULL);
        rt_kprintf("servo_bus: uart7 pinctrl via device ret=%d\n", ret);
        return;
    }

    np = rt_ofw_find_node_by_path("/serial@feba0000");
    if (np)
    {
        struct rt_device fake;

        rt_memset(&fake, 0, sizeof(fake));
        fake.ofw_node = np;
        ret = rt_pin_ctrl_confs_apply_by_name(&fake, RT_NULL);
        rt_ofw_node_put(np);
        rt_kprintf("servo_bus: uart7 pinctrl via ofw ret=%d\n", ret);
        return;
    }
#endif

    rt_kprintf("servo_bus: uart7 pinctrl not applied\n");
}

static rt_err_t servo_bus_configure_pins(void)
{
    /*
     * Radxa ROCK 5B 40pin:
     *   pin 15: GPIO3_C0 / UART7_TX_M1
     *   pin 11: GPIO3_C1 / UART7_RX_M1
     *   pin 37: GPIO0_A0 / output enable for bus transceiver
     *
     * Do not write GRF/IOMUX registers through bare physical addresses here.
     * RT-Thread does not map 0xfd5fxxxx into this task's virtual address space,
     * and a direct store aborts the RT shell. UART7 pinmux must come from DTS
     * or a mapped pinctrl path; this module only owns the OE GPIO.
     */
    servo_bus_apply_uart7_pinctrl();
    servo_gpio0_init_oe();
    servo_bus_set_oe(RT_FALSE);

    return RT_EOK;
}

static rt_bool_t servo_bus_payload_allowed_len(const char *payload, rt_size_t len)
{
    if (!payload)
    {
        return RT_FALSE;
    }

    if (len == 0 || len >= SERVO_MAX_PAYLOAD)
    {
        return RT_FALSE;
    }

    if (payload[0] != '#' && payload[0] != '$' && payload[0] != '{')
    {
        return RT_FALSE;
    }

    if (payload[len - 1] != '!' && payload[len - 1] != '}')
    {
        return RT_FALSE;
    }

    for (rt_size_t i = 0; i < len; ++i)
    {
        unsigned char ch = (unsigned char)payload[i];

        if (ch < 0x20 || ch > 0x7e)
        {
            return RT_FALSE;
        }
    }

    return RT_TRUE;
}

static rt_bool_t servo_bus_payload_allowed(const char *payload, rt_size_t *len_out)
{
    rt_size_t len;

    if (!payload)
    {
        return RT_FALSE;
    }

    len = rt_strlen(payload);
    if (!servo_bus_payload_allowed_len(payload, len))
    {
        return RT_FALSE;
    }

    if (len_out)
    {
        *len_out = len;
    }

    return RT_TRUE;
}

rt_err_t servo_bus_init(void)
{
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;
    rt_err_t ret;

    if (g_servo_bus.initialized)
    {
        return RT_EOK;
    }

    ret = servo_bus_configure_pins();
    if (ret != RT_EOK)
    {
        return ret;
    }

    g_servo_bus.uart = rt_device_find(SERVO_UART_NAME);
    if (!g_servo_bus.uart)
    {
        g_servo_bus.error_count++;
        rt_kprintf("servo_bus: %s not found\n", SERVO_UART_NAME);
        return -RT_ERROR;
    }

    config.baud_rate = SERVO_BAUD_RATE;
    config.data_bits = DATA_BITS_8;
    config.stop_bits = STOP_BITS_1;
    config.parity = PARITY_NONE;
    config.bufsz = RT_SERIAL_RB_BUFSZ;
    rt_device_control(g_servo_bus.uart, RT_DEVICE_CTRL_CONFIG, &config);

    if (rt_device_open(g_servo_bus.uart, RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX) != RT_EOK)
    {
        g_servo_bus.error_count++;
        rt_kprintf("servo_bus: open %s failed\n", SERVO_UART_NAME);
        return -RT_ERROR;
    }

    g_servo_bus.lock = rt_mutex_create("servo_bus", RT_IPC_FLAG_PRIO);
    if (!g_servo_bus.lock)
    {
        rt_device_close(g_servo_bus.uart);
        g_servo_bus.uart = RT_NULL;
        g_servo_bus.error_count++;
        rt_kprintf("servo_bus: mutex create failed\n");
        return -RT_ENOMEM;
    }

    g_servo_bus.initialized = RT_TRUE;
    rt_kprintf("servo_bus: ready on %s, baud=%d, tx_en=GPIO0_A0\n",
               SERVO_UART_NAME, SERVO_BAUD_RATE);

    return RT_EOK;
}

rt_err_t servo_bus_send_raw(const char *payload)
{
    rt_size_t len = 0;

    if (!servo_bus_payload_allowed(payload, &len))
    {
        g_servo_bus.error_count++;
        rt_kprintf("servo_bus: invalid payload\n");
        return -RT_EINVAL;
    }

    return servo_bus_send_raw_len(payload, len);
}

rt_err_t servo_bus_send_raw_len(const char *payload, rt_size_t len)
{
    rt_size_t written;
    rt_uint32_t wait_us;
    rt_err_t ret = RT_EOK;

    if (!g_servo_bus.initialized)
    {
        ret = servo_bus_init();
        if (ret != RT_EOK)
        {
            return ret;
        }
    }

    if (!servo_bus_payload_allowed_len(payload, len))
    {
        g_servo_bus.error_count++;
        rt_kprintf("servo_bus: invalid payload\n");
        return -RT_EINVAL;
    }

    rt_mutex_take(g_servo_bus.lock, RT_WAITING_FOREVER);

    servo_bus_set_oe(RT_TRUE);
    rt_hw_us_delay(50);

    written = rt_device_write(g_servo_bus.uart, 0, payload, len);
    wait_us = (rt_uint32_t)((len * 10UL * 1000000UL) / SERVO_BAUD_RATE) + 1000U;
    rt_hw_us_delay(wait_us);

    servo_bus_set_oe(RT_FALSE);

    if (written != len)
    {
        g_servo_bus.error_count++;
        ret = -RT_ERROR;
    }
    else
    {
        rt_size_t copy_len = len;

        if (copy_len >= sizeof(g_servo_bus.last_cmd))
        {
            copy_len = sizeof(g_servo_bus.last_cmd) - 1;
        }

        g_servo_bus.tx_count++;
        g_servo_bus.last_len = len;
        rt_memcpy(g_servo_bus.last_cmd, payload, copy_len);
        g_servo_bus.last_cmd[copy_len] = '\0';
    }

    rt_mutex_release(g_servo_bus.lock);

    return ret;
}

rt_err_t servo_bus_send_set(int id, int pwm, int duration_ms)
{
    char cmd[32];

    if (id < 0 || id > 999 || pwm < 500 || pwm > 2500)
    {
        return -RT_EINVAL;
    }

    if (duration_ms <= 0)
    {
        duration_ms = SERVO_DEFAULT_DURATION_MS;
    }
    if (duration_ms > 9999)
    {
        duration_ms = 9999;
    }

    rt_snprintf(cmd, sizeof(cmd), "#%03dP%04dT%04d!", id, pwm, duration_ms);
    return servo_bus_send_raw(cmd);
}

rt_err_t servo_bus_home(void)
{
    return servo_bus_send_raw("#000P1500T0600!#001P1500T0600!#002P1500T0600!#003P1500T0600!#004P1500T0600!#005P1500T0600!");
}

void servo_bus_diag(void)
{
    rt_ssize_t rt_pin_level = -1;
    int dr_level = servo_gpio0_read_bit(SERVO_GPIO_V2_DR, SERVO_TX_EN_PIN);
    int ddr_level = servo_gpio0_read_bit(SERVO_GPIO_V2_DDR, SERVO_TX_EN_PIN);
    int ext_level = servo_gpio0_read_bit(SERVO_GPIO_V2_EXT_PORT, SERVO_TX_EN_PIN);

    if (g_servo_bus.oe_rt_pin_ok)
    {
        rt_pin_level = rt_pin_read(SERVO_TX_EN_PIN);
    }

    rt_kprintf("servo_bus diag:\n");
    rt_kprintf("  initialized=%d uart=%s opened=%d tx_en=%d oe_rt_pin=%d oe_mmio=%d oe_iomux=%d\n",
               g_servo_bus.initialized,
               g_servo_bus.uart ? SERVO_UART_NAME : "NULL",
               g_servo_bus.uart ? 1 : 0,
               g_servo_bus.oe_high,
               g_servo_bus.oe_rt_pin_ok,
               g_servo_bus.oe_mmio_ok,
               g_servo_bus.oe_iomux_ok);
    rt_kprintf("  gpio0a0_iomux=0x%08x\n", servo_gpio0a0_iomux_read());
    rt_kprintf("  gpio0a0 rt_pin=%d dr=%d ddr=%d ext=%d\n", (int)rt_pin_level, dr_level, ddr_level, ext_level);
    rt_kprintf("  tx_count=%u error_count=%u last_len=%u\n",
               g_servo_bus.tx_count,
               g_servo_bus.error_count,
               (unsigned int)g_servo_bus.last_len);
    rt_kprintf("  last_cmd=%s\n", g_servo_bus.last_cmd);
}

static int servo_oe_cmd(int argc, char **argv)
{
    if (argc != 2)
    {
        rt_kprintf("Usage: servo_oe <high|low>\n");
        return -RT_EINVAL;
    }

    if (!rt_strcmp(argv[1], "high") || !rt_strcmp(argv[1], "1"))
    {
        servo_bus_set_oe(RT_TRUE);
    }
    else if (!rt_strcmp(argv[1], "low") || !rt_strcmp(argv[1], "0"))
    {
        servo_bus_set_oe(RT_FALSE);
    }
    else
    {
        rt_kprintf("Usage: servo_oe <high|low>\n");
        return -RT_EINVAL;
    }

    rt_kprintf("servo_bus: tx_en=%d\n", g_servo_bus.oe_high);
    return RT_EOK;
}
MSH_CMD_EXPORT_ALIAS(servo_oe_cmd, servo_oe, manually control bus servo TX enable);

static int servo_high_cmd(int argc, char **argv)
{
    RT_UNUSED(argc);
    RT_UNUSED(argv);

    servo_bus_set_oe(RT_TRUE);
    rt_kprintf("servo_bus: tx_en=1 rt_pin=%d gpio0a0_ext=%d\n",
               g_servo_bus.oe_rt_pin_ok ? (int)rt_pin_read(SERVO_TX_EN_PIN) : -1,
               servo_gpio0_read_bit(SERVO_GPIO_V2_EXT_PORT, SERVO_TX_EN_PIN));
    return RT_EOK;
}
MSH_CMD_EXPORT_ALIAS(servo_high_cmd, servo_high, set bus servo TX enable high);

static int servo_low_cmd(int argc, char **argv)
{
    RT_UNUSED(argc);
    RT_UNUSED(argv);

    servo_bus_set_oe(RT_FALSE);
    rt_kprintf("servo_bus: tx_en=0 rt_pin=%d gpio0a0_ext=%d\n",
               g_servo_bus.oe_rt_pin_ok ? (int)rt_pin_read(SERVO_TX_EN_PIN) : -1,
               servo_gpio0_read_bit(SERVO_GPIO_V2_EXT_PORT, SERVO_TX_EN_PIN));
    return RT_EOK;
}
MSH_CMD_EXPORT_ALIAS(servo_low_cmd, servo_low, set bus servo TX enable low);

static int servo_burst_cmd(int argc, char **argv)
{
    int count = 200;
    const char pattern[] = "U";

    if (argc == 2)
    {
        count = atoi(argv[1]);
    }
    if (count <= 0)
    {
        count = 200;
    }
    if (count > 5000)
    {
        count = 5000;
    }

    if (!g_servo_bus.initialized && servo_bus_init() != RT_EOK)
    {
        return -RT_ERROR;
    }

    servo_bus_set_oe(RT_TRUE);
    rt_hw_us_delay(50);
    for (int i = 0; i < count; ++i)
    {
        rt_device_write(g_servo_bus.uart, 0, pattern, 1);
        rt_hw_us_delay(100);
    }
    servo_bus_set_oe(RT_FALSE);

    g_servo_bus.tx_count++;
    rt_snprintf(g_servo_bus.last_cmd, sizeof(g_servo_bus.last_cmd), "burst:%d", count);
    g_servo_bus.last_len = count;
    rt_kprintf("servo_bus: burst sent %d bytes\n", count);
    return RT_EOK;
}
MSH_CMD_EXPORT_ALIAS(servo_burst_cmd, servo_burst, send repeated UART bytes for line probing);

static int servo_bus_init_cmd(int argc, char **argv)
{
    RT_UNUSED(argc);
    RT_UNUSED(argv);

    return servo_bus_init();
}
MSH_CMD_EXPORT_ALIAS(servo_bus_init_cmd, servo_bus_init, init RK3588 UART7 bus servo);

static int servo_raw_cmd(int argc, char **argv)
{
    if (argc != 2)
    {
        rt_kprintf("Usage: servo_raw <payload>\n");
        rt_kprintf("Example: servo_raw #001P1500T0600!\n");
        return -RT_EINVAL;
    }

    return servo_bus_send_raw(argv[1]);
}
MSH_CMD_EXPORT_ALIAS(servo_raw_cmd, servo_raw, send raw bus servo command);

static int servo_set_cmd(int argc, char **argv)
{
    int id;
    int pwm;
    int duration_ms = SERVO_DEFAULT_DURATION_MS;

    if (argc < 3 || argc > 4)
    {
        rt_kprintf("Usage: servo_set <id> <pwm> [duration_ms]\n");
        rt_kprintf("Example: servo_set 1 1500 600\n");
        return -RT_EINVAL;
    }

    id = atoi(argv[1]);
    pwm = atoi(argv[2]);
    if (argc == 4)
    {
        duration_ms = atoi(argv[3]);
    }

    return servo_bus_send_set(id, pwm, duration_ms);
}
MSH_CMD_EXPORT_ALIAS(servo_set_cmd, servo_set, send one bus servo position);

static int servo_home_cmd(int argc, char **argv)
{
    RT_UNUSED(argc);
    RT_UNUSED(argv);

    return servo_bus_home();
}
MSH_CMD_EXPORT_ALIAS(servo_home_cmd, servo_home, send six-axis home command);

static int servo_diag_cmd(int argc, char **argv)
{
    RT_UNUSED(argc);
    RT_UNUSED(argv);

    servo_bus_diag();
    return RT_EOK;
}
MSH_CMD_EXPORT_ALIAS(servo_diag_cmd, servo_diag, show bus servo diagnostics);
