// #include <rtthread.h>
// #include <rtdevice.h>

// static rt_device_t uart4_dev = RT_NULL, uart7_dev = RT_NULL;
// static rt_sem_t uart4_sem = RT_NULL, uart7_sem = RT_NULL;
// struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;

// static rt_err_t uart4_rx_ind(rt_device_t dev, rt_size_t size)
// {
//     if (size > 0)
//     {
//         rt_sem_release(uart4_sem);
//     }
//     return RT_EOK;
// }

// static rt_err_t uart7_rx_ind(rt_device_t dev, rt_size_t size)
// {
//     if (size > 0)
//     {
//         rt_sem_release(uart7_sem);
//     }
//     return RT_EOK;
// }

// static void uart4_demo_test(void *parameter)
// {
//     char ch;
//     char str[] = "hello RT-Thread!\r\n";
//     rt_device_control(uart4_dev, RT_DEVICE_CTRL_CONFIG, &config);

//     rt_device_open(uart4_dev, RT_DEVICE_FLAG_INT_RX);
//     rt_device_set_rx_indicate(uart4_dev, uart4_rx_ind);
//     rt_device_write(uart4_dev, 0, str, (sizeof(str) - 1));

//     while (1)
//     {
//         while (rt_device_read(uart4_dev, -1, &ch, 1) != 1)
//         {
//             rt_sem_take(uart4_sem, RT_WAITING_FOREVER);
//         }
//         rt_device_write(uart4_dev, 0, &ch, 1);
//     }
// }

// static void uart7_demo_test(void *parameter)
// {
//     char ch;
//     char str[] = "hello RT-Thread!\r\n";
//     rt_device_control(uart7_dev, RT_DEVICE_CTRL_CONFIG, &config);
//     rt_device_open(uart7_dev, RT_DEVICE_FLAG_INT_RX);
//     rt_device_set_rx_indicate(uart7_dev, uart7_rx_ind);

//     rt_device_write(uart7_dev, 0, str, (sizeof(str) - 1));

//     while (1)
//     {
//         while (rt_device_read(uart7_dev, -1, &ch, 1) != 1)
//         {
//             rt_sem_take(uart7_sem, RT_WAITING_FOREVER);
//         }
//         rt_device_write(uart7_dev, 0, &ch, 1);
//     }
// }

// int uart_test()
// {
//     rt_thread_t uart4_thread = RT_NULL, uart7_thread = RT_NULL;

//     uart4_dev = rt_device_find("uart4");
//     uart7_dev = rt_device_find("uart7");

//     if(!uart4_dev || !uart7_dev)
//     {
//         rt_kprintf("uart4 or uart7 don't find !!! \n");
//         return -RT_ERROR;
//     }

//     uart4_sem = rt_sem_create("uart4_sem",0,RT_IPC_FLAG_PRIO);
//     uart7_sem = rt_sem_create("uart7_sem",0,RT_IPC_FLAG_PRIO);

//     if(!uart4_sem || !uart7_sem)
//     {
//         rt_kprintf("uart4_sem or uart7_sem don't create !!! \n");
//         return -RT_ERROR;
//     }

//     uart4_thread = rt_thread_create("uart4_demo_test",
//                                 uart4_demo_test,
//                                 RT_NULL,
//                                 4096,20,10);

//     uart7_thread = rt_thread_create("uart7_demo_test",
//                         uart7_demo_test,
//                         RT_NULL,
//                         4096,20,10);

//     if(!uart4_thread || !uart7_thread )
//     {
//         rt_kprintf("uart4_thread or uart7_thread don't create !!! \n");
//         return -RT_ERROR;
//     }
//     rt_thread_startup(uart4_thread);
//     rt_thread_startup(uart7_thread);

//     return RT_EOK;
// }
// MSH_CMD_EXPORT(uart_test, uart_test);
