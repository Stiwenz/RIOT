/*
 * Copyright (C) 2022 HAW Hamburg
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     tests
 * @{
 *
 * @file
 * @brief       Test application for AT86RF215 IEEE 802.15.4 device driver
 *
 * @author      Leandro Lanzieri <leandro.lanzieri@haw-hamburg.de>
 *
 * @}
 */

#include <stdio.h>

#include "at86rf215.h"
#include "at86rf215_internal.h"
#include "at86rf215_params.h"
#include "init_dev.h"
#include "shell.h"
#include "test_utils/netdev_ieee802154_minimal.h"

#include "iolist.h"
#include "kernel_defines.h"
#include "net/netdev/ieee802154.h"
#include "net/ieee802154.h"
#include "net/l2util.h"
#include "od.h"

#include "thread.h"
#include "event.h"
#include "event/thread.h"
#include "sys/bus.h"
#include "od.h"

#define DEV_TX 1
#define DEV_RX 0

#define DEVICE_ROLE DEV_TX

static at86rf215_t at86rf215[NETDEV_IEEE802154_MINIMAL_NUMOF];

static char batmon_stack[THREAD_STACKSIZE_MAIN];
static char rf_stack[THREAD_STACKSIZE_MAIN];

void *batmon_thread(void *arg)
{
    (void) arg;

    msg_t msg;
    msg_bus_entry_t sub;
    msg_bus_t *bus = sys_bus_get(SYS_BUS_POWER);

    msg_bus_attach(bus, &sub);
    msg_bus_subscribe(&sub, SYS_BUS_POWER_EVENT_LOW_VOLTAGE);

    while (1) {
        msg_receive(&msg);
        puts("NA NA NA NA NA NA NA NA NA NA NA NA NA BATMON");
    }
}

static int cmd_enable_batmon(int argc, char **argv)
{
    int res;
    uint16_t voltage;
    netdev_t *netdev = &(at86rf215[0].netdev.netdev);

    if (argc < 2) {
        printf("usage: %s <treshold_mV>\n", argv[0]);
        return -1;
    }

    voltage = atoi(argv[1]);
    res = netdev->driver->set(netdev, NETOPT_BATMON, &voltage, sizeof(voltage));

    if (res != sizeof(voltage)) {
        puts("value out of range");
    }

    return res;
}

static int cmd_set_trim(int argc, char **argv)
{
    if (argc < 2) {
        printf("usage: %s <trim>\n", argv[0]);
        return 1;
    }

    uint8_t trim = atoi(argv[1]);
    at86rf215_t *dev = at86rf215;

    if (trim > 0xF) {
        puts("Trim value out of range");
        return 1;
    }

    printf("setting trim to %u fF\n", 300U * trim);
    at86rf215_set_trim(dev, trim);

    return 0;
}

static int cmd_set_clock_out(int argc, char **argv)
{
    const char *keys[] = {
        [AT86RF215_CLKO_OFF]    = "off",
        [AT86RF215_CLKO_26_MHz] = "26",
        [AT86RF215_CLKO_32_MHz] = "32",
        [AT86RF215_CLKO_16_MHz] = "16",
        [AT86RF215_CLKO_8_MHz]  = "8",
        [AT86RF215_CLKO_4_MHz]  = "4",
        [AT86RF215_CLKO_2_MHz]  = "2",
        [AT86RF215_CLKO_1_MHz]  = "1",
    };

    at86rf215_clko_freq_t freq = AT86RF215_CLKO_26_MHz;
    at86rf215_t *dev = at86rf215;

    if (argc > 1) {
        unsigned tmp = 0xFF;
        for (unsigned i = 0; i < ARRAY_SIZE(keys); ++i) {
            if (strcmp(argv[1], keys[i]) == 0) {
                tmp = i;
                break;
            }
        }

        if (tmp == 0xFF) {
            printf("usage: %s [freq in MHz | off]\n", argv[0]);
            printf("valid frequencies: off");
            for (unsigned i = 1; i < ARRAY_SIZE(keys); ++i) {
                printf(", %s MHz", keys[i]);
            }
            puts("");
            return 1;
        }

        freq = tmp;
    }

    printf("Clock output set to %s %s\n", keys[freq], freq ? "MHz" : "");
    at86rf215_set_clock_output(dev, AT86RF215_CLKO_4mA, freq);

    return 0;
}

static int cmd_get_random(int argc, char **argv)
{
    uint8_t values;
    uint8_t buffer[256];
    at86rf215_t *dev = at86rf215;

    if (argc > 1) {
        values = atoi(argv[1]);
    }
    else {
        values = 16;
    }

    if (values == 0) {
        printf("usage: %s [num]\n", argv[0]);
        return 1;
    }

    at86rf215_get_random(dev, buffer, values);

    od_hex_dump(buffer, values, 0);

    return 0;
}

int test_init(void)
{
    /* create battery monitor thread */
    thread_create(batmon_stack, sizeof(batmon_stack), THREAD_PRIORITY_MAIN - 1,
                  THREAD_CREATE_STACKTEST, batmon_thread, NULL, "batmon");
    return 0;
}

static int _init_driver(netdev_t *netdev, netdev_event_cb_t cb)
{
    /* set the application-provided callback */
    netdev->event_callback = cb;

    /* initialize the device driver */
    return netdev->driver->init(netdev);
}

static char _addr_str[IEEE802154_LONG_ADDRESS_LEN * 3];

// static int send(int iface, le_uint16_t dst_pan, uint8_t *dst, size_t dst_len,
//                 char *data)
// {
//     int res;
//     netdev_ieee802154_t *dev;
//     uint8_t *src;
//     size_t src_len;
//     uint8_t mhr[IEEE802154_MAX_HDR_LEN];
//     uint8_t flags;
//     le_uint16_t src_pan;

//     if (((unsigned)iface) > (NETDEV_IEEE802154_MINIMAL_NUMOF - 1)) {
//         printf("txtsnd: %d is not an interface\n", iface);
//         return 1;
//     }

//     iolist_t iol_data = {
//         .iol_base = data,
//         .iol_len = strlen(data)
//     };

//     dev = container_of(_devices[iface].dev, netdev_ieee802154_t, netdev);
//     flags = (uint8_t)(dev->flags & NETDEV_IEEE802154_SEND_MASK);
//     flags |= IEEE802154_FCF_TYPE_DATA;
//     src_pan = byteorder_btols(byteorder_htons(dev->pan));
//     if (dst_pan.u16 == 0) {
//         dst_pan = src_pan;
//     }
//     if (dev->flags & NETDEV_IEEE802154_SRC_MODE_LONG) {
//         src_len = 8;
//         src = dev->long_addr;
//     }
//     else {
//         src_len = 2;
//         src = dev->short_addr;
//     }
//     /* fill MAC header, seq should be set by device */
//     if ((res = ieee802154_set_frame_hdr(mhr, src, src_len,
//                                         dst, dst_len,
//                                         src_pan, dst_pan,
//                                         flags, dev->seq++)) < 0) {
//         puts("txtsnd: Error preperaring frame");
//         return 1;
//     }

//     iolist_t iol_hdr = {
//         .iol_next = &iol_data,
//         .iol_base = mhr,
//         .iol_len = (size_t)res
//     };

//     be_uint16_t _dst_pan = byteorder_ltobs(dst_pan);
//     l2util_addr_to_str(dst, dst_len, _addr_str);
//     printf("txtsnd: sending %u bytes to %s", (unsigned)iol_data.iol_len, _addr_str);
//     l2util_addr_to_str((uint8_t*) &_dst_pan, sizeof(dst_pan), _addr_str);
//     printf(" (PAN: %s)\n", _addr_str);

//     res = netdev_ieee802154_minimal_send(&dev->netdev, &iol_hdr);

//     if (res < 0) {
//         puts("txtsnd: Error on sending");
//         return 1;
//     }
//     return 0;
// }


at86rf215_t *at86rf215_subghz = NULL;
at86rf215_t *at86rf215_24ghz = NULL;

int netdev_ieee802154_minimal_init_devs(netdev_event_cb_t cb) {
    unsigned idx = 0;
    puts("Initializing AT86RF215 at86rf215");

    for (unsigned i = 0; i < AT86RF215_NUM; i++) {

        printf("%d out of %d\n", i + 1, AT86RF215_NUM);

        if (IS_USED(MODULE_AT86RF215_SUBGHZ)) {
            puts("Sub-GHz");
            at86rf215_subghz = &at86rf215[idx];
            idx++;
        }

        if (IS_USED(MODULE_AT86RF215_24GHZ)) {
            puts("2.4 GHz");
            at86rf215_24ghz = &at86rf215[idx];
            idx++;
        }

        /* setup the specific driver */
        at86rf215_reg_write(at86rf215_subghz, 0x05, 0x07);
        ztimer_sleep(ZTIMER_USEC, 1000 * US_PER_MS);
        printf("reset and cfg\n");
        at86rf215_setup(at86rf215_subghz, at86rf215_24ghz, &at86rf215_params[0], 0);
        at86rf215_reg_write(at86rf215_subghz, 0x05, 0x07);
        ztimer_sleep(ZTIMER_USEC, 1000 * US_PER_MS);
        printf("reset and cfg\n");
        int res = 0;
        if (at86rf215_subghz) {
            res = _init_driver(&at86rf215_subghz->netdev.netdev, cb);
            if (res) {
                printf("err\n");
                // return res;
            }
        }
        if (at86rf215_24ghz) {
            res = _init_driver(&at86rf215_24ghz->netdev.netdev, cb);
            if (res) {
                return res;
            }
        }
    }

    return 0;
}

static const shell_command_t shell_commands[] = {
    { "batmon", "Enable the battery monitor", cmd_enable_batmon },
    { "set_trim", "at86rf215: Set the trim value of the crystal oscillator", cmd_set_trim },
    { "set_clko", "at86rf215: Configure the Clock Output pin", cmd_set_clock_out },
    { "get_random", "at86rf215: Get random values from the radio", cmd_get_random },
    { NULL, NULL, NULL }
};


void *rf_thread(void *arg)
{

    while (1)
    {
    printf("send msg\n");
        // at86rf215_send(at86rf215_subghz, data, sizeof(data));
    // netdev_ieee802154_minimal_send(&at86rf215_subghz->netdev.netdev, item);
    ztimer_sleep(ZTIMER_USEC, 1 * US_PER_SEC);
    }
}

int main(void)
{
    puts("Test application for AT86RF215 IEEE 802.15.4 device driver");

    int res = netdev_ieee802154_minimal_init();
    if (res) {
        puts("Error initializing at86rf215");
        return 1;
    }

    // /* create battery monitor thread */
    thread_create(batmon_stack, sizeof(batmon_stack), THREAD_PRIORITY_MAIN - 1,
                  THREAD_CREATE_STACKTEST, batmon_thread, NULL, "batmon");



    /* start the shell */
    puts("Initialization successful - starting the shell now");



    at86rf215_reset_and_cfg(at86rf215_subghz);
    uint8_t ret = at86rf215_get_phy_mode(&at86rf215[0]);
    printf("PHY = %d\n", ret);
    uint8_t val = at86rf215_reg_read(at86rf215_subghz, 0x300);
    printf("REG_BBC0_IRQM = 0x%x\n", val);
    val = 0;
    val = at86rf215_reg_read(at86rf215_subghz, 0x100);
    printf("REG_RF09_IRQM = 0x%x\n", val);
    // thread_create(rf_stack, sizeof(rf_stack), THREAD_PRIORITY_MAIN - 1,
    //               THREAD_CREATE_STACKTEST, rf_thread, NULL, "rf");

    char *str = "Hello 123213131313132131313132113131231313123";
    char data[] = "Hello 123213131313132131313132113131231313123";
    // Выделяем память для элемента списка
    iolist_t *item = (iolist_t *)malloc(sizeof(iolist_t));
    if (item == NULL)
    {
        perror("Failed to allocate memory for iolist item");
    }
#if (DEVICE_ROLE == DEV_RX)
    at86rf215_cca(at86rf215_subghz);
#endif
        // Инициализируем поля структуры
    item->iol_next = NULL;
    item->iol_base = str;
    item->iol_len = strlen(str);
    static uint16_t value;
    netdev_ieee802154_get(&at86rf215_subghz->netdev, NETOPT_MAX_PDU_SIZE, &value, sizeof(uint16_t));
    printf("pdu size = %d\n", value);
    static uint8_t option = 1;
    static uint8_t scheme = 5;
    // at86rf215_subghz->netdev.netdev.driver->set(&at86rf215_subghz->netdev.netdev, NETOPT_MR_OFDM_OPTION,(uint8_t *)&option, sizeof(uint8_t));
    int err = -3;
    err = at86rf215_configure_OFDM(at86rf215_subghz, option, scheme);
    printf("OFDM configure ret = %d\n", err);
    at86rf215_set_txpower(at86rf215_subghz, -10);
    // err = at86rf215_OFDM_set_option(at86rf215_subghz, 0);
    // printf("set option ret = %d\n", err);
    // err = at86rf215_OFDM_set_scheme(at86rf215_subghz, 3);
    // printf("set scheme ret = %d\n", err);
    printf("current OFDM option = %d\n", at86rf215_OFDM_get_option(at86rf215_subghz));
    printf("current OFDM scheme = %d\n", at86rf215_OFDM_get_scheme(at86rf215_subghz));
    printf("current TX Power = %d\n", at86rf215_get_txpower(at86rf215_subghz));
    printf("CONFIG_IEEE802154_DEFAULT_PHY_MODE = %d\n", CONFIG_IEEE802154_DEFAULT_PHY_MODE);
    // send(0,0023,);
    // while (1)
    // {
// #if (DEVICE_ROLE == DEV_TX)
        // printf("send msg\n");
        // // at86rf215_send(at86rf215_subghz, data, sizeof(data));
        // netdev_ieee802154_minimal_send(&at86rf215_subghz->netdev.netdev, item);
        // ztimer_sleep(ZTIMER_USEC, 1 * US_PER_SEC);
// #elif (DEVICE_ROLE == DEV_RX)
//         val = at86rf215_reg_read(at86rf215_subghz, 0x02);
//          printf("REG_BBC0_IRQS = 0x%x\n", val);
//         ztimer_sleep(ZTIMER_USEC, 1 * US_PER_SEC);
// #endif
    // }
    char line_buf[SHELL_DEFAULT_BUFSIZE];
    shell_run(shell_commands, line_buf, SHELL_DEFAULT_BUFSIZE);


    return 0;
}
