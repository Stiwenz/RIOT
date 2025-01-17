/*
 * Copyright (C) 2022 HAW Hamburg
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for
 * more details.
 */

/**
 * @ingroup     test_utils_netdev_ieee802154_minimal
 * @{
 *
 * @file
 * @brief       Shell commands for netdev Eth minimal test utility module
 *
 * @author      Leandro Lanzieri <leandro.lanzieri@haw-hamburg.de>
 */

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "iolist.h"
#include "kernel_defines.h"
#include "net/netdev/ieee802154.h"
#include "net/ieee802154.h"
#include "net/l2util.h"
#include "od.h"
#include "shell.h"

#include "cmx638.h"

#include "netdev_ieee802154_minimal_internal.h"
#include "test_utils/netdev_ieee802154_minimal.h"
#include "init_dev.h"

#define ENABLE_DEBUG 0
#include "debug.h"

static char _addr_str[IEEE802154_LONG_ADDRESS_LEN * 3];

int send_ieee(int iface, le_uint16_t dst_pan, uint8_t *dst_addr,
                size_t dst_len, char *data, uint16_t data_size);

int ifconfig_list(int idx)
{
    int res;
    uint8_t tmp[IEEE802154_LONG_ADDRESS_LEN];
    netdev_t *netdev = _devices[idx].dev;
    netdev_ieee802154_t *dev = container_of(netdev, netdev_ieee802154_t, netdev);

    netopt_enable_t enable_val;
    uint16_t u16_val;

    printf("Iface %3d  HWaddr: ", idx);
    res = netdev_ieee802154_minimal_get(netdev, NETOPT_ADDRESS, tmp, IEEE802154_SHORT_ADDRESS_LEN);
    l2util_addr_to_str(tmp, IEEE802154_SHORT_ADDRESS_LEN, _addr_str);
    printf("%s", _addr_str);

    printf(", Long HWaddr: ");
    res = netdev_ieee802154_minimal_get(netdev, NETOPT_ADDRESS_LONG, tmp, IEEE802154_LONG_ADDRESS_LEN);
    l2util_addr_to_str(tmp, IEEE802154_LONG_ADDRESS_LEN, _addr_str);
    printf("%s", _addr_str);

    printf(", PAN: 0x%04x", dev->pan);

    res = netdev_ieee802154_minimal_get(netdev, NETOPT_ADDR_LEN, &u16_val, sizeof(u16_val));
    if (res == -ENOTSUP) {
        puts("\n           Address length: ENOTSUP");
    }
    else if (res < 0) {
        printf("\n           Address length: %i", (int)res);
        return 1;
    }
    else {
        printf("\n           Address length: %u", (unsigned)u16_val);
    }

    res = netdev_ieee802154_minimal_get(netdev, NETOPT_SRC_LEN, &u16_val, sizeof(u16_val));
    if (res == -ENOTSUP) {
        puts(", Source address length: ENOTSUP");
    }
    else if (res < 0) {
        printf(", Source address length: %i", (int)res);
        return 1;
    }
    else {
        printf(", Source address length: %u", (unsigned)u16_val);
    }

    res = netdev_ieee802154_minimal_get(netdev, NETOPT_MAX_PDU_SIZE, &u16_val, sizeof(u16_val));
    if (res == -ENOTSUP) {
        puts(", Max.Payload: ENOTSUP");
    }
    else if (res < 0) {
        printf(", Max.Payload: %i", (int)res);
        return 1;
    }
    else {
        printf(", Max.Payload: %u", (unsigned)u16_val);
    }
    printf("\n           Channel: %u", dev->chan);

    res = netdev_ieee802154_minimal_get(netdev, NETOPT_CHANNEL_PAGE, &u16_val, sizeof(u16_val));
    if (res == -ENOTSUP) {
        puts(", Ch.page: ENOTSUP");
    }
    else if (res < 0) {
        printf(", Ch.page: %i", (int)res);
        return 1;
    }
    else {
        printf(", Ch.page: %u", (unsigned)u16_val);
    }

    res = netdev_ieee802154_minimal_get(netdev, NETOPT_TX_POWER, &u16_val, sizeof(u16_val));
    if (res == -ENOTSUP) {
        puts(", TXPower: ENOTSUP");
    }
    else if (res < 0) {
        printf(", TXPower: %i", (int)res);
        return 1;
    }
    else {
        printf(", TXPower: %u", (unsigned)u16_val);
    }
    res = netdev_ieee802154_minimal_get(netdev, NETOPT_IS_WIRED, &u16_val, sizeof(u16_val));
    if (res < 0) {
        puts(", wireless");
    }
    else {
        puts(", wired");
    }

    printf("         ");
    res = netdev_ieee802154_minimal_get(netdev, NETOPT_PRELOADING, &enable_val,
                                        sizeof(netopt_enable_t));
    if ((res > 0) && (enable_val == NETOPT_ENABLE)) {
        printf("  PRELOAD");
    }
    res = netdev_ieee802154_minimal_get(netdev, NETOPT_AUTOACK, &enable_val,
                                        sizeof(netopt_enable_t));
    if ((res > 0) && (enable_val == NETOPT_ENABLE)) {
        printf("  AUTOACK");
    }
    res = netdev_ieee802154_minimal_get(netdev, NETOPT_RAWMODE, &enable_val,
                                        sizeof(netopt_enable_t));
    if ((res > 0) && (enable_val == NETOPT_ENABLE)) {
        printf("  RAW");
    }
    res = netdev_ieee802154_minimal_get(netdev, NETOPT_AUTOCCA, &enable_val,
                                        sizeof(netopt_enable_t));
    if ((res > 0) && (enable_val == NETOPT_ENABLE)) {
        printf("  AUTOCCA");
    }
    res = netdev_ieee802154_minimal_get(netdev, NETOPT_CSMA, &enable_val,
                                        sizeof(netopt_enable_t));
    if ((res > 0) && (enable_val == NETOPT_ENABLE)) {
        printf("  CSMA");
    }
    puts("");

    return 0;
}

int cmd_ifconfig(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    for (unsigned int i = 0; i < NETDEV_IEEE802154_MINIMAL_NUMOF; i++) {
        ifconfig_list(i);
    }
    return 0;
}

static void txtsnd_usage(char *cmd_name)
{
    printf("usage: %s <iface> [<pan>] <addr> <text>\n", cmd_name);
}

char test_data[2000];

extern bool tx_done;
uint32_t current_time = 0;
uint32_t end_time = 0;


static int cmd_txtest(int argc, char **argv)
{
    // size_t res = 0;
    // printf("SEND START\n");
    // printf("arg[%d] = %s\n",argc, argv[1]);
    // printf("arg[%d] = %s\n",argc, argv[2]);
    uint16_t data_size;
    uint8_t packet_qty;
    data_size = 100;
    packet_qty = 1;
    // data_size = atoi(argv[1]);
    // packet_qty = atoi(argv[2]);
    if(!packet_qty)
        packet_qty = 1;
    // printf("data_size = %d\n", data_size);
    uint16_t iface = 0;
    le_uint16_t pan = {
        .u16 = 0x0023
        };
    uint8_t addr[IEEE802154_LONG_ADDRESS_LEN] = {0xA6, 0xF1, 0xDE, 0xEA, 0x89, 0x77, 0xE1, 0xEE};
     current_time = xtimer_now_usec();
     for (size_t i = 0; i < packet_qty; i++)
     {
        while(!tx_done)
            ztimer_sleep(ZTIMER_USEC, 1);
        tx_done = false;
       send_ieee(iface, pan, addr, 8, test_data, data_size);
     }
    end_time = xtimer_now_usec() - current_time;
    printf("tx time = %ld us\n", end_time);
    uint32_t throughput_bytes_s = (uint32_t)(data_size * packet_qty)/((float)((float)end_time/1000000));
    uint32_t throughput_kbits_s = (uint32_t)((throughput_bytes_s * 8)/1000);
    printf("Throughput: %ld bytes/s (%ld bits/s)\n", throughput_bytes_s, throughput_kbits_s);
     return 0;
}


int cmd_txtsnd(int argc, char **argv)
{
    char *text;
    uint8_t addr[IEEE802154_LONG_ADDRESS_LEN];
    int iface, idx = 2;
    size_t res;
    le_uint16_t pan = { 0 };

    switch (argc) {
    case 4:
        break;
    case 5:
        res = l2util_addr_from_str(argv[idx++], pan.u8);
        if ((res == 0) || (res > sizeof(pan))) {
            txtsnd_usage(argv[0]);
            return 1;
        }
        pan.u16 = byteorder_swaps(pan.u16);
        break;
    default:
        txtsnd_usage(argv[0]);
        return 1;
    }

    iface = atoi(argv[1]);
    res = l2util_addr_from_str(argv[idx++], addr);
    if (res == 0) {
        txtsnd_usage(argv[0]);
        return 1;
    }
    text = argv[idx++];
    printf("pan.u8[0] = 0x%x, pan.u8[1] = 0x%x, pan.u16 = 0x%x\n", pan.u8[0], pan.u8[1], pan.u16);
    printf("res = %d, 0x%x\n", res, res);
    return send_ieee(iface, pan, addr, res, text, strlen(text));
}

static char audio_data[27] = {0};

    int res;
    netdev_ieee802154_t *dev;
    uint8_t *src;
    size_t src_len;
    uint8_t mhr[IEEE802154_MAX_HDR_LEN];
    uint8_t flags;
    le_uint16_t src_pan;

int send_ieee(int iface, le_uint16_t dst_pan, uint8_t *dst, size_t dst_len,
                char *data, uint16_t data_size)
{


    if (((unsigned)iface) > (NETDEV_IEEE802154_MINIMAL_NUMOF - 1)) {
        printf("txtsnd: %d is not an interface\n", iface);
        return 1;
    }

    iolist_t iol_data = {
        .iol_base = data,
        .iol_len = data_size
    };

    dev = container_of(_devices[iface].dev, netdev_ieee802154_t, netdev);
    flags = (uint8_t)(dev->flags & NETDEV_IEEE802154_SEND_MASK);
    flags |= IEEE802154_FCF_TYPE_DATA;
    src_pan = byteorder_btols(byteorder_htons(dev->pan));
    if (dst_pan.u16 == 0) {
        dst_pan = src_pan;
    }
    if (dev->flags & NETDEV_IEEE802154_SRC_MODE_LONG) {
        src_len = 8;
        src = dev->long_addr;
    }
    else {
        src_len = 2;
        src = dev->short_addr;
    }
    /* fill MAC header, seq should be set by device */
    if ((res = ieee802154_set_frame_hdr(mhr, src, src_len,
                                        dst, dst_len,
                                        src_pan, dst_pan,
                                        flags, 0)) < 0) {
        puts("txtsnd: Error preperaring frame");
        return 1;
    }

    iolist_t iol_hdr = {
        .iol_next = &iol_data,
        .iol_base = mhr,
        .iol_len = (size_t)res
    };

    // be_uint16_t _dst_pan = byteorder_ltobs(dst_pan);
    // l2util_addr_to_str(dst, dst_len, _addr_str);
    // printf("txtsnd: sending %u bytes to %s", (unsigned)iol_data.iol_len, _addr_str);
    // l2util_addr_to_str((uint8_t*) &_dst_pan, sizeof(dst_pan), _addr_str);
    // printf(" (PAN: %s)\n", _addr_str);

    res = netdev_ieee802154_minimal_send(&dev->netdev, &iol_hdr);

    if (res < 0) {
        puts("txtsnd: Error on sending");
        return 1;
    }
    return 0;
}

/* declare shell commands */
SHELL_COMMAND(ifconfig, "Configure the device", cmd_ifconfig);
SHELL_COMMAND(txtsnd, "Send an IEEE 802.15.4 packet", cmd_txtsnd);
SHELL_COMMAND(txtest, "Send n byte x time in IEEE 802.15.4 format", cmd_txtest);
/** @} */
