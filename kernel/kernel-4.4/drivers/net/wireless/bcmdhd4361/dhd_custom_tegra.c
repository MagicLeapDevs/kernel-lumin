/* Copyright (c) 2016, Magic Leap, Inc. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/wlan_plat.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/etherdevice.h>

#ifdef CONFIG_BROADCOM_WIFI_RESERVED_MEM
extern int dhd_init_wlan_mem(void);
extern void *dhd_wlan_mem_prealloc(int section, unsigned long size);
#endif /* CONFIG_BROADCOM_WIFI_RESERVED_MEM */

#define DHD_WIFI_TURNON_DELAY        200
#define DHD_DT_COMPAT_ENTRY          "android,bcmdhd_wlan4361"
#define DHD_WLAN_REG_ON_PROPNAME     "wl_reg_on"
#define DHD_WLAN_HOST_WAKE_PROPNAME  "wl_host_wake"
#define DHD_WIFI_MAC_ADDR_FILE       "/persist/wlan-mac"

/* Magic Leap range of MAC addresses */
static const u8 wifi_mac_add_ml_id[3] = {0x60, 0x4B, 0xAA};

static int s_wlan_reg_on                = -1;
static int s_wlan_host_wake_up          = -1;
static int s_wlan_host_wake_irq         = 0;

int dhd_wlan_power(int on) {
    if (s_wlan_reg_on < 0) {
        return -ENODEV;
    }

    if (on) {
        if (gpio_direction_output(s_wlan_reg_on, 1)) {
            pr_err("%s: WL_REG_ON is failed to pull up\n", __func__);
            return -EIO;
        }
        if (gpio_get_value(s_wlan_reg_on)) {
            pr_err("WL_REG_ON on-step-2 : [%d]\n",
                   gpio_get_value(s_wlan_reg_on));
        } else {
            pr_err("[%s] gpio value is 0. We need reinit.\n", __func__);
            if (gpio_direction_output(s_wlan_reg_on, 1)) {
                pr_err("%s: WL_REG_ON is failed to pull up\n", __func__);
            }
        }
        msleep(DHD_WIFI_TURNON_DELAY);
    } else {
        if (gpio_direction_output(s_wlan_reg_on, 0)) {
            pr_err("%s: WL_REG_ON is failed to pull up\n", __func__);
            return -EIO;
        }
        if (gpio_get_value(s_wlan_reg_on)) {
            pr_err("WL_REG_ON on-step-2 : [%d]\n",
                   gpio_get_value(s_wlan_reg_on));
        }
    }
    return 0;
}
EXPORT_SYMBOL(dhd_wlan_power);

static int dhd_wlan_reset(int onoff) {
    return 0;
}

static int dhd_wlan_set_carddetect(int val) {
    return 0;
}

static int dhd_wlan_get_mac_addr(unsigned char *buf) {
    struct file *fp;
    int         rdlen;
    char        str[32];
    int         mac[6];
    int         err = 0;
    uint        rand_mac;

    /* open wifi mac address file */
    fp = filp_open(DHD_WIFI_MAC_ADDR_FILE, O_RDONLY, 0);
    if (IS_ERR(fp)) {
        pr_err("%s: cannot open %s\n", __func__, DHD_WIFI_MAC_ADDR_FILE);
        err = -ENOENT;
        goto done;
    }

    /* read wifi mac address file */
    memset(str, 0, sizeof(str));
    rdlen = kernel_read(fp, fp->f_pos, str, 17);
    if (rdlen > 0) {
        fp->f_pos += rdlen;
    }

    if (rdlen != 17) {
        pr_err("%s: bad mac address file %s: len %d != 17",
               __func__, DHD_WIFI_MAC_ADDR_FILE, rdlen);
        err = -ENOENT;
    } else if (sscanf(str, "%x:%x:%x:%x:%x:%x",
                      &mac[0], &mac[1], &mac[2],
                      &mac[3], &mac[4], &mac[5]) != 6) {
        pr_err("%s: bad mac address file %s\n",
               __func__, DHD_WIFI_MAC_ADDR_FILE);
        err = -ENOENT;
    } else {
        pr_err("%s: using wifi mac %02x:%02x:%02x:%02x:%02x:%02x\n",
               __func__, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
        buf[0] = (unsigned char) mac[0];
        buf[1] = (unsigned char) mac[1];
        buf[2] = (unsigned char) mac[2];
        buf[3] = (unsigned char) mac[3];
        buf[4] = (unsigned char) mac[4];
        buf[5] = (unsigned char) mac[5];
        if (!is_valid_ether_addr(buf)) {
            pr_err("%s: invalid mac %02x:%02x:%02x:%02x:%02x:%02x\n",
                   __func__, buf[0], buf[1], buf[2], buf[3], buf[4], buf[5]);
            err = -EINVAL;
        }
    }

    /* close wifi mac address file */
    filp_close(fp, NULL);

done:
    if (err) {
        prandom_seed((uint)jiffies);
        rand_mac = prandom_u32();

        memcpy(buf, wifi_mac_add_ml_id, sizeof(wifi_mac_add_ml_id));
        buf[3] = (unsigned char)rand_mac;
        buf[4] = (unsigned char)(rand_mac >> 8);
        buf[5] = (unsigned char)(rand_mac >> 16);

        pr_info("%s: generated random MAC=%02x:%02x:%02x:%02x:%02x:%02x\n",
                __func__, buf[0], buf[1], buf[2],
                buf[3], buf[4], buf[5]);
    }

    return 0;
}

static int __init dhd_wifi_init_gpio(void) {
    int ret = 0;
    struct device_node *root_node = NULL;

    pr_info("%s: enter\n", __func__);
    root_node = of_find_compatible_node(NULL, NULL, DHD_DT_COMPAT_ENTRY);
    if (!root_node) {
        WARN(1, "Failed to get device node of %s\n", DHD_DT_COMPAT_ENTRY);
        return -ENODEV;
    }

    /* ========== WLAN_REG_ON ============ */
    s_wlan_reg_on = of_get_named_gpio(root_node, DHD_WLAN_REG_ON_PROPNAME, 0);
    pr_info("%s: gpio_wlan_power : %d\n", __func__, s_wlan_reg_on);
    ret = gpio_request_one(s_wlan_reg_on, GPIOF_OUT_INIT_HIGH, "WL_REG_ON");
    if (ret) {
        pr_err("%s: Failed to request gpio %d for WL_REG_ON, err = %d\n",
               __func__, s_wlan_reg_on, ret);
        goto done;
    } else {
        pr_err("%s: gpio_request WL_REG_ON done - WLAN_EN: GPIO %d\n",
               __func__, s_wlan_reg_on);
    }

    ret = gpio_direction_output(s_wlan_reg_on, 1);
    if (ret) {
        pr_err("%s: WL_REG_ON failed to pull up, err = %d\n", __func__, ret);
        goto done;
    } else {
        pr_err("%s: WL_REG_ON is pulled up\n", __func__);
    }

    if (gpio_get_value(s_wlan_reg_on)) {
        pr_info("%s: Initial WL_REG_ON: [%d]\n", __func__,
                gpio_get_value(s_wlan_reg_on));
    }

    /* Wait for DHD_WIFI_TURNON_DELAY due to power stability */
    msleep(DHD_WIFI_TURNON_DELAY);

    /* ========== WLAN_HOST_WAKE ============ */
    s_wlan_host_wake_up = of_get_named_gpio(root_node,
                                            DHD_WLAN_HOST_WAKE_PROPNAME,
                                            0);
    pr_info("%s: gpio_wlan_host_wake : %d\n", __func__,
            s_wlan_host_wake_up);
    ret = gpio_request_one(s_wlan_host_wake_up, GPIOF_IN, "WLAN_HOST_WAKE");
    if (ret) {
        pr_err("%s: Failed to request gpio %d for"
               " WLAN_HOST_WAKE, err = %d\n", __func__,
               s_wlan_host_wake_up, ret);
        goto done;
    } else {
        pr_err("%s: gpio_request WLAN_HOST_WAKE done"
               " - WLAN_HOST_WAKE: GPIO %d\n",
               __func__, s_wlan_host_wake_up);
    }

    gpio_direction_input(s_wlan_host_wake_up);
    s_wlan_host_wake_irq = gpio_to_irq(s_wlan_host_wake_up);

done:
    return ret;
}

struct resource dhd_wlan_resources[] = {
    [0] = {
        .name   = "bcmdhd_wlan_irq",
        .start  = 0, /* Dummy */
        .end    = 0, /* Dummy */
        .flags  = IORESOURCE_IRQ | IORESOURCE_IRQ_SHAREABLE |
#ifdef CONFIG_BCMDHD_PCIE
        IORESOURCE_IRQ_HIGHEDGE,
#else
        IORESOURCE_IRQ_HIGHLEVEL,
#endif /* CONFIG_BCMDHD_PCIE */
    },
};
EXPORT_SYMBOL(dhd_wlan_resources);

struct wifi_platform_data dhd_wlan_control = {
    .set_power      = dhd_wlan_power,
    .set_reset      = dhd_wlan_reset,
    .set_carddetect = dhd_wlan_set_carddetect,
    .get_mac_addr   = dhd_wlan_get_mac_addr,
#ifdef CONFIG_BROADCOM_WIFI_RESERVED_MEM
    .mem_prealloc   = dhd_wlan_mem_prealloc,
#endif /* CONFIG_BROADCOM_WIFI_RESERVED_MEM */
};
EXPORT_SYMBOL(dhd_wlan_control);

static int __init dhd_wlan_init(void) {
    int ret;

    pr_info("%s: enter\n", __func__);
    ret = dhd_wifi_init_gpio();
    if (ret < 0) {
        pr_err("%s: failed to initialize GPIO, ret=%d\n", __func__, ret);
        return ret;
    }

    if (s_wlan_host_wake_irq) {
        dhd_wlan_resources[0].start = s_wlan_host_wake_irq;
        dhd_wlan_resources[0].end   = s_wlan_host_wake_irq;
    }

#ifdef CONFIG_BROADCOM_WIFI_RESERVED_MEM
    ret = dhd_init_wlan_mem();
    if (ret < 0) {
        pr_err("%s: failed to alloc reserved memory, ret=%d\n",
               __func__, ret);
    }
#endif /* CONFIG_BROADCOM_WIFI_RESERVED_MEM */

    return 0;
}
device_initcall(dhd_wlan_init);

