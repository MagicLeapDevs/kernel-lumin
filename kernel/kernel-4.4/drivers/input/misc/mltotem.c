/*
 * mltotem.c
 *
 * Magic Leap Totem Input Driver
 *
 * Copyright (c) 2017-2018, Magic Leap, Inc.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/spi/spi.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/miscdevice.h>
#include <linux/circ_buf.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/wait.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/input/mltotem_ioctl.h>
#include <misc/ml_wearable.h>

#define CREATE_TRACE_POINTS

#undef TRACE_SYSTEM
#define TRACE_SYSTEM mltotem
#include <trace/events/mltotem.h>

/**
 * enum mlt_pkt_type - type of totem packet.
 * @TOTEM_PKT_INPUT:    Input packet.
 * @TOTEM_PKT_LED:      LED control packet.
 * @TOTEM_PKT_HAPTICS:  Haptics feedback packet.
 * @TOTEM_PKT_RAW:      Raw data packet.
 * @TOTEM_PKT_AGC:      AGC packet.
 */
enum mlt_pkt_type {
	TOTEM_PKT_INPUT         = 0x00,
	TOTEM_PKT_LED           = 0x01,
	TOTEM_PKT_HAPTICS       = 0x02,
	TOTEM_PKT_RAW           = 0x03,
	TOTEM_PKT_AGC           = 0x04,
};

#define TOTEM_TOUCH_ABS_X_MAX          0xfff
#define TOTEM_TOUCH_ABS_Y_MAX          0xfff
#define TOTEM_TOUCH_PRESSURE_MAX       0xff
#define TOTEM_ANALOG_TRIGGER_MIN_TSHLD 0x14
#define TOTEM_ANALOG_TRIGGER_MAX       0xff
#define TOTEM_ANALOG_TRIGGER_PULLED(x) ((x) >= TOTEM_ANALOG_TRIGGER_MIN_TSHLD)

struct mlt_button_map {
	u8  button;
	u16 key_code;
};

static const struct mlt_button_map s_btn_map[] = {
	{ 0x01, BTN_1 },
	{ 0x02, BTN_2 },
};
#define TOTEM_NUM_OF_BTNS (sizeof(s_btn_map) / sizeof(struct mlt_button_map))
#define TOTEM_BTNS_SUPPORTED (0x01 | 0x02)  /**< BTN_1 and BTN_2 */

struct mlt_input_data {
	/* Bytes 0-17 */
	u16    accel[3];
	u16    gyro[3];
	u16    mgnt[3];
	/* Bytes 18-21 */
	u32    touch_x         : 12;
	u32    touch_y         : 12;
	u32    touch_force     : 8;
	/* Bytes 22-23 */
	u16     touch_state     : 2;
	u16     buttons         : 2;
	u16     gesture         : 2;
	u16     em_agc          : 3;
	u16     unused          : 7;
	/* Bytes 24-27 */
	u32    trigger         : 8;
	u32    tstamp          : 24;
} __packed;

#define TOTEM_INPUT_EVT_SZ sizeof(struct mlt_input_data)

struct mlt_pkt {
	u8                packet_type; /**< One of mlt_pkt_type */
	union {
		struct mlt_led         led;
		struct mlt_haptics     haptics;
		char                   raw[MLT_MAX_RAW_PAYLOAD_SZ];
		struct mlt_input_data  input;
		u8                     agc;
	} __packed u;
} __packed;

struct mlt_gzl_pkt {
	u8             totem_id;    /**< One of TOTEM_ID_x */
	uint32_t       time_sync;   /**< MTime - wearable time */
	u8             payload_len; /**< Size of pkt */
	struct mlt_pkt pkt;
} __packed;

#define TOTEM_PKT_SZ             sizeof(struct mlt_gzl_pkt)
#define TOTEM_PKT_TYPE_SZ        sizeof(((struct mlt_pkt *)0)->packet_type)
#define TOTEM_QUEUE_LEN          (1 << 3)
#define TOTEM_BUF_LEN            (TOTEM_PKT_SZ * TOTEM_QUEUE_LEN)
#define TOTEM_TIMEOUT_MS         1000
#define RTC_TICKS_PER_SEC        (32768)
#define RTC_TICKS_PER_SEC_DIV_64 (RTC_TICKS_PER_SEC / 64)
#define RTC_US_PER_SEC_DIV_64    (1000 * 1000 / 64)

#define RING_BUF_SPACE(circ)    \
	CIRC_SPACE((circ)->head, (circ)->tail, TOTEM_QUEUE_LEN)

#define RING_BUF_CNT(circ)      \
	CIRC_CNT((circ)->head, (circ)->tail, TOTEM_QUEUE_LEN)

#define RING_BUF_HEAD(circ)     \
	((circ)->buf + (circ)->head * TOTEM_PKT_SZ)

#define RING_BUF_TAIL(circ)     \
	((circ)->buf + (circ)->tail * TOTEM_PKT_SZ)

#define RING_BUF_INC_HEAD(circ) \
	((circ)->head = (((circ)->head + 1) & (TOTEM_QUEUE_LEN - 1)))

#define RING_BUF_INC_TAIL(circ) \
	((circ)->tail = (((circ)->tail + 1) & (TOTEM_QUEUE_LEN - 1)))

#define RING_BUF_ADD(circ, pkt, ignore_overrun)                 \
	do {                                                    \
		if (unlikely(RING_BUF_SPACE(circ) == 0)) {      \
			if (ignore_overrun)                     \
				RING_BUF_INC_TAIL(circ);        \
			else                                    \
				BUG();                          \
		}                                               \
		memcpy(RING_BUF_HEAD(circ), pkt, sizeof(*pkt)); \
		RING_BUF_INC_HEAD(circ);                        \
	} while (0)

#define RING_BUF_FETCH(circ, pkt)                               \
	do {                                                    \
		if (unlikely(RING_BUF_CNT(circ) == 0))          \
			BUG();                                  \
		memcpy(pkt, RING_BUF_TAIL(circ), sizeof(*pkt)); \
		RING_BUF_INC_TAIL(circ);                        \
	} while (0)

enum mlt_totem_caps {
	TOTEM_CAPS_IMU         = 0x01,
	TOTEM_CAPS_TOUCH       = 0x02,
	TOTEM_CAPS_BTN         = 0x04,
	TOTEM_CAPS_TRIGGER     = 0x08,
	TOTEM_CAPS_EM_AGC      = 0x10,
};

#define TOTEM_GRIP_CAPS        0x1F       /* Capabilities of Grip totem */
#define TOTEM_IDEV_MAX         2          /* Max number of totems */
#define TOTEM_ID_TO_IDX(id)    ((id) - 1) /* Convert id to internal index */
#define TOTEM_IDX_TO_ID(idx)   ((idx) + 1) /* Convert internal index to id */
#define IS_TOTEM_ID_VALID(id)  ((id) == TOTEM_ID_1 || (id) == TOTEM_ID_2)
#define TOTEM_IDEV_NAME_LEN    32         /* Max length of input dev name */

struct mlt_input_dev {
	u8                      caps;
	u8                      btn_scan;
	u8                      em_agc;
	u8                      touch_state;
	u8                      gesture;
	u8                      trigger;
	struct input_dev        *idev_imu;
	char                    idev_imu_name[TOTEM_IDEV_NAME_LEN];
	struct input_dev        *idev_ctrl;
	char                    idev_ctrl_name[TOTEM_IDEV_NAME_LEN];
};

struct mlt {
	struct spi_device       *spi;
	struct mlt_input_dev    input[TOTEM_IDEV_MAX];
	struct miscdevice       mdev;
	struct circ_buf         rx_queue;
	struct circ_buf         tx_queue;
	wait_queue_head_t       rx_wait_queue;
	wait_queue_head_t       tx_wait_queue;
	struct mutex            queue_lock;
	struct extsync_event    int_time;
};

/* Local functions pre-definition */
static int     mlt_cdev_open(struct inode *, struct file *);
static long    mlt_cdev_ioctl(struct file *, unsigned int, unsigned long);
static int     mlt_cdev_release(struct inode *, struct file *);

static const struct file_operations mlt_cdev_fops = {
	.owner          = THIS_MODULE,
	.open           = mlt_cdev_open,
	.unlocked_ioctl = mlt_cdev_ioctl,
	.release        = mlt_cdev_release,
	.llseek         = no_llseek,
};

/* Convert 64 bit us value to 24 bit 32khz */
/* for Nordic RTC timestamp compare.       */
uint32_t us_conv_val_to_24rtc(uint64_t us)
{
	uint64_t ticks = (us * RTC_TICKS_PER_SEC_DIV_64) /
			  RTC_US_PER_SEC_DIV_64;

	return (uint32_t) (ticks & 0x00FFFFFF);
}

static int (*irq_custom_callback)(struct extsync_event *);

void mltotem_register_irq_custom_callback(int (*callback)
						(struct extsync_event *))
{
	irq_custom_callback = callback;
}
EXPORT_SYMBOL(mltotem_register_irq_custom_callback);

void mltotem_deregister_irq_custom_callback(void)
{
	irq_custom_callback = NULL;
}
EXPORT_SYMBOL(mltotem_deregister_irq_custom_callback);


static void mlt_queue_rx_pkt(struct mlt *totem, const struct mlt_gzl_pkt *pkt)
{
	mutex_lock(&totem->queue_lock);
	/* Overwrite the head in case of overrun condition */
	RING_BUF_ADD(&totem->rx_queue, pkt, true);
	wake_up_interruptible(&totem->rx_wait_queue);
	mutex_unlock(&totem->queue_lock);
}

static int mlt_queue_tx_pkt(struct mlt *totem, const struct mlt_gzl_pkt *pkt)
{
	int ret = msecs_to_jiffies(TOTEM_TIMEOUT_MS);

	mutex_lock(&totem->queue_lock);
	while (RING_BUF_SPACE(&totem->tx_queue) == 0) {
		mutex_unlock(&totem->queue_lock);
		ret = wait_event_interruptible_timeout(totem->tx_wait_queue,
					RING_BUF_SPACE(&totem->tx_queue) > 0,
					ret);
		if (ret == 0)
			return -ETIMEDOUT;
		else if (ret < 0)
			return ret;
		mutex_lock(&totem->queue_lock);
	}
	RING_BUF_ADD(&totem->tx_queue, pkt, false);
	mutex_unlock(&totem->queue_lock);
	return 0;
}

static int mlt_led_set_ioctl(struct mlt *totem,
			     const struct mlt_led_req __user *req)
{
	struct mlt_gzl_pkt     gzl_pkt;
	struct mlt_led_req     led_req;

	if (copy_from_user(&led_req, req, sizeof(struct mlt_led_req)))
		return -EFAULT;

	if (!IS_TOTEM_ID_VALID(led_req.totem_id))
		return -EINVAL;

	gzl_pkt.totem_id = led_req.totem_id;
	gzl_pkt.payload_len = TOTEM_PKT_TYPE_SZ + sizeof(struct mlt_led);
	gzl_pkt.pkt.packet_type = TOTEM_PKT_LED;
	memcpy(&gzl_pkt.pkt.u.led, &led_req.led, sizeof(struct mlt_led));

	return mlt_queue_tx_pkt(totem, &gzl_pkt);
}

static int mlt_haptics_set_ioctl(struct mlt *totem,
				 const struct mlt_haptics_req __user *req)
{
	struct mlt_gzl_pkt     gzl_pkt;
	struct mlt_haptics_req haptics_req;

	if (copy_from_user(&haptics_req, req, sizeof(struct  mlt_haptics_req)))
		return -EFAULT;

	if (!IS_TOTEM_ID_VALID(haptics_req.totem_id))
		return -EINVAL;

	gzl_pkt.totem_id = haptics_req.totem_id;
	gzl_pkt.payload_len = TOTEM_PKT_TYPE_SZ;
	switch (haptics_req.driver) {
	case MLT_HAPTICS_DRV2605:
		gzl_pkt.payload_len += sizeof(struct mlt_haptics_drv2605);
		break;
	case MLT_HAPTICS_AFT14A901:
		gzl_pkt.payload_len += sizeof(struct mlt_haptics_aft14a901);
		break;
	default:
		return -EINVAL;
	}
	gzl_pkt.pkt.packet_type = TOTEM_PKT_HAPTICS;
	memcpy(&gzl_pkt.pkt.u.haptics, &haptics_req.haptics,
	       sizeof(struct mlt_haptics));

	return mlt_queue_tx_pkt(totem, &gzl_pkt);
}

static int mlt_raw_get_ioctl(struct mlt *totem,
			     struct mlt_raw_pkt __user *resp)
{
	struct mlt_gzl_pkt     gzl_pkt;
	struct mlt_raw_pkt     raw_pkt;
	int                    ret;

	memset(&raw_pkt, 0, sizeof(struct mlt_raw_pkt));
	mutex_lock(&totem->queue_lock);
	while (RING_BUF_CNT(&totem->rx_queue) == 0) {
		mutex_unlock(&totem->queue_lock);
		ret = wait_event_interruptible(totem->rx_wait_queue,
					RING_BUF_CNT(&totem->rx_queue) > 0);
		if (ret)
			return ret;
		mutex_lock(&totem->queue_lock);
	}
	RING_BUF_FETCH(&totem->rx_queue, &gzl_pkt);
	mutex_unlock(&totem->queue_lock);

	if (!IS_TOTEM_ID_VALID(gzl_pkt.totem_id) ||
	    gzl_pkt.payload_len <= TOTEM_PKT_TYPE_SZ ||
	    gzl_pkt.payload_len > MLT_MAX_RAW_PAYLOAD_SZ + TOTEM_PKT_TYPE_SZ) {
		dev_err(&totem->spi->dev, "%s: invalid pkt len = %d\n",
			__func__, gzl_pkt.payload_len);
		return -EFAULT;
	}

	raw_pkt.totem_id = gzl_pkt.totem_id;
	raw_pkt.len = gzl_pkt.payload_len - TOTEM_PKT_TYPE_SZ;
	memcpy(&raw_pkt.raw, &gzl_pkt.pkt.u.raw, raw_pkt.len);
	if (copy_to_user(resp, &raw_pkt, sizeof(struct mlt_raw_pkt)))
		return -EFAULT;

	return 0;
}

static int mlt_raw_set_ioctl(struct mlt *totem,
			     const struct mlt_raw_pkt __user *req)
{
	struct mlt_gzl_pkt     gzl_pkt;
	struct mlt_raw_pkt     raw_pkt;

	if (copy_from_user(&raw_pkt, req, sizeof(struct mlt_raw_pkt)))
		return -EFAULT;

	if (!IS_TOTEM_ID_VALID(raw_pkt.totem_id))
		return -EINVAL;

	if (raw_pkt.len == 0 || raw_pkt.len > MLT_MAX_RAW_PAYLOAD_SZ) {
		dev_err(&totem->spi->dev, "%s: invalid pkt len = %d\n",
			__func__, raw_pkt.len);
		return -EINVAL;
	}

	gzl_pkt.totem_id = raw_pkt.totem_id;
	gzl_pkt.pkt.packet_type = TOTEM_PKT_RAW;
	gzl_pkt.payload_len = TOTEM_PKT_TYPE_SZ + raw_pkt.len;
	memcpy(&gzl_pkt.pkt.u.raw, &raw_pkt.raw, raw_pkt.len);

	return mlt_queue_tx_pkt(totem, &gzl_pkt);
}

static int mlt_agc_set_ioctl(struct mlt *totem,
			     const struct mlt_agc_req __user *req)
{
	struct mlt_gzl_pkt     gzl_pkt;
	struct mlt_agc_req     agc_req;

	if (copy_from_user(&agc_req, req, sizeof(struct mlt_agc_req)))
		return -EFAULT;

	if (!IS_TOTEM_ID_VALID(agc_req.totem_id) ||
	    agc_req.agc > TOTEM_INPUT_EM_AGC_MAX)
		return -EINVAL;

	gzl_pkt.totem_id = agc_req.totem_id;
	gzl_pkt.payload_len = TOTEM_PKT_TYPE_SZ + sizeof(gzl_pkt.pkt.u.agc);
	gzl_pkt.pkt.packet_type = TOTEM_PKT_AGC;
	gzl_pkt.pkt.u.agc = agc_req.agc;

	return mlt_queue_tx_pkt(totem, &gzl_pkt);
}

static int mlt_agc_get_ioctl(struct mlt *totem, struct mlt_agc_req __user *req)
{
	struct mlt_agc_req agc_req;

	if (get_user(agc_req.totem_id, &req->totem_id))
		return -EFAULT;

	if (!IS_TOTEM_ID_VALID(agc_req.totem_id))
		return -EINVAL;

	mutex_lock(&totem->queue_lock);
	agc_req.agc = totem->input[TOTEM_ID_TO_IDX(agc_req.totem_id)].em_agc;
	mutex_unlock(&totem->queue_lock);
	if (put_user(agc_req.agc, &req->agc))
		return -EFAULT;

	return 0;
}

static int mlt_cdev_open(struct inode *inode, struct file *filep)
{
	struct mlt *totem = container_of(filep->private_data,
					 struct mlt, mdev);

	dev_dbg(&totem->spi->dev, "%s\n", __func__);
	return 0;
}

static long mlt_cdev_ioctl(struct file *filep, unsigned int cmd,
			   unsigned long arg)
{
	struct mlt *totem = container_of(filep->private_data,
					 struct mlt, mdev);
	void __user *user_arg = (void __user *)arg;
	int ret;

	switch (cmd) {
	case MLT_IOCTL_LED_SET:
		ret = mlt_led_set_ioctl(totem, user_arg);
		break;
	case MLT_IOCTL_HAPTICS_SET:
		ret = mlt_haptics_set_ioctl(totem, user_arg);
		break;
	case MLT_IOCTL_RAW_GET:
		ret = mlt_raw_get_ioctl(totem, user_arg);
		break;
	case MLT_IOCTL_RAW_SET:
		ret = mlt_raw_set_ioctl(totem, user_arg);
		break;
	case MLT_IOCTL_AGC_SET:
		ret = mlt_agc_set_ioctl(totem, user_arg);
		break;
	case MLT_IOCTL_AGC_GET:
		ret = mlt_agc_get_ioctl(totem, user_arg);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int mlt_cdev_release(struct inode *inode, struct file *filep)
{
	struct mlt *totem = container_of(filep->private_data,
					 struct mlt, mdev);

	dev_dbg(&totem->spi->dev, "%s\n", __func__);
	return 0;
}

static int mlt_validate_input_pkt(struct device *dev,
				  const struct mlt_gzl_pkt *rx_pkt)
{
	struct mlt *totem = dev_get_drvdata(dev);

	/* Check totem id */
	if (!IS_TOTEM_ID_VALID(rx_pkt->totem_id)) {
		dev_dbg(dev, "rx: invalid totem_id = %d\n", rx_pkt->totem_id);
		return -EIO;
	}
	/* Check if input device is registered */
	if (!totem->input[TOTEM_ID_TO_IDX(rx_pkt->totem_id)].idev_imu &&
	    !totem->input[TOTEM_ID_TO_IDX(rx_pkt->totem_id)].idev_ctrl) {
		dev_dbg(dev, "totem id = %d: input devs are not registered\n",
			rx_pkt->totem_id);
		return -EIO;
	}
	/* Check payload len */
	if (rx_pkt->payload_len != TOTEM_PKT_TYPE_SZ +
				   sizeof(struct mlt_input_data)) {
		dev_dbg(dev, "rx: invalid payload len = %d\n",
			rx_pkt->payload_len);
		return -EIO;
	}
	return 0;
}

static void mlt_send_dof_evt(struct input_dev *idev,
			     u8 caps,
			     const struct mlt_input_data *d)
{
	trace_mlt_send_dof_evt(caps, le32_to_cpu(d->tstamp));
	if (caps & TOTEM_CAPS_IMU) {
		input_report_abs(idev, ABS_ACCEL_X, le32_to_cpu(d->accel[0]));
		input_report_abs(idev, ABS_ACCEL_Y, le32_to_cpu(d->accel[1]));
		input_report_abs(idev, ABS_ACCEL_Z, le32_to_cpu(d->accel[2]));

		input_report_abs(idev, ABS_GYRO_X, le32_to_cpu(d->gyro[0]));
		input_report_abs(idev, ABS_GYRO_Y, le32_to_cpu(d->gyro[1]));
		input_report_abs(idev, ABS_GYRO_Z, le32_to_cpu(d->gyro[2]));

		input_report_abs(idev, ABS_MGNT_X, le32_to_cpu(d->mgnt[0]));
		input_report_abs(idev, ABS_MGNT_Y, le32_to_cpu(d->mgnt[1]));
		input_report_abs(idev, ABS_MGNT_Z, le32_to_cpu(d->mgnt[2]));
	}

	input_event(idev, EV_MSC, MSC_TIMESTAMP, le32_to_cpu(d->tstamp));
}

static void mlt_send_touch_evt(struct input_dev *idev,
			       const struct mlt_input_data *idata,
			       u8 *prev_touch_state,
			       u8 *prev_gesture)
{
	if (idata->touch_x || idata->touch_y) {
		input_report_abs(idev, ABS_MT_POSITION_X,
				 le32_to_cpu(idata->touch_x));
		input_report_abs(idev, ABS_MT_POSITION_Y,
				 le32_to_cpu(idata->touch_y));
		input_report_abs(idev, ABS_MT_PRESSURE, idata->touch_force);
		input_mt_sync(idev);
	}

	if ((idata->gesture != *prev_gesture) && idata->gesture)
		input_event(idev, EV_MSC, MSC_GESTURE, idata->gesture);

	if (idata->touch_state != *prev_touch_state)
		input_event(idev, EV_MSC, MSC_ACTIVITY, idata->touch_state);

	*prev_touch_state = idata->touch_state;
	*prev_gesture = idata->gesture;
}

static void mlt_send_button_evt(struct input_dev *idev,
				const struct mlt_input_data *idata,
				u8 *prev_status)
{
	u8 new_status = idata->buttons;
	u8 changed = (new_status ^ *prev_status) & TOTEM_BTNS_SUPPORTED;
	bool pressed;
	int  bit_pos;

	/*
	 * While not all the buttons that have changed
	 * their state were proccessed do the following:
	 * - Find the first button that has changed its state.
	 * - Check if it's pressed or not.
	 * - Report key envent.
	 * - Mask the bit associated with this button.
	 */
	while (changed) {
		bit_pos = __ffs(changed);
		pressed = new_status & BIT(bit_pos);
		input_report_key(idev, s_btn_map[bit_pos].key_code, pressed);
		changed &= ~(1 << bit_pos);
	}
	*prev_status = new_status;
}

static void mlt_send_trigger_evt(struct input_dev *idev,
				const struct mlt_input_data *idata,
				u8 *prev_status)
{
	if (TOTEM_ANALOG_TRIGGER_PULLED(idata->trigger))
		input_report_abs(idev, ABS_ANALOG_TRIGGER, idata->trigger);
	else if (TOTEM_ANALOG_TRIGGER_PULLED(*prev_status))
		input_report_abs(idev, ABS_ANALOG_TRIGGER, 0);
	*prev_status = idata->trigger;
}

static void mlt_send_em_agc_evt(struct input_dev *idev,
				const struct mlt_input_data *idata,
				u8 *prev_status)
{
	if (idata->em_agc != *prev_status) {
		*prev_status = idata->em_agc;
		input_report_abs(idev, ABS_GAIN, idata->em_agc);
	}
}

static void mlt_spi_io(struct spi_device *spi)
{
	struct mlt           *totem = spi_get_drvdata(spi);
	struct mlt_input_dev *input = NULL;
	struct mlt_gzl_pkt   rx_pkt;
	struct mlt_gzl_pkt   tx_pkt;
	int                  err;
	struct spi_message   msg;
	struct spi_transfer  t = {
		.tx_buf = &tx_pkt,
		.rx_buf = &rx_pkt,
		.len    = TOTEM_PKT_SZ,
	};

	memset(&tx_pkt, 0, sizeof(struct mlt_gzl_pkt));
	memset(&rx_pkt, 0, sizeof(struct mlt_gzl_pkt));

	spi_message_init(&msg);
	mutex_lock(&totem->queue_lock);
	/* Check if we have anything to send */
	if (RING_BUF_CNT(&totem->tx_queue) > 0) {
		RING_BUF_FETCH(&totem->tx_queue, &tx_pkt);
		print_hex_dump_debug("<out<", DUMP_PREFIX_NONE, 16, 1,
			       &tx_pkt, sizeof(struct mlt_gzl_pkt), 0);
		wake_up_interruptible(&totem->tx_wait_queue);
	}
	tx_pkt.time_sync = us_conv_val_to_24rtc(totem->int_time.mtime);
	mutex_unlock(&totem->queue_lock);

	spi_message_add_tail(&t, &msg);
	err = spi_sync(spi, &msg);
	if (err) {
		dev_dbg(&spi->dev, "spi I/O failure, err = %d\n", err);
		return;
	}

	if (rx_pkt.payload_len == 0 ||
	    rx_pkt.payload_len > sizeof(struct mlt_pkt)) {
		dev_dbg(&spi->dev, "rx: payload_len = %d\n",
			rx_pkt.payload_len);
		return;
	}

	if (rx_pkt.pkt.packet_type == TOTEM_PKT_INPUT) {
		/* Input event data goes to input device */
		/* Sanity check received data */
		if (mlt_validate_input_pkt(&spi->dev, &rx_pkt))
			return;

		input = &totem->input[TOTEM_ID_TO_IDX(rx_pkt.totem_id)];
		if (input->caps & TOTEM_CAPS_EM_AGC) {
			mutex_lock(&totem->queue_lock);
			mlt_send_em_agc_evt(input->idev_imu,
					    &rx_pkt.pkt.u.input,
					    &input->em_agc);
			mutex_unlock(&totem->queue_lock);
		}
		if (input->caps & TOTEM_CAPS_IMU)
			mlt_send_dof_evt(input->idev_imu, input->caps,
					 &rx_pkt.pkt.u.input);
		input_sync(input->idev_imu);
		trace_mlt_input_sync_imu(input->caps, input->idev_imu_name);
		if (input->caps & TOTEM_CAPS_TOUCH)
			mlt_send_touch_evt(input->idev_ctrl,
					   &rx_pkt.pkt.u.input,
					   &input->touch_state,
					   &input->gesture);
		if (input->caps & TOTEM_CAPS_BTN)
			mlt_send_button_evt(input->idev_ctrl,
					    &rx_pkt.pkt.u.input,
					    &input->btn_scan);
		if (input->caps & TOTEM_CAPS_TRIGGER)
			mlt_send_trigger_evt(input->idev_ctrl,
					     &rx_pkt.pkt.u.input,
					     &input->trigger);
		input_sync(input->idev_ctrl);
		trace_mlt_input_sync_ctrl(input->caps, input->idev_ctrl_name);
	} else if (rx_pkt.pkt.packet_type == TOTEM_PKT_RAW) {
		/* raw data packet goes to rx queue */
		print_hex_dump_debug("<in<", DUMP_PREFIX_NONE, 16, 1,
				     &rx_pkt, sizeof(struct mlt_gzl_pkt), 0);
		mlt_queue_rx_pkt(totem, &rx_pkt);
	}
}

static irqreturn_t mlt_irq_hard(int irq, void *data)
{
	struct mlt *totem = (struct mlt *)data;

	trace_mlt_irq_hard(irq);

	if (irq_custom_callback) {
		/* Zero mtime until we have valid data from callback function */
		if (irq_custom_callback(&totem->int_time) == -1)
			totem->int_time.mtime = 0;
	}
	return IRQ_WAKE_THREAD;
}

static irqreturn_t mlt_irq_threaded(int irq, void *data)
{
	struct mlt *totem = (struct mlt *)data;

	mlt_spi_io(totem->spi);
	return IRQ_HANDLED;
}

static int mlt_input_dev_register(struct device *parent,
				  u8 idx,
				  u8 caps)
{
	struct mlt *totem = dev_get_platdata(parent);
	struct mlt_input_dev *input = &totem->input[idx];
	int ret;
	int i;

	if (!IS_TOTEM_ID_VALID(TOTEM_IDX_TO_ID(idx)) || !caps) {
		dev_err(parent, "%s: invalid argument\n", __func__);
		return -EINVAL;
	}

	input->caps = caps;

	if (!(caps & (TOTEM_CAPS_IMU | TOTEM_CAPS_EM_AGC)))
		goto ctrl_dev;

	input->idev_imu = input_allocate_device();
	if (!input->idev_imu) {
		ret = -ENOMEM;
		goto free_mem;
	}

	snprintf(input->idev_imu_name, TOTEM_IDEV_NAME_LEN,
		 "totem-imu-%d", idx);
	input->idev_imu->name = input->idev_imu_name;
	input->idev_imu->id.bustype = BUS_SPI;
	input->idev_imu->dev.parent = parent;

	if (!(caps & TOTEM_CAPS_IMU))
		goto em_agc;

	__set_bit(EV_ABS, input->idev_imu->evbit);
	__set_bit(EV_MSC, input->idev_imu->evbit);
	__set_bit(MSC_TIMESTAMP, input->idev_imu->mscbit);

	input_set_abs_params(input->idev_imu, ABS_ACCEL_X, 0, U16_MAX, 0, 0);
	input_set_abs_params(input->idev_imu, ABS_ACCEL_Y, 0, U16_MAX, 0, 0);
	input_set_abs_params(input->idev_imu, ABS_ACCEL_Z, 0, U16_MAX, 0, 0);

	input_set_abs_params(input->idev_imu, ABS_GYRO_X, 0, U16_MAX, 0, 0);
	input_set_abs_params(input->idev_imu, ABS_GYRO_Y, 0, U16_MAX, 0, 0);
	input_set_abs_params(input->idev_imu, ABS_GYRO_Z, 0, U16_MAX, 0, 0);

	input_set_abs_params(input->idev_imu, ABS_MGNT_X, 0, U16_MAX, 0, 0);
	input_set_abs_params(input->idev_imu, ABS_MGNT_Y, 0, U16_MAX, 0, 0);
	input_set_abs_params(input->idev_imu, ABS_MGNT_Z, 0, U16_MAX, 0, 0);

em_agc:
	__set_bit(ABS_GAIN, input->idev_imu->absbit);
	input_set_abs_params(input->idev_imu, ABS_GAIN, 0,
			     TOTEM_INPUT_EM_AGC_MAX, 0, 0);

	ret = input_register_device(input->idev_imu);
	if (ret) {
		dev_err(parent, "input_register_device(%s) failed: %d\n",
			input->idev_imu_name, ret);
		goto free_mem;
	}

ctrl_dev:
	if (!(caps & (TOTEM_CAPS_TOUCH | TOTEM_CAPS_BTN | TOTEM_CAPS_TRIGGER)))
		return 0;

	input->idev_ctrl = input_allocate_device();
	if (!input->idev_ctrl) {
		ret = -ENOMEM;
		goto free_mem;
	}

	snprintf(input->idev_ctrl_name, TOTEM_IDEV_NAME_LEN,
		 "totem-ctrl-%d", idx);
	input->idev_ctrl->name = input->idev_ctrl_name;
	input->idev_ctrl->id.bustype = BUS_SPI;
	input->idev_ctrl->dev.parent = parent;

	if (caps & TOTEM_CAPS_TOUCH) {
		__set_bit(BTN_TOUCH, input->idev_ctrl->keybit);
		__set_bit(EV_KEY, input->idev_ctrl->evbit);
		__set_bit(EV_ABS, input->idev_ctrl->evbit);
		input_set_abs_params(input->idev_ctrl, ABS_MT_POSITION_X,
				     0, TOTEM_TOUCH_ABS_X_MAX, 0, 0);
		input_set_abs_params(input->idev_ctrl, ABS_MT_POSITION_Y,
				     0, TOTEM_TOUCH_ABS_Y_MAX, 0, 0);
		input_set_abs_params(input->idev_ctrl, ABS_MT_PRESSURE, 0,
				     TOTEM_TOUCH_PRESSURE_MAX, 0, 0);
		input_set_capability(input->idev_ctrl, EV_MSC, MSC_ACTIVITY);
		input_set_capability(input->idev_ctrl, EV_MSC, MSC_GESTURE);
	}
	if (caps & TOTEM_CAPS_BTN) {
		__set_bit(EV_KEY, input->idev_ctrl->evbit);
		for (i = 0; i < TOTEM_NUM_OF_BTNS; i++) {
			if (s_btn_map[i].key_code == KEY_RESERVED)
				continue;
			__set_bit(s_btn_map[i].key_code,
				  input->idev_ctrl->keybit);
		}
	}
	if (caps & TOTEM_CAPS_TRIGGER) {
		__set_bit(EV_ABS, input->idev_imu->evbit);
		input_set_abs_params(input->idev_ctrl, ABS_ANALOG_TRIGGER, 0,
				     TOTEM_ANALOG_TRIGGER_MAX, 0, 0);
	}

	ret = input_register_device(input->idev_ctrl);
	if (ret) {
		dev_err(parent, "input_register_device(%s) failed: %d\n",
			input->idev_ctrl_name, ret);
		goto free_mem;
	}

	return 0;

free_mem:
	input_free_device(input->idev_ctrl);
	input->idev_ctrl = NULL;
	input_free_device(input->idev_imu);
	input->idev_imu = NULL;
	input->caps = 0;
	return ret;
}

static void mlt_input_dev_deregister(struct device *parent, u8 idx)
{
	struct mlt *totem = dev_get_platdata(parent);
	struct mlt_input_dev *input = &totem->input[idx];

	if (input->idev_ctrl) {
		input_unregister_device(input->idev_ctrl);
		input->idev_ctrl = NULL;
	}
	if (input->idev_imu) {
		input_unregister_device(input->idev_imu);
		input->idev_imu = NULL;
	}
	input->caps = 0;
}

#ifdef CONFIG_PM
static int mlt_pm_suspend(struct device *dev)
{
	struct mlt *totem = dev_get_drvdata(dev);

	disable_irq(totem->spi->irq);

	return 0;
}

static int mlt_pm_resume(struct device *dev)
{
	struct mlt *totem = dev_get_drvdata(dev);

	enable_irq(totem->spi->irq);

	return 0;
}

static const struct dev_pm_ops mlt_pm_ops = {
	.suspend = mlt_pm_suspend,
	.resume = mlt_pm_resume,
};
#endif

static int mlt_probe(struct spi_device *spi)
{
	struct mlt    *totem = NULL;
	struct device *dev = &spi->dev;
	int           i;
	int           err;

	dev_info(dev, "%s: enter\n", __func__);

	if (spi->irq <= 0) {
		dev_err(dev, "no irq defined\n");
		return -ENODEV;
	}
	dev_info(dev, "max speed = %u hz\n", spi->max_speed_hz);

	/* memory allocation */
	totem = kzalloc(sizeof(struct mlt), GFP_KERNEL);
	if (!totem)
		return -ENOMEM;

	totem->spi = spi;
	dev->platform_data = totem;
	spi_set_drvdata(spi, totem);

	totem->tx_queue.head = totem->tx_queue.tail = 0;
	totem->tx_queue.buf = kzalloc(TOTEM_BUF_LEN, GFP_KERNEL);
	if (!totem->tx_queue.buf) {
		err = -ENOMEM;
		goto free_totem_mem;
	}

	totem->rx_queue.head = totem->rx_queue.tail = 0;
	totem->rx_queue.buf = kzalloc(TOTEM_BUF_LEN, GFP_KERNEL);
	if (!totem->rx_queue.buf) {
		err = -ENOMEM;
		goto free_tx_mem;
	}

	/* set all the pointers and init static variables */
	mutex_init(&totem->queue_lock);
	init_waitqueue_head(&totem->rx_wait_queue);
	init_waitqueue_head(&totem->tx_wait_queue);

	/* misc device */
	totem->mdev.minor = MISC_DYNAMIC_MINOR;
	totem->mdev.fops = &mlt_cdev_fops;
	totem->mdev.name = "ml-totem";
	err = misc_register(&totem->mdev);
	if (err) {
		dev_err(dev, "failed to register miscdev, err = %d\n", err);
		goto free_rx_mem;
	}

	/*
	 * TBD: provide external API to add/remove totems.
	 * Temporarily register 2 GRIP totems from probe.
	 */
	for (i = 0; i < TOTEM_IDEV_MAX; i++) {
		err = mlt_input_dev_register(dev, i, TOTEM_GRIP_CAPS);
		if (err)
			goto err_unreg_dev;
	}

	/* we assume that totem controller is always up and running */
	err = devm_request_threaded_irq(dev, spi->irq,
					mlt_irq_hard,
					mlt_irq_threaded,
					IRQF_TRIGGER_RISING | IRQF_ONESHOT,
					"totem_irq", totem);
	if (err) {
		dev_err(dev, "failed to request irq, err: %d\n", err);
		goto err_unreg_dev;
	}

	dev_info(dev, "%s: exit\n", __func__);
	return 0;

err_unreg_dev:
	for (i = 0; i < TOTEM_IDEV_MAX; i++)
		mlt_input_dev_deregister(dev, i);
	misc_deregister(&totem->mdev);

free_rx_mem:
	mutex_destroy(&totem->queue_lock);
	kfree(totem->rx_queue.buf);

free_tx_mem:
	kfree(totem->tx_queue.buf);

free_totem_mem:
	spi_set_drvdata(spi, NULL);
	kfree(totem);
	return err;
}

static int mlt_remove(struct spi_device *spi)
{
	struct mlt *totem = spi_get_drvdata(spi);
	int i;

	for (i = 0; i < TOTEM_IDEV_MAX; i++)
		mlt_input_dev_deregister(&spi->dev, i);
	misc_deregister(&totem->mdev);
	mutex_destroy(&totem->queue_lock);
	kfree(totem->rx_queue.buf);
	kfree(totem->tx_queue.buf);
	kfree(totem);
	spi_set_drvdata(spi, NULL);
	return 0;
}

static void mlt_shutdown(struct spi_device *spi)
{
	struct mlt *totem = spi_get_drvdata(spi);

	disable_irq(totem->spi->irq);
}

static const struct of_device_id mlt_match[] = {
	{ .compatible = "ml,totem" },
	{ },
};
MODULE_DEVICE_TABLE(of, mlt_match);

static struct spi_driver mlt_driver = {
	.driver	= {
		.name	= "mltotem",
		.owner	= THIS_MODULE,
#if defined(CONFIG_PM)
		.pm = &mlt_pm_ops,
#endif
		.of_match_table = mlt_match,
	},
	.probe	= mlt_probe,
	.remove	= mlt_remove,
	.shutdown = mlt_shutdown,
};

module_spi_driver(mlt_driver);

MODULE_AUTHOR("Magic Leap, Inc.");
MODULE_DESCRIPTION("Magic Leap Totem Input Driver");
MODULE_LICENSE("GPL v2");
