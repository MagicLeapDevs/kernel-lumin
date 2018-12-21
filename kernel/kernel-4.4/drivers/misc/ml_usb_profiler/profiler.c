/* Copyright (c) 2017, Magic Leap, Inc. All rights reserved.
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

/******************************************
* To build:
* add the following 2 configs to the
* kernel defconfig file
*   CONFIG_ML_USB_PROFILER=m
*   CONFIG_KPROBES=y
*
* To Run:
* insmod ml_usb_profiler.ko
* echo 1 > /d/tracing/tracing_on
* cat /d/tracing/trace
******************************************/

#include <linux/module.h>
#include <linux/kprobes.h>
#include <linux/kallsyms.h>
#include <linux/usbdevice_fs.h>
#include <linux/device.h>
#include <linux/usb.h>
#include <linux/fs.h>


struct usb_dev_state {
	struct list_head list;      /* state list */
	struct usb_device *dev;
	struct file *file;
	spinlock_t lock;            /* protects the async urb lists */
	struct list_head async_pending;
	struct list_head async_completed;
	wait_queue_head_t wait;     /* wake up if a request completed */
	unsigned int discsignr;
	struct pid *disc_pid;
	const struct cred *cred;
	void __user *disccontext;
	unsigned long ifclaimed;
	u32 secid;
	u32 disabled_bulk_eps;
};

struct usb_port {
	struct usb_device *child;
	struct device dev;
	struct usb_dev_state *port_owner;
	struct usb_port *peer;
	struct dev_pm_qos_request *req;
	enum usb_port_connect_type connect_type;
	u32 location;  /*sub_port_location */
	struct mutex status_lock;
	u8 portnum;
	unsigned int is_superspeed:1;
};

/*
* USB Speed Mode Enum
* USB_SPEED_UNKNOWN = 0,             enumerating
* USB_SPEED_LOW, USB_SPEED_FULL,     usb 1.1
* USB_SPEED_HIGH,                    usb 2.0
* USB_SPEED_WIRELESS,                wireless (usb 2.5)
* USB_SPEED_SUPER,                   usb 3.0
* USB_SPEED_SUPER_PLUS,	             usb 3.1
*/

/************
*  Globals
************/
int urbct;  /*urb blocks counter */
int reg_err; /* jprobe registration error*/


/*** hub ***/

static int hub_port_init_probe(struct usb_hub *hub,
		struct usb_device *udev, int port1, int retry_counter)
{
	enum usb_device_speed	oldspeed = udev->speed;

	trace_printk("Hub:hub_port_init:Port:%d, speed=%d, devNum=%d\n",
		port1, oldspeed, udev->devnum);

	if (oldspeed == USB_SPEED_LOW)
		trace_printk("HUB:hub_port_init,USB_SPEED_LOW(1.1),\n");

	jprobe_return();
	return 0;

}
static int usb_get_device_descriptor_probe(struct usb_device *dev,
		unsigned int size)
{
	trace_printk("Hub:usb_get_device_descriptor, devNum=%d,size=%d\n",
		dev->devnum, size);

	jprobe_return();
	return 0;
}
static int usb_new_device_probe(struct usb_device *udev)
{
	trace_printk("Hub:usb_new_device: idVendor=%04x,idProduct=%04x,devNum=%d,busNum=%d,speed=%d\n",
		le16_to_cpu(udev->descriptor.idVendor),
		le16_to_cpu(udev->descriptor.idProduct), udev->devnum,
		udev->bus->busnum, udev->speed);

	jprobe_return();
	return 0;
}

static void usb_disconnect_probe(struct usb_device **pdev)
{
	struct usb_device *udev = *pdev;

	trace_printk("Hub:USB disconnect,idVendor=%04x, idProduct=%04x,devNum=%d, busNum=%d\n",
		le16_to_cpu(udev->descriptor.idVendor),
		le16_to_cpu(udev->descriptor.idProduct),
		udev->devnum, udev->bus->busnum);

	jprobe_return();
}
/*** driver ***/
static int usb_probe_device_probe(struct device *dev)
{
	struct usb_device_driver *udriver = to_usb_device_driver(dev->driver);
	struct usb_device *udev = to_usb_device(dev);

	trace_printk("Driver:usb_probe_device, drvName=%s, devNum=%d\n",
		udriver->name, udev->devnum);

	jprobe_return();
	return 0;
}


static int usb_unbind_interface_probe(struct device *dev)
{
	struct usb_interface *intf = to_usb_interface(dev);
		struct usb_device *udev;
	udev = interface_to_usbdev(intf);

	trace_printk("Driver:usb_unbind_interface, devNum=%d, intfNum=%d\n",
		udev->devnum, intf->altsetting->desc.bInterfaceNumber);

	jprobe_return();
	return 0;
}

static int usb_driver_claim_interface_probe(struct usb_driver *driver,
				struct usb_interface *iface, void *priv)
{
	struct device *dev;
	struct usb_device *udev;

	dev = &iface->dev;
	udev = interface_to_usbdev(iface);


	trace_printk("driver:usb_driver_claim_interface,DrvName=%s,devNum=%d intfNum=%d\n",
		driver->name, udev->devnum,
		iface->altsetting->desc.bInterfaceNumber);

	jprobe_return();
	return 0;
}

static int usb_probe_interface_probe(struct device *dev)
{
	struct usb_driver *driver = to_usb_driver(dev->driver);
	struct usb_interface *intf = to_usb_interface(dev);
	struct usb_device *udev = interface_to_usbdev(intf);

	trace_printk("driver:usb_probe_intface,drvName=%s,devNum=%d,intfNum=%d\n",
		driver->name, udev->devnum,
		intf->altsetting->desc.bInterfaceNumber);
	jprobe_return();
	return 0;
}
/*** message.c ***/
static int usb_set_configuration_probe(struct usb_device *dev,
			int configuration)
{
	int i;
	struct usb_host_config *cp = NULL;

	for (i = 0; i < dev->descriptor.bNumConfigurations; i++) {
		if (dev->config[i].desc.bConfigurationValue ==
				configuration) {
			cp = &dev->config[i];
			break;
		}
	}

	trace_printk("Message:usb_set_configuration, devNum=%d,configNum=%d numOfIntf=%d\n",
		dev->devnum, configuration, cp->desc.bNumInterfaces);

	 jprobe_return();
	 return 0;
}
static int create_intf_ep_devs_probe(struct usb_interface *intf)
{
	 struct usb_device *udev = interface_to_usbdev(intf);
	 struct usb_host_interface *alt = intf->cur_altsetting;

	trace_printk("Message:create_intf_ep_devs,devNum=%d,NumOfEp=%d\n",
		udev->devnum, alt->desc.bNumEndpoints);

	 jprobe_return();
	 return 0;
}
/*** Endpoint.c ***/
static int usb_create_ep_devs_probe(struct device *parent,
			struct usb_host_endpoint *endpoint,
			struct usb_device *udev)
{
	trace_printk("Endpoint:usb_create_ep_devs,devNum=%d,Ep#%2x, maxPktSize=%d\n",
			udev->devnum, endpoint->desc.bEndpointAddress,
			(endpoint->desc.wMaxPacketSize) & 0x7ff);

	 jprobe_return();
	 return 0;
}

/*** USB ***/
static int usb_bus_notify_probe(struct notifier_block *nb,
					unsigned long action, void *data)
{

	switch (action) {
	case BUS_NOTIFY_ADD_DEVICE:
			trace_printk("bus notify usb:added dev to /dev, add to Sysfs file\n");
		break;

	case BUS_NOTIFY_DEL_DEVICE:
			trace_printk("bus notify usb: bus delete dev fm /dev, rmvd fm Sysfs file\n");
		break;

	case BUS_NOTIFY_REMOVED_DEVICE:	/* device removed */
			trace_printk("bus notify usb:bus removed the device fm list event\n");
		break;
	case BUS_NOTIFY_BIND_DRIVER:		/* driver about to be bound */
			trace_printk("bus notify usb to bind intf event\n");
		break;
	case BUS_NOTIFY_BOUND_DRIVER:	    /* driver bound to device */
			trace_printk("bus notify usb to bound intf event\n");
		break;
	case BUS_NOTIFY_UNBIND_DRIVER:	/* driver about to be unbound */
			trace_printk("bus notify usb to unbind intf event\n");
		break;
	case BUS_NOTIFY_UNBOUND_DRIVER:	/* driver is unbound */
			trace_printk("bus notify usb to unbound intf event\n");
		break;

	}


	jprobe_return();
	return 0;
}

/*** devio.c ***/
static int usbdev_open_probe(struct inode *inode, struct file *file)
{
	trace_printk("devio:usbdev_open, File OPEN\n");
	urbct = 0;
	jprobe_return();
	return 0;
}


static long usbdev_do_ioctl_probe(struct file *file,
		unsigned int cmd, void __user *p)
{
	struct usb_dev_state *ps = file->private_data;
	struct usb_device *dev = ps->dev;


	switch (cmd) {
	case USBDEVFS_CONTROL:
		trace_printk("devio:usbdev_do_ioctl:devNum=%d, USBDEVFS_CONTROL\n", (int)(dev->devnum));
		break;

	case USBDEVFS_BULK:
		trace_printk("devio:usbdev_do_ioctl: devNum=%d, USBDEVFS_BULK\n", (int)(dev->devnum));
		break;

	case USBDEVFS_RESETEP:
		trace_printk("devio:usbdev_do_ioctl: devNum=%d, USBDEVFS_RESETEP\n", (int)(dev->devnum));
		break;

	case USBDEVFS_RESET:
		trace_printk("devio:usbdev_do_ioctl: devNum=%d, USBDEVFS_RESET\n", (int)(dev->devnum));
		break;

	case USBDEVFS_CLEAR_HALT:
		trace_printk("devio:usbdev_do_ioctl: devNum=%d, USBDEVFS_CLEAR_HALT\n", (int)(dev->devnum));
		break;

	case USBDEVFS_GETDRIVER:
		trace_printk("devio:usbdev_do_ioctl: devNum=%d, USBDEVFS_GETDRIVER\n", (int)(dev->devnum));
		break;

	case USBDEVFS_CONNECTINFO:
		trace_printk("devio:usbdev_do_ioctl: devNum=%d, USBDEVFS_CONNECTINFO\n", (int)(dev->devnum));
		break;

	case USBDEVFS_SETINTERFACE:
		trace_printk("devio:usbdev_do_ioctl: devNum=%d, USBDEVFS_SETINTERFACE\n", (int)(dev->devnum));
		break;

	case USBDEVFS_SETCONFIGURATION:
		trace_printk("devio:usbdev_do_ioctl: devNum=%d, USBDEVFS_SETCONFIGURATION\n", (int)(dev->devnum));
		break;


	case USBDEVFS_SUBMITURB:
		urbct++;
		if ((urbct == 1) || !(urbct % 500)) {
			trace_printk("devio:usbdev_do_ioctl: devNum=%d, USBDEVFS_SUBMITURB, urbct=%d\n",
			(int)(dev->devnum), urbct);
		}
		break;


	case USBDEVFS_DISCARDURB:
		trace_printk("devio:usbdev_do_ioctl: devNum=%d, USBDEVFS_DISCARDURB\n", (int)(dev->devnum));
		break;

	case USBDEVFS_DISCSIGNAL:
		trace_printk("devio:usbdev_do_ioctl: devNum=%d, USBDEVFS_DISCSIGNAL\n", (int)(dev->devnum));
		break;

	case USBDEVFS_CLAIMINTERFACE:
		trace_printk("devio:usbdev_do_ioctl: devNum=%d, USBDEVFS_CLAIMINTERFACE\n", (int)(dev->devnum));
		break;

	case USBDEVFS_RELEASEINTERFACE:
		trace_printk("devio:usbdev_do_ioctl: devNum=%d, USBDEVFS_RELEASEINTERFACE\n", (int)(dev->devnum));
		break;

	case USBDEVFS_IOCTL:
		trace_printk("devio:usbdev_do_ioctl: devNum=%d, USBDEVFS_IOCTL\n", (int)(dev->devnum));
		break;

	case USBDEVFS_CLAIM_PORT:
		trace_printk("devio:usbdev_do_ioctl: devNum=%d, USBDEVFS_CLAIM_PORT\n", (int)(dev->devnum));
		break;

	case USBDEVFS_RELEASE_PORT:
		trace_printk("devio:usbdev_do_ioctl: devNum=%d, USBDEVFS_RELEASE_PORT\n", (int)(dev->devnum));
		break;
	case USBDEVFS_GET_CAPABILITIES:
		trace_printk("devio:usbdev_do_ioctl: devNum=%d, USBDEVFS_GET_CAPABILITIES\n", (int)(dev->devnum));
		break;

	case USBDEVFS_DISCONNECT_CLAIM:
		trace_printk("devio:usbdev_do_ioctl: devNum=%d, USBDEVFS_DISCONNECT_CLAIM\n", (int)(dev->devnum));
		break;

	case USBDEVFS_ALLOC_STREAMS:
		trace_printk("devio:usbdev_do_ioctl: devNum=%d, USBDEVFS_ALLOC_STREAMS\n", (int)(dev->devnum));
		break;

	case USBDEVFS_FREE_STREAMS:
		trace_printk("devio:usbdev_do_ioctl: devNum=%d, USBDEVFS_FREE_STREAMS\n", (int)(dev->devnum));
		break;
	 }

	jprobe_return();
	return 0;
}


static int proc_do_submiturb_probe(struct usb_dev_state *ps,
			struct usbdevfs_urb *uurb,
			struct usbdevfs_iso_packet_desc __user *iso_frame_desc,
			void __user *arg)
{
	if ((urbct == 1) || !(urbct % 500)) {
		trace_printk("devio:proc_do_submiturb,uurbType=%d,urbBufsize=%d, streamId=%d\n",
			uurb->type, uurb->buffer_length, uurb->stream_id);
	}

	jprobe_return();
	return 0;
}

static int usbdev_release_probe(struct inode *inode, struct file *file)
{
	struct usb_dev_state *ps = file->private_data;
	struct usb_device *dev = ps->dev;

	trace_printk("devio:usbdev_release,File Release,urbCount=%d,devNum=%d\n",
		urbct, dev->devnum);

	jprobe_return();
	return 0;
}
static void usbdev_remove_probe(struct usb_device *udev)
{
	trace_printk("devio:usbdev_remove, devNum=%d\n", udev->devnum);
	jprobe_return();
}


/*** HUB ***/
static struct jprobe hub_port_init_jp = {
	.kp.symbol_name = "hub_port_init",
	.entry = (kprobe_opcode_t *) hub_port_init_probe,
};


static struct jprobe usb_get_device_descriptor_jp = {
	.kp.symbol_name = "usb_get_device_descriptor",
	.entry = (kprobe_opcode_t *) usb_get_device_descriptor_probe,
};

static struct jprobe usb_new_device_jp = {
	.kp.symbol_name = "usb_new_device",
	.entry = (kprobe_opcode_t *) usb_new_device_probe,
};

static struct jprobe usb_disconnect_jp = {
	.kp.symbol_name = "usb_disconnect",
	.entry = (kprobe_opcode_t *) usb_disconnect_probe,
};


/*** driver ***/
static struct jprobe usb_probe_device_jp = {
	.kp.symbol_name = "usb_probe_device",
	.entry = (kprobe_opcode_t *) usb_probe_device_probe,
};

static struct jprobe usb_unbind_interface_jp = {
	.kp.symbol_name = "usb_unbind_interface",
	.entry = (kprobe_opcode_t *) usb_unbind_interface_probe,
};

static struct jprobe usb_driver_claim_interface_jp = {
	.kp.symbol_name = "usb_driver_claim_interface",
	.entry = (kprobe_opcode_t *) usb_driver_claim_interface_probe,
};

static struct jprobe usb_probe_interface_jp = {
	.kp.symbol_name = "usb_probe_interface",
	.entry = (kprobe_opcode_t *) usb_probe_interface_probe,
};

/*** Message ***/
static struct jprobe usb_set_configuration_jp = {
	.kp.symbol_name = "usb_set_configuration",
	.entry = (kprobe_opcode_t *) usb_set_configuration_probe,
};

static struct jprobe create_intf_ep_devs_jp = {
	.kp.symbol_name = "create_intf_ep_devs",
	.entry = (kprobe_opcode_t *) create_intf_ep_devs_probe,
};
/*** Endpoing ***/
static struct jprobe usb_create_ep_devs_jp = {
	.kp.symbol_name = "usb_create_ep_devs",
	.entry = (kprobe_opcode_t *) usb_create_ep_devs_probe,
};


/*** USB ***/

static struct jprobe usb_bus_notify_jp = {
	.kp.symbol_name = "usb_bus_notify",
	.entry = (kprobe_opcode_t *) usb_bus_notify_probe,
};


/*** devio ***/
static struct jprobe usbdev_open_jp = {
	.kp.symbol_name = "usbdev_open",
	.entry = (kprobe_opcode_t *) usbdev_open_probe,
};

static struct jprobe usbdev_do_ioctl_jp = {
	.kp.symbol_name = "usbdev_do_ioctl",
	.entry = (kprobe_opcode_t *) usbdev_do_ioctl_probe,
};

static struct jprobe usbdev_release_jp = {
	.kp.symbol_name = "usbdev_release",
	.entry = (kprobe_opcode_t *) usbdev_release_probe,
};

static struct jprobe usbdev_remove_jp = {
	.kp.symbol_name = "usbdev_remove",
	.entry = (kprobe_opcode_t *) usbdev_remove_probe,
};
/*
*static struct jprobe proc_do_submiturb_jp = {
*	.kp.symbol_name = "proc_do_submiturb",
*	.entry = (kprobe_opcode_t *) proc_do_submiturb_probe,
* };
*/

/****************************
*  PROFILER_INIT
****************************/

static int __init profiler_init(void)
{


	/***  jprobe registration ***/
	reg_err = register_jprobe(&usb_probe_interface_jp);
	if (reg_err) {
		trace_printk("ML: usb_probe_interface_jp failed = %d", reg_err);
		return reg_err;
	}
	trace_printk("plant jprobe at %p, handler addr %p\n",
		usb_probe_interface_jp.kp.addr, usb_probe_interface_jp.entry);


	reg_err = register_jprobe(&usb_bus_notify_jp);
	if (reg_err) {
		trace_printk("ML: usb_probe_interface_jp failed = %d", reg_err);
		return reg_err;
	}
	trace_printk("plant jprobe at %p, handler addr %p\n",
		usb_bus_notify_jp.kp.addr, usb_bus_notify_jp.entry);

	reg_err = register_jprobe(&usb_disconnect_jp);
	if (reg_err) {
		trace_printk("ML: usb_disconnect_jp failed = %d", reg_err);
		return reg_err;
	}
	trace_printk("plant jprobe at %p, handler addr %p\n",
			usb_disconnect_jp.kp.addr, usb_disconnect_jp.entry);

	reg_err = register_jprobe(&usb_unbind_interface_jp);
	if (reg_err) {
		trace_printk("ML: usb_unbind_interface_jp failed = %d", reg_err);
		return reg_err;
	}
	trace_printk("plant jprobe at %p, handler addr %p\n",
		usb_unbind_interface_jp.kp.addr, usb_unbind_interface_jp.entry);

	reg_err = register_jprobe(&hub_port_init_jp);
	if (reg_err) {
		trace_printk("ML: hub_port_init_jp failed = %d", reg_err);
		return reg_err;
	}
	trace_printk("plant jprobe at %p, handler addr %p\n",
		hub_port_init_jp.kp.addr, hub_port_init_jp.entry);

	reg_err = register_jprobe(&usb_get_device_descriptor_jp);
	if (reg_err) {
		trace_printk("ML: usb_get_device_descriptor_jp failed = %d",
			reg_err);
		return reg_err;
	}
	trace_printk("plant jprobe at %p, handler addr %p\n",
		usb_get_device_descriptor_jp.kp.addr,
		usb_get_device_descriptor_jp.entry);

	reg_err = register_jprobe(&usb_new_device_jp);
	if (reg_err) {
		trace_printk("ML: usb_new_device_jp failed = %d", reg_err);
		return reg_err;
	}
	trace_printk("plant jprobe at %p, handler addr %p\n",
		usb_new_device_jp.kp.addr, usb_new_device_jp.entry);

	reg_err = register_jprobe(&usb_probe_device_jp);
	if (reg_err) {
		trace_printk("ML: usb_probe_device_jp failed = %d", reg_err);
		return reg_err;
	}
	trace_printk("plant jprobe at %p, handler addr %p\n",
		usb_probe_device_jp.kp.addr, usb_probe_device_jp.entry);

	reg_err = register_jprobe(&usb_set_configuration_jp);
	if (reg_err) {
		trace_printk("ML: usb_set_configuration_jp failed = %d", reg_err);
		return reg_err;
	}
	trace_printk("plant jprobe at %p, handler addr %p\n",
		usb_set_configuration_jp.kp.addr,
		usb_set_configuration_jp.entry);


	reg_err = register_jprobe(&usbdev_open_jp);
	if (reg_err) {
		trace_printk("ML: usbdev_open_jp failed = %d", reg_err);
		return reg_err;
	}
	trace_printk("plant jprobe at %p, handler addr %p\n",
		usbdev_open_jp.kp.addr, usbdev_open_jp.entry);


	reg_err = register_jprobe(&usbdev_do_ioctl_jp);
	if (reg_err) {
		trace_printk("ML: usbdev_do_ioctl_jp failed = %d", reg_err);
		return reg_err;
	}
	trace_printk("plant jprobe at %p, handler addr %p\n",
		usbdev_do_ioctl_jp.kp.addr, usbdev_do_ioctl_jp.entry);

	reg_err = register_jprobe(&usbdev_release_jp);
	if (reg_err) {
		trace_printk("ML: usbdev_release_jp failed = %d", reg_err);
		return reg_err;
	}
	trace_printk("plant jprobe at %p, handler addr %p\n",
		usbdev_release_jp.kp.addr, usbdev_release_jp.entry);

	reg_err = register_jprobe(&usb_driver_claim_interface_jp);
	if (reg_err) {
		trace_printk("ML: usb_driver_claim_interface_jp failed = %d",
			reg_err);
		return reg_err;
	}
	trace_printk("plant jprobe at %p, handler addr %p\n",
		usb_driver_claim_interface_jp.kp.addr,
		usb_driver_claim_interface_jp.entry);

	reg_err = register_jprobe(&usbdev_remove_jp);
	if (reg_err) {
		trace_printk("ML: usbdev_remove_jp failed = %d", reg_err);
		return reg_err;
	}
	trace_printk("plant jprobe at %p, handler addr %p\n",
		usbdev_remove_jp.kp.addr, usbdev_remove_jp.entry);

/*
*	reg_err = register_jprobe(&proc_do_submiturb_jp);
*	if(reg_err) {
*		trace_printk("ML: proc_do_submiturb_jp failed = %d",reg_err);
*		return reg_err;
*	}
*	trace_printk("plant jprobe at %p, handler addr %p\n",
*		proc_do_submiturb_jp.kp.addr, proc_do_submiturb_jp.entry);
*/


	reg_err = register_jprobe(&create_intf_ep_devs_jp);
	if (reg_err) {
		trace_printk("ML: create_intf_ep_devs_jp failed = %d", reg_err);
		return reg_err;
	}

	trace_printk("plant jprobe at %p, handler addr %p\n",
		create_intf_ep_devs_jp.kp.addr, create_intf_ep_devs_jp.entry);

	reg_err = register_jprobe(&usb_create_ep_devs_jp);
	if (reg_err) {
		trace_printk("ML: usb_create_ep_devs_jp failed = %d",
					reg_err);
		return reg_err;
	}
	trace_printk("plant jprobe at %p, handler addr %p\n",
		usb_create_ep_devs_jp.kp.addr, usb_create_ep_devs_jp.entry);


	return reg_err;
}

static void __exit profiler_exit(void)
{
	if (reg_err != 0) {
		trace_printk("Exit: reg reg_err=%d", reg_err);
		return;
	}
	unregister_jprobe(&usb_probe_interface_jp);
	unregister_jprobe(&usb_bus_notify_jp);
	unregister_jprobe(&usb_disconnect_jp);
	unregister_jprobe(&usb_unbind_interface_jp);
	unregister_jprobe(&hub_port_init_jp);
	unregister_jprobe(&usb_get_device_descriptor_jp);
	unregister_jprobe(&usb_new_device_jp);
	unregister_jprobe(&usb_probe_device_jp);
	unregister_jprobe(&usb_set_configuration_jp);
	unregister_jprobe(&usbdev_open_jp);
	unregister_jprobe(&usbdev_do_ioctl_jp);
	unregister_jprobe(&usbdev_release_jp);
	unregister_jprobe(&usb_driver_claim_interface_jp);
	unregister_jprobe(&usbdev_remove_jp);
	unregister_jprobe(&create_intf_ep_devs_jp);
	unregister_jprobe(&usb_create_ep_devs_jp);
	/*unregister_jprobe(&proc_do_submiturb_jp);*/
}

module_init(profiler_init);
module_exit(profiler_exit);
MODULE_AUTHOR("Magic Leap LLC");
MODULE_DESCRIPTION("Usb debugger/profiler");
MODULE_LICENSE("GPL v2");
