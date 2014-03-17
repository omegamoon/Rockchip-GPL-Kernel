/*
 *  Copyright (c) 2013 Realtek Semiconductor Corp. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 *  This product is covered by one or more of the following patents:
 *  US6,570,884, US6,115,776, and US6,327,625.
 */

#include <linux/init.h>
#include <linux/signal.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/version.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/mii.h>
#include <linux/ethtool.h>
#include <linux/usb.h>
#include <linux/crc32.h>
#include <asm/uaccess.h>
#include "compatibility.h"
#include <mach/gpio.h>

/* Version Information */
#define DRIVER_VERSION "v1.0.0 (2013/04/18)"
#define DRIVER_AUTHOR "Albert Kuo <albertk@realtek.com>"
#define DRIVER_DESC "rtl8152 based usb-ethernet driver"
#define MODULENAME "r8152"


#define	IDR			0xc000
#define	MAR			0xcd00
#define RMS			0xc014
#define CFG9346			0xe81c
#define RCR			0xc010
#define PHYAR			0xde00
#define PHYstatus		0xe908

enum rtl_register_content {
	_100bps		= 0x08,
	_10bps		= 0x04,
	LinkStatus	= 0x02,
	FullDup		= 0x01,
};

#define	INTBUFSIZE		10

#define	RTL8152_REQT_READ	0xc0
#define	RTL8152_REQT_WRITE	0x40
#define	RTL8152_REQ_GET_REGS	0x05
#define	RTL8152_REQ_SET_REGS	0x05


#define	RTL8152_MTU		1536
#define	RTL8152_TX_TIMEOUT	(HZ)

/* rtl8152 flags */
enum rtl8152_flags {
	RTL8152_UNPLUG = 0,
	RX_URB_FAIL,
	RTL8152_SET_RX_MODE
};

/* Define these values to match your device */
#define	VENDOR_ID_REALTEK		0x0bda
#define	VENDOR_ID_MELCO			0x0411
#define	VENDOR_ID_MICRONET		0x3980
#define	VENDOR_ID_LONGSHINE		0x07b8
#define	VENDOR_ID_OQO			0x1557
#define	VENDOR_ID_ZYXEL			0x0586

#define PRODUCT_ID_RTL8152		0x8152
#define	PRODUCT_ID_LUAKTX		0x0012
#define	PRODUCT_ID_LCS8138TX		0x401a
#define PRODUCT_ID_SP128AR		0x0003
#define	PRODUCT_ID_PRESTIGE		0x401a

#define MCU_TYPE_PLA			0x0100
#define MCU_TYPE_USB			0x0000

typedef struct _RxDesc
{
	u32 opts1;
	u32 opts2;
	u32 opts3;
	u32 opts4;
	u32 opts5;
	u32 opts6;
}RxDesc;

enum rtl_desc_bit {
	FirstFrag	= (1 << 31), /* First segment of a packet */
	LastFrag	= (1 << 30), /* Final segment of a packet */
};

typedef struct _TxDesc
{
	u32 opts1;
	u32 opts2;
}TxDesc;

struct rtl8152 {
	unsigned long flags;
	struct usb_device *udev;
	struct tasklet_struct tl;
	struct net_device *netdev;
	struct urb *rx_urb, *tx_urb, *intr_urb;
	struct sk_buff *tx_skb, *rx_skb;
	struct delayed_work schedule;
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,22)
	struct net_device_stats stats;
#endif
	u32 msg_enable;
	int intr_interval;
	u8 *intr_buff;
	u8 version;
	u8 speed;
};

typedef struct rtl8152 rtl8152_t;

enum rtl_cmd {
	RTLTOOL_PLA_OCP_READ_DWORD=0,
	RTLTOOL_PLA_OCP_WRITE_DWORD,
	RTLTOOL_USB_OCP_READ_DWORD,
	RTLTOOL_USB_OCP_WRITE_DWORD,
	RTLTOOL_PLA_OCP_READ,
	RTLTOOL_PLA_OCP_WRITE,
	RTLTOOL_USB_OCP_READ,
	RTLTOOL_USB_OCP_WRITE,
	RTLTOOL_USB_INFO,

	RTLTOOL_INVALID
};

struct usb_device_info {
	__u16		idVendor;
	__u16		idProduct;
	__u16		bcdDevice;
	__u8		dev_addr[8];
	char		devpath[16];
};

struct rtltool_cmd {
	__u32	cmd;
	__u32	offset;
	__u32	byteen;
	__u32	data;
	void	*buf;
	struct usb_device_info nic_info;
	struct sockaddr ifru_addr;
	struct sockaddr ifru_netmask;
	struct sockaddr ifru_hwaddr;
};

enum rtl_version {
	RTL_VER_UNKNOWN = 0,
	RTL_VER_01,
	RTL_VER_02
};

static int rtl8152_set_speed(rtl8152_t *tp, u8 autoneg, u16 speed, u8 duplex);
/*
**
**	device related part of the code
**
*/
static
int get_registers(rtl8152_t *tp, u16 value, u16 index, u16 size, void *data)
{
	return usb_control_msg(tp->udev, usb_rcvctrlpipe(tp->udev, 0),
			       RTL8152_REQ_GET_REGS, RTL8152_REQT_READ,
			       value, index, data, size, 500);
}

static
int set_registers(rtl8152_t *tp, u16 value, u16 index, u16 size, void *data)
{
	return usb_control_msg(tp->udev, usb_sndctrlpipe(tp->udev, 0),
			       RTL8152_REQ_SET_REGS, RTL8152_REQT_WRITE,
			       value, index, data, size, 500);
}

static int pla_ocp_read(rtl8152_t *tp, u16 index, u16 size, void *data)
{
	u16	limit = 64;
	int	ret = 0;

	if (test_bit(RTL8152_UNPLUG, &tp->flags))
		return -ENODEV;

	if ((size & 3) || !size || (index & 3) || !data)
		return -EPERM;

	if ((u32)index + (u32)size > 0xffff)
		return -EPERM;

	while (size) {
		if (size > limit) {
			ret = get_registers(tp, index, MCU_TYPE_PLA,
				limit, data);
			if (ret < 0)
				break;

			index += limit;
			data += limit;
			size -= limit;
		} else {
			ret = get_registers(tp, index, MCU_TYPE_PLA,
				size, data);
			if (ret < 0)
				break;

			index += size;
			data += size;
			size = 0;
			break;
		}
	}

	return ret;
}

static
int pla_ocp_write(rtl8152_t *tp, u16 index, u16 byteen, u16 size, void *data)
{
	int	ret;
	u16	byteen_start, byteen_end, byen;
	u16	limit = 512;

	if (test_bit(RTL8152_UNPLUG, &tp->flags))
		return -ENODEV;

	if ((size & 3) || !size || (index & 3) || !data)
		return -EPERM;

	if ((u32)index + (u32)size > 0xffff)
		return -EPERM;

	byteen_start = byteen & 0xf;
	byteen_end = byteen & 0xf0;

	byen = byteen_start | (byteen_start << 4);
	ret = set_registers(tp, index, MCU_TYPE_PLA | byen, 4, data);
	if (ret < 0)
		goto error1;

	index += 4;
	data += 4;
	size -= 4;

	if (size) {
		size -= 4;

		while (size) {
			if (size > limit) {
				ret = set_registers(tp, index,
					MCU_TYPE_PLA | 0xff, limit, data);
				if (ret < 0)
					goto error1;

				index += limit;
				data += limit;
				size -= limit;
			} else {
				ret = set_registers(tp, index,
					MCU_TYPE_PLA | 0xff, size, data);
				if (ret < 0)
					goto error1;

				index += size;
				data += size;
				size = 0;
				break;
			}
		}

		byen = byteen_end | (byteen_end >> 4);
		ret = set_registers(tp, index, MCU_TYPE_PLA | byen, 4, data);
		if (ret < 0)
			goto error1;
	}

error1:
	return ret;
}

static int usb_ocp_read(rtl8152_t *tp, u16 index, u16 size, void *data)
{
	u16	limit = 64;
	int	ret = 0;

	if (test_bit(RTL8152_UNPLUG, &tp->flags))
		return -ENODEV;

	if ((size & 3) || !size || (index & 3))
		return -EPERM;

	if ((u32)index + (u32)size > 0xffff)
		return -EPERM;

	while (size) {
		if (size > limit) {
			ret = get_registers(tp, index, MCU_TYPE_USB,
				limit, data);
			if (ret < 0)
				break;

			index += limit;
			data += limit;
			size -= limit;
		} else {
			ret = get_registers(tp, index, MCU_TYPE_USB,
				size, data);
			if (ret < 0)
				break;

			index += size;
			data += size;
			size = 0;
			break;
		}
	}

	return ret;
}

static
int usb_ocp_write(rtl8152_t *tp, u16 index, u16 byteen, u16 size, void *data)
{
	int	ret;
	u16	byteen_start, byteen_end, byen;
	u16	limit = 512;

	if (test_bit(RTL8152_UNPLUG, &tp->flags))
		return -ENODEV;

	if ((size & 3) || !size || (index & 3))
		return -EPERM;

	if ((u32)index + (u32)size > 0xffff)
		return -EPERM;

	byteen_start = byteen & 0xf;
	byteen_end = byteen & 0xf0;

	byen = byteen_start | (byteen_start << 4);
	ret = set_registers(tp, index, MCU_TYPE_USB | byen, 4, data);
	if (ret < 0)
		goto error1;

	index += 4;
	data += 4;
	size -= 4;

	if (size) {
		size -= 4;

		while (size) {
			if (size > limit) {
				ret = set_registers(tp, index,
					MCU_TYPE_USB | 0xff, limit, data);
				if (ret < 0)
					goto error1;

				index += limit;
				data += limit;
				size -= limit;
			} else {
				ret = set_registers(tp, index,
					MCU_TYPE_USB | 0xff, size, data);
				if (ret < 0)
					goto error1;

				index += size;
				data += size;
				size = 0;
				break;
			}
		}

		byen = byteen_end | (byteen_end >> 4);
		ret = set_registers(tp, index, MCU_TYPE_USB | byen, 4, data);
		if (ret < 0)
			goto error1;
	}

error1:
	return ret;
}

static void write_mii_word(rtl8152_t *tp, u32 reg_addr, u32 value)
{
	__le32	ocp_data;
	int	i;

	ocp_data = __cpu_to_le32(0x80000000 |
				 ((reg_addr & 0x1f) << 16) |
				 (value & 0xffff));
	pla_ocp_write(tp, PHYAR, 0xff, sizeof(ocp_data), &ocp_data);

	for (i = 20; i > 0; i--) {
		udelay(25);
		pla_ocp_read(tp, PHYAR, sizeof(ocp_data), &ocp_data);
		if (!(ocp_data & __cpu_to_le32(0x80000000)))
			break;
	}
	udelay(20);
}

static u16 read_mii_word(rtl8152_t *tp, u32 reg_addr)
{
	__le32	ocp_data;
	int	i;

	ocp_data = __cpu_to_le32((reg_addr & 0x1f) << 16);
	pla_ocp_write(tp, PHYAR, 0xff, sizeof(ocp_data), &ocp_data);

	for (i = 20; i > 0; i--) {
		udelay(25);
		pla_ocp_read(tp, PHYAR, sizeof(ocp_data), &ocp_data);
		if (ocp_data & __cpu_to_le32(0x80000000))
			break;
	}
	udelay(20);

	return (u16)__le32_to_cpu(ocp_data);
}

static inline void set_ethernet_addr(rtl8152_t *tp)
{
	struct net_device *dev = tp->netdev;
	u8 node_id[8] = {0};// = {0x00, 0xe0, 0x4c, 0x68, 0x00, 0x04, 0x00, 0x00};

	if (pla_ocp_read(tp, IDR, sizeof(node_id), node_id) < 0)
		dev_printk(KERN_NOTICE, &tp->udev->dev, "inet addr fail\n");
	else {
		__le32	ocp_data;

		if (tp->version == RTL_VER_01) {
			usb_ocp_read(tp, 0xd428, sizeof(ocp_data), &ocp_data);
			ocp_data |= __cpu_to_le32(0x8000);
			usb_ocp_write(tp, 0xd428, 0x22, sizeof(ocp_data), &ocp_data);
			usb_ocp_write(tp, 0xc2f0, 0x3f, sizeof(node_id), node_id);
			ocp_data &= __cpu_to_le32(~0x8000);
			usb_ocp_write(tp, 0xd428, 0x22, sizeof(ocp_data), &ocp_data);
		}

		memcpy(dev->dev_addr, node_id, sizeof(node_id));
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,13)
		memcpy(dev->perm_addr, dev->dev_addr, dev->addr_len);
#endif
	}
}

static int rtl8152_set_mac_address(struct net_device *netdev, void *p)
{
	rtl8152_t *tp = netdev_priv(netdev);
	struct sockaddr *addr = p;
	__le32	ocp_data;

	if (!is_valid_ether_addr(addr->sa_data))
		return -EADDRNOTAVAIL;

	memcpy(netdev->dev_addr, addr->sa_data, netdev->addr_len);

	ocp_data = __cpu_to_le32(0xc0);
	pla_ocp_write(tp, CFG9346, 0x11, sizeof(ocp_data), &ocp_data);
	pla_ocp_write(tp, IDR, 0x3f, 8, addr->sa_data);
	ocp_data = __cpu_to_le32(0x00);
	return pla_ocp_write(tp, CFG9346, 0x11, sizeof(ocp_data), &ocp_data);
}

static int alloc_all_urbs(rtl8152_t *tp)
{
	tp->rx_urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!tp->rx_urb)
		return 0;
	tp->tx_urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!tp->tx_urb) {
		usb_free_urb(tp->rx_urb);
		return 0;
	}
	tp->intr_urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!tp->intr_urb) {
		usb_free_urb(tp->rx_urb);
		usb_free_urb(tp->tx_urb);
		return 0;
	}

	return 1;
}

static void free_all_urbs(rtl8152_t *tp)
{
	usb_free_urb(tp->rx_urb);
	usb_free_urb(tp->tx_urb);
	usb_free_urb(tp->intr_urb);
}

static struct net_device_stats *rtl8152_get_stats(struct net_device *dev)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,22)
	struct rtl8152 *tp = netdev_priv(dev);

	return (struct net_device_stats *)&tp->stats;
#else
	return &dev->stats;
#endif
}

static void read_bulk_callback(struct urb *urb)
{
	rtl8152_t *tp;
	unsigned pkt_len;
	struct sk_buff *skb;
	struct net_device *netdev;
	struct net_device_stats *stats;
	int status = urb->status;
	int result;
	RxDesc *rx_desc;

	tp = urb->context;
	if (!tp)
		return;
	if (test_bit(RTL8152_UNPLUG, &tp->flags))
		return;
	netdev = tp->netdev;
	if (!netif_device_present(netdev))
		return;

	stats = rtl8152_get_stats(netdev);
	switch (status) {
	case 0:
		break;
	case -ESHUTDOWN:
		set_bit(RTL8152_UNPLUG, &tp->flags);
		netif_device_detach(tp->netdev);
	case -ENOENT:
		return;	/* the urb is in unlink state */
	case -ETIME:
		if (printk_ratelimit())
			dev_warn(&urb->dev->dev, "may be reset is needed?..\n");
		goto goon;
	default:
		if (printk_ratelimit())
			dev_warn(&urb->dev->dev, "Rx status %d\n", status);
		goto goon;
	}

	/* protect against short packets (tell me why we got some?!?) */
	if (urb->actual_length < sizeof(*rx_desc)) {
		goto goon;
	}

	rx_desc = (RxDesc *)urb->transfer_buffer;
	pkt_len = le32_to_cpu(rx_desc->opts1) & 0x7fff;
	if (urb->actual_length < sizeof(RxDesc) + pkt_len) {
		goto goon;
	}

	skb = netdev_alloc_skb_ip_align(netdev, pkt_len);
	if (!skb) {
		goto goon;
	}
	memcpy(skb->data, tp->rx_skb->data + sizeof(RxDesc), pkt_len);
	skb_put(skb, pkt_len);
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,22)
	skb->dev = netdev;
#endif
	skb->protocol = eth_type_trans(skb, netdev);
	netif_rx(skb);
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,29)
	netdev->last_rx = jiffies;
#endif
	stats->rx_packets++;
	stats->rx_bytes += pkt_len;
goon:
	usb_fill_bulk_urb(tp->rx_urb, tp->udev, usb_rcvbulkpipe(tp->udev, 1),
		      tp->rx_skb->data, RTL8152_MTU + sizeof(RxDesc),
		      (usb_complete_t)read_bulk_callback, tp);
	result = usb_submit_urb(tp->rx_urb, GFP_ATOMIC);
	if (result == -ENODEV) {
		netif_device_detach(tp->netdev);
	} else if (result) {
		set_bit(RX_URB_FAIL, &tp->flags);
		goto resched;
	} else {
		clear_bit(RX_URB_FAIL, &tp->flags);
	}

	return;
resched:
	tasklet_schedule(&tp->tl);
}

static void rx_fixup(unsigned long data)
{
	rtl8152_t *tp;
	int status;

	tp = (rtl8152_t *)data;

//	if (test_bit(RX_URB_FAIL, &tp->flags))
//		if (tp->rx_skb)
//			goto try_again;
//	usb_fill_bulk_urb(tp->rx_urb, tp->udev, usb_rcvbulkpipe(tp->udev, 1),
//		      tp->rx_skb->data, RTL8152_MTU + sizeof(RxDesc),
//		      (usb_complete_t)read_bulk_callback, tp);
//try_again:
	status = usb_submit_urb(tp->rx_urb, GFP_ATOMIC);
	if (status == -ENODEV) {
		netif_device_detach(tp->netdev);
	} else if (status) {
		set_bit(RX_URB_FAIL, &tp->flags);
		goto tlsched;
	} else {
		clear_bit(RX_URB_FAIL, &tp->flags);
	}

	return;
tlsched:
	tasklet_schedule(&tp->tl);
}

static void write_bulk_callback(struct urb *urb)
{
	rtl8152_t *tp;
	int status = urb->status;

	tp = urb->context;
	if (!tp)
		return;
	dev_kfree_skb_irq(tp->tx_skb);
	if (!netif_device_present(tp->netdev))
		return;
	if (status)
		dev_info(&urb->dev->dev, "%s: Tx status %d\n",
			 tp->netdev->name, status);
	tp->netdev->trans_start = jiffies;
	netif_wake_queue(tp->netdev);
}

#if 0
static void intr_callback(struct urb *urb)
{
	rtl8152_t *tp;
	struct net_device_stats *stats;
	__u8 *d;
	int status = urb->status;
	int res;

	tp = urb->context;
	if (!tp)
		return;
	switch (status) {
	case 0:			/* success */
		break;
	case -ECONNRESET:	/* unlink */
	case -ESHUTDOWN:
		netif_device_detach(tp->netdev);
	case -ENOENT:
		return;
	case -EOVERFLOW:
		dev_info(&urb->dev->dev, "%s: intr status -EOVERFLOW\n",
			 tp->netdev->name);
		goto resubmit;
	/* -EPIPE:  should clear the halt */
	default:
		dev_info(&urb->dev->dev, "%s: intr status %d\n",
			 tp->netdev->name, status);
		goto resubmit;
	}

	stats = rtl8152_get_stats(tp->netdev);
	d = urb->transfer_buffer;
	if (d[0] & TSR_ERRORS) {
		stats->tx_errors++;
		if (d[INT_TSR] & (TSR_ECOL | TSR_JBR))
			stats->tx_aborted_errors++;
		if (d[INT_TSR] & TSR_LCOL)
			stats->tx_window_errors++;
		if (d[INT_TSR] & TSR_LOSS_CRS)
			stats->tx_carrier_errors++;
	}
	/* Report link status changes to the network stack */
	if ((d[INT_MSR] & MSR_LINK) == 0) {
		if (netif_carrier_ok(tp->netdev)) {
			netif_carrier_off(tp->netdev);
			dbg("%s: LINK LOST\n", __func__);
		}
	} else {
		if (!netif_carrier_ok(tp->netdev)) {
			netif_carrier_on(tp->netdev);
			dbg("%s: LINK CAME BACK\n", __func__);
		}
	}

resubmit:
	res = usb_submit_urb (urb, GFP_ATOMIC);
	if (res == -ENODEV) {
		netif_device_detach(tp->netdev);
	} else if (res)
		printk ("can't resubmit intr, %s-%s/input0, status %d",
				tp->udev->bus->bus_name,
				tp->udev->devpath, res);
}
#endif

/*
**
**	network related part of the code
**
*/

static void rtl8152_tx_timeout(struct net_device *netdev)
{
	rtl8152_t *tp = netdev_priv(netdev);
	struct net_device_stats *stats = rtl8152_get_stats(netdev);
	netif_warn(tp, tx_err, netdev, "Tx timeout.\n");
	usb_unlink_urb(tp->tx_urb);
	stats->tx_errors++;
}

static void rtl8152_set_rx_mode(struct net_device *netdev)
{
	rtl8152_t *tp = netdev_priv(netdev);
	u32 tmp, mc_filter[2];	/* Multicast hash filter */
	__le32	ocp_data;

	if (in_atomic()) {
		if (tp->speed & LinkStatus)
			set_bit(RTL8152_SET_RX_MODE, &tp->flags);
		return;
	}

	clear_bit(RTL8152_SET_RX_MODE, &tp->flags);
	netif_stop_queue(netdev);
	pla_ocp_read(tp, RCR, sizeof(ocp_data), &ocp_data);
	ocp_data &= __cpu_to_le32(~0x0000000f);
	ocp_data |= __cpu_to_le32(0x0000000a);

	if (netdev->flags & IFF_PROMISC) {
		/* Unconditionally log net taps. */
		netif_notice(tp, link, netdev, "Promiscuous mode enabled\n");
		ocp_data |= __cpu_to_le32(0x05);
		mc_filter[1] = mc_filter[0] = 0xffffffff;
	} else if ((netdev_mc_count(netdev) > 32) ||
		   (netdev->flags & IFF_ALLMULTI)) {
		/* Too many to filter perfectly -- accept all multicasts. */
		ocp_data |= __cpu_to_le32(0x04);
		mc_filter[1] = mc_filter[0] = 0xffffffff;
	} else {
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35)
		struct dev_mc_list *mclist;
		unsigned int i;

		mc_filter[1] = mc_filter[0] = 0;
		for (i = 0, mclist = netdev->mc_list; mclist && i < netdev->mc_count;
		     i++, mclist = mclist->next) {
			int bit_nr;
			bit_nr = ether_crc(ETH_ALEN, mclist->dmi_addr) >> 26;
			mc_filter[bit_nr >> 5] |= 1 << (bit_nr & 31);
			ocp_data |= __cpu_to_le32(0x04);
		}
#else
		struct netdev_hw_addr *ha;

		mc_filter[1] = mc_filter[0] = 0;
		netdev_for_each_mc_addr(ha, netdev) {
			int bit_nr = ether_crc(ETH_ALEN, ha->addr) >> 26;
			mc_filter[bit_nr >> 5] |= 1 << (bit_nr & 31);
			ocp_data |= __cpu_to_le32(0x04);
		}
#endif
	}

	tmp = mc_filter[0];
	mc_filter[0] = __cpu_to_le32(swab32(mc_filter[1]));
	mc_filter[1] = __cpu_to_le32(swab32(tmp));

	pla_ocp_write(tp, MAR, 0xff, sizeof(mc_filter), mc_filter);
	pla_ocp_write(tp, RCR, 0x11, sizeof(ocp_data), &ocp_data);
	netif_wake_queue(netdev);
}

static netdev_tx_t rtl8152_start_xmit(struct sk_buff *skb,
					    struct net_device *netdev)
{
	rtl8152_t *tp = netdev_priv(netdev);
	struct net_device_stats *stats = rtl8152_get_stats(netdev);
	TxDesc *tx_desc;
	int len, res;

	netif_stop_queue(netdev);
//	len = (skb->len < 60) ? 60 : skb->len;
	len = skb->len;
	//printk("%s %d skb->data=%p\n",__FUNCTION__,__LINE__,skb->data); //modify by nition
	if (skb_header_cloned(skb) || skb_headroom(skb) < sizeof(*tx_desc) || ((unsigned long)skb->data & 3)) {
		struct sk_buff *tx_skb;

		tx_skb = skb_copy_expand(skb, sizeof(*tx_desc), 0, GFP_ATOMIC);
		dev_kfree_skb_any(skb);
		if (!tx_skb) {
			stats->tx_dropped++;
			netif_wake_queue(netdev);
			return NETDEV_TX_OK;
		}
		skb = tx_skb;
	//printk("%s %d skb->data=%p\n",__FUNCTION__,__LINE__,skb->data); //modify by nition
	}
	tx_desc = (TxDesc *)skb_push(skb, sizeof(*tx_desc));
	memset(tx_desc, 0, sizeof(*tx_desc));
	tx_desc->opts1 = cpu_to_le32((len & 0xffff) | FirstFrag | LastFrag);
	tp->tx_skb = skb;
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,31)
	netdev->trans_start = jiffies;
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,36)
	skb_tx_timestamp(skb);
#endif
	//printk("%s %d skb->data=%p\n",__FUNCTION__,__LINE__,skb->data);//modify by nition
	usb_fill_bulk_urb(tp->tx_urb, tp->udev, usb_sndbulkpipe(tp->udev, 2),
			  skb->data, skb->len,
			  (usb_complete_t)write_bulk_callback, tp);
	if ((res = usb_submit_urb(tp->tx_urb, GFP_ATOMIC))) {
		/* Can we get/handle EPIPE here? */
		if (res == -ENODEV) {
			netif_device_detach(tp->netdev);
		} else {
			netif_warn(tp, tx_err, netdev, "failed tx_urb %d\n", res);
			stats->tx_errors++;
			netif_start_queue(netdev);
		}
	} else {
		stats->tx_packets++;
		stats->tx_bytes += skb->len;
	}

	return NETDEV_TX_OK;
}

static void r8152b_reset_packet_filter(rtl8152_t *tp)
{
	__le32	ocp_data;

	pla_ocp_read(tp, 0xc0b4, sizeof(ocp_data), &ocp_data);
	ocp_data &= __cpu_to_le32(~0x00000001);
	pla_ocp_write(tp, 0xc0b4, 0x11, sizeof(ocp_data), &ocp_data);
	ocp_data |= __cpu_to_le32(0x00000001);
	pla_ocp_write(tp, 0xc0b4, 0x11, sizeof(ocp_data), &ocp_data);
}

static void rtl8152_nic_reset(rtl8152_t *tp)
{
	__le32	ocp_data;
	int	i;

	ocp_data = __cpu_to_le32(0x10000000);
	pla_ocp_write(tp, 0xe810, 0x88, sizeof(ocp_data), &ocp_data);

	for (i = 0; i < 1000; i++) {
		pla_ocp_read(tp, 0xe810, sizeof(ocp_data), &ocp_data);
		if (!(ocp_data & __cpu_to_le32(0x10000000)))
			break;
		udelay(100);
	}
}

static u8 rtl8152_get_speed(rtl8152_t *tp)
{
	__le32 ocp_data;

	pla_ocp_read(tp, PHYstatus, sizeof(ocp_data), &ocp_data);

	return (u8)__le32_to_cpu(ocp_data);
}

static int rtl8152_enable(rtl8152_t *tp)
{
	__le32	ocp_data;
	u8 speed;

	speed = rtl8152_get_speed(tp);
	if (speed & _100bps) {
		pla_ocp_read(tp, 0xe080, sizeof(ocp_data), &ocp_data);
		ocp_data &= __cpu_to_le32(~0x2);
		pla_ocp_write(tp, 0xe080, 0x11, sizeof(ocp_data), &ocp_data);
	} else {
		pla_ocp_read(tp, 0xe080, sizeof(ocp_data), &ocp_data);
		ocp_data |= __cpu_to_le32(0x2);
		pla_ocp_write(tp, 0xe080, 0x11, sizeof(ocp_data), &ocp_data);
	}

	r8152b_reset_packet_filter(tp);

	pla_ocp_read(tp, 0xe810, sizeof(ocp_data), &ocp_data);
	ocp_data |= __cpu_to_le32(0x0c000000);
	pla_ocp_write(tp, 0xe810, 0x88, sizeof(ocp_data), &ocp_data);

	pla_ocp_read(tp, 0xe858, sizeof(ocp_data), &ocp_data);
	ocp_data &= __cpu_to_le32(~0x00080000);
	pla_ocp_write(tp, 0xe858, 0x44, sizeof(ocp_data), &ocp_data);

	usb_fill_bulk_urb(tp->rx_urb, tp->udev, usb_rcvbulkpipe(tp->udev, 1),
		      tp->rx_skb->data, RTL8152_MTU + sizeof(RxDesc),
		      (usb_complete_t)read_bulk_callback, tp);

	return usb_submit_urb(tp->rx_urb, GFP_KERNEL);
}

static void rtl8152_disable(rtl8152_t *tp)
{
	__le32	ocp_data;
	int	i;

	usb_kill_urb(tp->intr_urb);

	pla_ocp_read(tp, RCR, sizeof(ocp_data), &ocp_data);
	ocp_data &= __cpu_to_le32(~0x000f);
	pla_ocp_write(tp, RCR, 0x11, sizeof(ocp_data), &ocp_data);

	usb_kill_urb(tp->tx_urb);

	pla_ocp_read(tp, 0xe858, sizeof(ocp_data), &ocp_data);
	ocp_data |= __cpu_to_le32(0x080000);
	pla_ocp_write(tp, 0xe858, 0x44, sizeof(ocp_data), &ocp_data);

	for (i = 0; i < 1000; i++) {
		pla_ocp_read(tp, 0xe84c, sizeof(ocp_data), &ocp_data);
		if ((ocp_data & __cpu_to_le32(0x30000000)) == __cpu_to_le32(0x30000000))
			break;
		mdelay(1);
	}

	for (i = 0; i < 1000; i++) {
		pla_ocp_read(tp, 0xe610, sizeof(ocp_data), &ocp_data);
		if (ocp_data & __cpu_to_le32(0x00000800))
			break;
		mdelay(1);
	}

	usb_kill_urb(tp->rx_urb);

	rtl8152_nic_reset(tp);
}

static void r8152b_exit_oob(rtl8152_t *tp)
{
	__le32	ocp_data;
	int	i;

	pla_ocp_read(tp, RCR, sizeof(ocp_data), &ocp_data);
	ocp_data &= __cpu_to_le32(~0x000f);
	pla_ocp_write(tp, RCR, 0x11, sizeof(ocp_data), &ocp_data);

	pla_ocp_read(tp, 0xe858, sizeof(ocp_data), &ocp_data);
	ocp_data |= __cpu_to_le32(0x080000);
	pla_ocp_write(tp, 0xe858, 0x44, sizeof(ocp_data), &ocp_data);

	pla_ocp_read(tp, 0xe81c, sizeof(ocp_data), &ocp_data);
	ocp_data &= __cpu_to_le32(~0x00c0);
	pla_ocp_write(tp, 0xe81c, 0x11, sizeof(ocp_data), &ocp_data);

	pla_ocp_read(tp, 0xe810, sizeof(ocp_data), &ocp_data);
	ocp_data &= __cpu_to_le32(~0x0c000000);
	pla_ocp_write(tp, 0xe810, 0x88, sizeof(ocp_data), &ocp_data);

	pla_ocp_read(tp, 0xe84c, sizeof(ocp_data), &ocp_data);
	ocp_data &= __cpu_to_le32(~0x80000000);
	pla_ocp_write(tp, 0xe84c, 0x88, sizeof(ocp_data), &ocp_data);

	pla_ocp_read(tp, 0xe8dc, sizeof(ocp_data), &ocp_data);
	ocp_data &= __cpu_to_le32(~0x40000000);
	pla_ocp_write(tp, 0xe8dc, 0x88, sizeof(ocp_data), &ocp_data);

	for (i = 0; i < 1000; i++) {
		pla_ocp_read(tp, 0xe84c, sizeof(ocp_data), &ocp_data);
		if (ocp_data & __cpu_to_le32(0x02000000))
			break;
		mdelay(1);
	}

	pla_ocp_read(tp, 0xe8dc, sizeof(ocp_data), &ocp_data);
	ocp_data |= __cpu_to_le32(0x80000000);
	pla_ocp_write(tp, 0xe8dc, 0x88, sizeof(ocp_data), &ocp_data);

	for (i = 0; i < 1000; i++) {
		pla_ocp_read(tp, 0xe84c, sizeof(ocp_data), &ocp_data);
		if (ocp_data & __cpu_to_le32(0x02000000))
			break;
		mdelay(1);
	}

	rtl8152_nic_reset(tp);

	pla_ocp_read(tp, 0xc0a0, sizeof(ocp_data), &ocp_data);
	ocp_data &= __cpu_to_le32(~0x00ff00ff);
	ocp_data |= __cpu_to_le32(0x00080002);
	pla_ocp_write(tp, 0xc0a0, 0x55, sizeof(ocp_data), &ocp_data);
	usb_ocp_read(tp, 0xb808, sizeof(ocp_data), &ocp_data);
	if (ocp_data & __cpu_to_le32(0x06)) {
		ocp_data = __cpu_to_le32(0x00000060);
		pla_ocp_write(tp, 0xc0a4, 0xff, sizeof(ocp_data), &ocp_data);
		ocp_data = __cpu_to_le32(0x00000078);
		pla_ocp_write(tp, 0xc0a8, 0xff, sizeof(ocp_data), &ocp_data);
	} else {
		ocp_data = __cpu_to_le32(0x00000038);
		pla_ocp_write(tp, 0xc0a4, 0xff, sizeof(ocp_data), &ocp_data);
		ocp_data = __cpu_to_le32(0x00000048);
		pla_ocp_write(tp, 0xc0a8, 0xff, sizeof(ocp_data), &ocp_data);
	}
	ocp_data = __cpu_to_le32(0x00400008);
	pla_ocp_write(tp, 0xe618, 0xff, sizeof(ocp_data), &ocp_data);
	usb_ocp_read(tp, 0xd408, sizeof(ocp_data), &ocp_data);
	ocp_data &= __cpu_to_le32(~0x00ff0000);
	ocp_data |= __cpu_to_le32(0x00030000);
	usb_ocp_write(tp, 0xd408, 0x44, sizeof(ocp_data), &ocp_data);
	ocp_data = __cpu_to_le32(0x7a120180);
	usb_ocp_write(tp, 0xd40c, 0xff, sizeof(ocp_data), &ocp_data);
	ocp_data = __cpu_to_le32(0x00000101);
	usb_ocp_write(tp, 0xd434, 0xff, sizeof(ocp_data), &ocp_data);

	pla_ocp_read(tp, 0xe854, sizeof(ocp_data), &ocp_data);
	ocp_data &= __cpu_to_le32(~0x00000040);
	pla_ocp_write(tp, 0xe854, 0x11, sizeof(ocp_data), &ocp_data);

	ocp_data = __cpu_to_le32(0x05ee0000);
	pla_ocp_write(tp, RMS, 0xcc, sizeof(ocp_data), &ocp_data);

	pla_ocp_read(tp, 0xe610, sizeof(ocp_data), &ocp_data);
	ocp_data |= __cpu_to_le32(0x00000080);
	pla_ocp_write(tp, 0xe610, 0x11, sizeof(ocp_data), &ocp_data);
}

static void r8152b_enter_oob(rtl8152_t *tp)
{
	__le32	ocp_data;
	int	i;

	pla_ocp_read(tp, 0xe84c, sizeof(ocp_data), &ocp_data);
	ocp_data &= __cpu_to_le32(~0x80000000);
	pla_ocp_write(tp, 0xe84c, 0x88, sizeof(ocp_data), &ocp_data);

	pla_ocp_read(tp, 0xc0a0, sizeof(ocp_data), &ocp_data);
	ocp_data &= __cpu_to_le32(~0x07ff00ff);
	ocp_data |= __cpu_to_le32(0x01800003);
	pla_ocp_write(tp, 0xc0a0, 0x55, sizeof(ocp_data), &ocp_data);
	ocp_data = __cpu_to_le32(0x0000004a);
	pla_ocp_write(tp, 0xc0a4, 0x11, sizeof(ocp_data), &ocp_data);
	ocp_data = __cpu_to_le32(0x0000005a);
	pla_ocp_write(tp, 0xc0a8, 0x11, sizeof(ocp_data), &ocp_data);

	rtl8152_disable(tp);

	for (i = 0; i < 1000; i++) {
		pla_ocp_read(tp, 0xe84c, sizeof(ocp_data), &ocp_data);
		if (ocp_data & __cpu_to_le32(0x02000000))
			break;
		mdelay(1);
	}

	pla_ocp_read(tp, 0xe8dc, sizeof(ocp_data), &ocp_data);
	ocp_data |= __cpu_to_le32(0x80000000);
	pla_ocp_write(tp, 0xe8dc, 0x88, sizeof(ocp_data), &ocp_data);

	for (i = 0; i < 1000; i++) {
		pla_ocp_read(tp, 0xe84c, sizeof(ocp_data), &ocp_data);
		if (ocp_data & __cpu_to_le32(0x02000000))
			break;
		mdelay(1);
	}

	ocp_data = __cpu_to_le32(0x05f20000);
	pla_ocp_write(tp, RMS, 0xcc, sizeof(ocp_data), &ocp_data);

	pla_ocp_read(tp, 0xc0b4, sizeof(ocp_data), &ocp_data);
	ocp_data |= __cpu_to_le32(0x00010000);
	pla_ocp_write(tp, 0xc0b4, 0x44, sizeof(ocp_data), &ocp_data);

	pla_ocp_read(tp, 0xe854, sizeof(ocp_data), &ocp_data);
	ocp_data |= __cpu_to_le32(0x00000040);
	pla_ocp_write(tp, 0xe854, 0x11, sizeof(ocp_data), &ocp_data);

	pla_ocp_read(tp, 0xd1a0, sizeof(ocp_data), &ocp_data);
	ocp_data |= __cpu_to_le32(0x00000001);
	pla_ocp_write(tp, 0xd1a0, 0x11, sizeof(ocp_data), &ocp_data);

	pla_ocp_read(tp, 0xe84c, sizeof(ocp_data), &ocp_data);
	ocp_data |= __cpu_to_le32(0x01000000);
	pla_ocp_write(tp, 0xe84c, 0x88, sizeof(ocp_data), &ocp_data);

	pla_ocp_read(tp, 0xe84c, sizeof(ocp_data), &ocp_data);
	ocp_data |= __cpu_to_le32(0x80000000);
	pla_ocp_write(tp, 0xe84c, 0x88, sizeof(ocp_data), &ocp_data);

	ocp_data = __cpu_to_le32(0x00020000);
	pla_ocp_write(tp, 0xe820, 0x44, sizeof(ocp_data), &ocp_data);

	pla_ocp_read(tp, 0xe858, sizeof(ocp_data), &ocp_data);
	ocp_data &= __cpu_to_le32(~0x080000);
	pla_ocp_write(tp, 0xe858, 0x44, sizeof(ocp_data), &ocp_data);

	pla_ocp_read(tp, RCR, sizeof(ocp_data), &ocp_data);
	ocp_data |= __cpu_to_le32(0x000e);
	pla_ocp_write(tp, RCR, 0x11, sizeof(ocp_data), &ocp_data);
}

static void r8152b_disable_aldps(rtl8152_t *tp)
{
	__le32	ocp_data;

	ocp_data = __cpu_to_le32(0x2000);
	pla_ocp_write(tp, 0xe86c, 0x33, sizeof(ocp_data), &ocp_data);
	ocp_data = __cpu_to_le32(0x0310);
	pla_ocp_write(tp, 0xb010, 0x33, sizeof(ocp_data), &ocp_data);
	msleep(20);
}

static void r8152b_enable_aldps(rtl8152_t *tp)
{
	__le32	ocp_data;

	ocp_data = __cpu_to_le32(0x2000);
	pla_ocp_write(tp, 0xe86c, 0x33, sizeof(ocp_data), &ocp_data);
	ocp_data = __cpu_to_le32(0x8310);
	pla_ocp_write(tp, 0xb010, 0x33, sizeof(ocp_data), &ocp_data);
}

static void rtl8152_down(rtl8152_t *tp)
{
	__le32	ocp_data;

	usb_ocp_read(tp, 0xd800, sizeof(ocp_data), &ocp_data);
	ocp_data &= __cpu_to_le32(~0x100);
	usb_ocp_write(tp, 0xd800, 0x22, sizeof(ocp_data), &ocp_data);

	r8152b_disable_aldps(tp);
	r8152b_enter_oob(tp);
	r8152b_enable_aldps(tp);
	if (tp->version == RTL_VER_01)
		rtl8152_set_speed(tp, AUTONEG_ENABLE, SPEED_10, DUPLEX_FULL);
}

static void set_carrier(rtl8152_t *tp)
{
	struct net_device *netdev = tp->netdev;
	u8 speed;

	speed = rtl8152_get_speed(tp);

	if (speed & LinkStatus) {
		if (!(tp->speed & LinkStatus)) {
			rtl8152_enable(tp);
			set_bit(RTL8152_SET_RX_MODE, &tp->flags);
			netif_carrier_on(netdev);
		}
	} else {
		if (tp->speed & LinkStatus) {
			netif_carrier_off(netdev);
			rtl8152_disable(tp);
		}
	}
	tp->speed = speed;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,20)

void rtl_work_func_t(void *data)
{
	rtl8152_t *tp = (rtl8152_t *)data;

	if (!netif_running(tp->netdev))
		goto out1;

	if (test_bit(RTL8152_UNPLUG, &tp->flags))
		goto out1;

	set_carrier(tp);

	if (test_bit(RTL8152_SET_RX_MODE, &tp->flags))
		rtl8152_set_rx_mode(tp->netdev);

	schedule_delayed_work(&tp->schedule, HZ);

out1:
	return;
}

#else

static void rtl_work_func_t(struct work_struct *work)
{
	rtl8152_t *tp = container_of(work, rtl8152_t, schedule.work);

	if (!netif_running(tp->netdev))
		goto out1;

	if (test_bit(RTL8152_UNPLUG, &tp->flags))
		goto out1;

	set_carrier(tp);

	if (test_bit(RTL8152_SET_RX_MODE, &tp->flags))
		rtl8152_set_rx_mode(tp->netdev);

	schedule_delayed_work(&tp->schedule, HZ);

out1:
	return;
}

#endif

static int rtl8152_open(struct net_device *netdev)
{
	rtl8152_t *tp = netdev_priv(netdev);
	int res = 0;

	tp->speed = rtl8152_get_speed(tp);
	if (tp->speed & LinkStatus) {
		res = rtl8152_enable(tp);
		netif_carrier_on(netdev);
	} else {
		netif_stop_queue(netdev);
		netif_carrier_off(netdev);
	}

	if (res) {
		if (res == -ENODEV) {
			netif_device_detach(tp->netdev);
		}
		netif_warn(tp, rx_err, netdev, "rx_urb submit failed: %d\n", res);
		return res;
	}
#if 0
	usb_fill_int_urb(tp->intr_urb, tp->udev, usb_rcvintpipe(tp->udev, 3),
		     tp->intr_buff, INTBUFSIZE, intr_callback,
		     tp, tp->intr_interval);
	if ((res = usb_submit_urb(tp->intr_urb, GFP_KERNEL))) {
		if (res == -ENODEV)
			netif_device_detach(tp->netdev);
		dev_warn(&netdev->dev, "intr_urb submit failed: %d\n", res);
		usb_kill_urb(tp->rx_urb);
		return res;
	}
#endif
	rtl8152_set_speed(tp, AUTONEG_ENABLE, SPEED_100, DUPLEX_FULL);
	netif_start_queue(netdev);
	schedule_delayed_work(&tp->schedule, 0);

	return res;
}

static int rtl8152_close(struct net_device *netdev)
{
	rtl8152_t *tp = netdev_priv(netdev);
	int res = 0;

	cancel_delayed_work_sync(&tp->schedule);
	netif_stop_queue(netdev);
	rtl8152_disable(tp);

	return res;
}

static void rtl_clear_bp(rtl8152_t *tp)
{
	__le32	ocp_data;

	ocp_data = 0;
	pla_ocp_write(tp, 0xfc28, 0xff, sizeof(ocp_data), &ocp_data);
	pla_ocp_write(tp, 0xfc2c, 0xff, sizeof(ocp_data), &ocp_data);
	pla_ocp_write(tp, 0xfc30, 0xff, sizeof(ocp_data), &ocp_data);
	pla_ocp_write(tp, 0xfc34, 0xff, sizeof(ocp_data), &ocp_data);
	usb_ocp_write(tp, 0xfc28, 0xff, sizeof(ocp_data), &ocp_data);
	usb_ocp_write(tp, 0xfc2c, 0xff, sizeof(ocp_data), &ocp_data);
	usb_ocp_write(tp, 0xfc30, 0xff, sizeof(ocp_data), &ocp_data);
	usb_ocp_write(tp, 0xfc34, 0xff, sizeof(ocp_data), &ocp_data);
	msleep(3);
	pla_ocp_write(tp, 0xfc24, 0xcc, sizeof(ocp_data), &ocp_data);
	usb_ocp_write(tp, 0xfc24, 0xcc, sizeof(ocp_data), &ocp_data);
}

static void patch1(rtl8152_t *tp)
{
	__le32	ocp_data;
	u8	patch_data[] = {
		0x08, 0xe0, 0x3e, 0xe0,
		0x4d, 0xe0, 0x5a, 0xe0,
		0x8a, 0xe0, 0xce, 0xe0,
		0xd0, 0xe0, 0xd2, 0xe0,
		0x2f, 0xc3, 0x60, 0x72,
		0xa0, 0x49, 0x10, 0xf0,
		0xa4, 0x49, 0x0e, 0xf0,
		0x2a, 0xc3, 0x62, 0x72,
		0x26, 0x70, 0x80, 0x49,
		0x05, 0xf0, 0x2f, 0x48,
		0x62, 0x9a, 0x24, 0x70,
		0x60, 0x98, 0x22, 0xc3,
		0x60, 0x99, 0x21, 0xc3,
		0x00, 0xbb, 0x2c, 0x75,
		0xdc, 0x21, 0xbc, 0x25,
		0x04, 0x13, 0x08, 0xf0,
		0x03, 0x13, 0x06, 0xf0,
		0x02, 0x13, 0x04, 0xf0,
		0x01, 0x13, 0x02, 0xf0,
		0x03, 0xe0, 0xd4, 0x49,
		0x04, 0xf1, 0x14, 0xc2,
		0x12, 0xc3, 0x00, 0xbb,
		0x12, 0xc3, 0x60, 0x75,
		0xd0, 0x49, 0x05, 0xf1,
		0x50, 0x48, 0x60, 0x9d,
		0x09, 0xc6, 0x00, 0xbe,
		0xd0, 0x48, 0x60, 0x9d,
		0xf3, 0xe7, 0xc2, 0xc0,
		0x38, 0xd2, 0xc6, 0xd2,
		0x84, 0x17, 0xa2, 0x13,
		0x0c, 0x17, 0xbc, 0xc0,
		0xa2, 0xd1, 0x0f, 0xc4,
		0x22, 0x40, 0x06, 0xf0,
		0x40, 0x73, 0x20, 0x9b,
		0x42, 0x73, 0x22, 0x9b,
		0x05, 0xe0, 0x42, 0x73,
		0x20, 0x9b, 0x44, 0x73,
		0x22, 0x9b, 0x02, 0xc4,
		0x00, 0xbc, 0xd6, 0x06,
		0x38, 0xd0, 0x0b, 0xc0,
		0x00, 0x71, 0x0a, 0xc0,
		0x00, 0x72, 0xa0, 0x49,
		0x04, 0xf0, 0xa4, 0x49,
		0x02, 0xf0, 0x93, 0x48,
		0x04, 0xc0, 0x00, 0xb8,
		0x00, 0xe4, 0xc2, 0xc0,
		0x8c, 0x09, 0x2c, 0x75,
		0x2f, 0xc3, 0x60, 0x73,
		0xb1, 0x49, 0x0d, 0xf1,
		0xdc, 0x21, 0xbc, 0x25,
		0x24, 0xc6, 0xc0, 0x77,
		0x04, 0x13, 0x11, 0xf0,
		0x03, 0x13, 0x13, 0xf0,
		0x02, 0x13, 0x15, 0xf0,
		0x01, 0x13, 0x17, 0xf0,
		0xd4, 0x49, 0x03, 0xf1,
		0x19, 0xc5, 0x00, 0xbd,
		0x18, 0xc5, 0x00, 0xbd,
		0x17, 0xc5, 0x00, 0xbd,
		0x16, 0xc5, 0x00, 0xbd,
		0xf1, 0x49, 0xfb, 0xf1,
		0x0f, 0xc5, 0x00, 0xbd,
		0xf4, 0x49, 0xf9, 0xf1,
		0x0b, 0xc5, 0x00, 0xbd,
		0xf3, 0x49, 0xf5, 0xf1,
		0x07, 0xc5, 0x00, 0xbd,
		0xf2, 0x49, 0xf1, 0xf1,
		0x03, 0xc5, 0x00, 0xbd,
		0xb6, 0xc0, 0x6a, 0x14,
		0xa2, 0x13, 0xd6, 0x13,
		0xfa, 0x14, 0xa0, 0xd1,
		0xd4, 0x49, 0x28, 0xf0,
		0x02, 0xb4, 0x2a, 0xc4,
		0x00, 0x1d, 0x2e, 0xe8,
		0xe0, 0x73, 0xb9, 0x21,
		0xbd, 0x25, 0x04, 0x13,
		0x02, 0xf0, 0x1a, 0xe0,
		0x22, 0xc4, 0x23, 0xc3,
		0x2f, 0xe8, 0x23, 0xc3,
		0x2d, 0xe8, 0x00, 0x1d,
		0x21, 0xe8, 0xe2, 0x73,
		0xbb, 0x49, 0xfc, 0xf0,
		0xe0, 0x73, 0xb7, 0x48,
		0x03, 0xb4, 0x81, 0x1d,
		0x19, 0xe8, 0x40, 0x1a,
		0x84, 0x1d, 0x16, 0xe8,
		0x12, 0xc3, 0x1e, 0xe8,
		0x03, 0xb0, 0x81, 0x1d,
		0x11, 0xe8, 0x0e, 0xc3,
		0x19, 0xe8, 0x02, 0xb0,
		0x06, 0xc7, 0x04, 0x1e,
		0xe0, 0x9e, 0x02, 0xc6,
		0x00, 0xbe, 0x22, 0x02,
		0x20, 0xe4, 0x04, 0xb8,
		0x34, 0xb0, 0x00, 0x02,
		0x00, 0x03, 0x00, 0x0e,
		0x00, 0x0c, 0x09, 0xc7,
		0xe0, 0x9b, 0xe2, 0x9a,
		0xe4, 0x9c, 0xe6, 0x8d,
		0xe6, 0x76, 0xef, 0x49,
		0xfe, 0xf1, 0x80, 0xff,
		0x08, 0xea, 0x82, 0x1d,
		0xf5, 0xef, 0x00, 0x1a,
		0x88, 0x1d, 0xf2, 0xef,
		0xed, 0xc2, 0xf0, 0xef,
		0x80, 0xff, 0x02, 0xc6,
		0x00, 0xbe, 0x46, 0x06,
		0x02, 0xc6, 0x00, 0xbe,
		0x00, 0x00, 0x36, 0xf0,
		0x08, 0x1c, 0xea, 0x8c,
		0xe3, 0x64, 0xc7, 0x49,
		0x25, 0xf1, 0xe0, 0x75,
		0xff, 0x1b, 0xeb, 0x47,
		0xff, 0x1b, 0x6b, 0x47,
		0xe0, 0x9d, 0x15, 0xc3,
		0x60, 0x75, 0xd8, 0x49,
		0x04, 0xf0, 0x81, 0x1d,
		0xe2, 0x8d, 0x05, 0xe0,
		0xe2, 0x63, 0x81, 0x1d,
		0xdd, 0x47, 0xe2, 0x8b,
		0x0b, 0xc3, 0x00, 0x1d,
		0x61, 0x8d, 0x3c, 0x03,
		0x60, 0x75, 0xd8, 0x49,
		0x06, 0xf1, 0xdf, 0x48,
		0x61, 0x95, 0x16, 0xe0,
		0x4e, 0xe8, 0x12, 0xe8,
		0x21, 0xc5, 0xa0, 0x73,
		0xb0, 0x49, 0x03, 0xf0,
		0x31, 0x48, 0xa0, 0x9b,
		0x0d, 0xe0, 0xc0, 0x49,
		0x0b, 0xf1, 0xe2, 0x63,
		0x7e, 0x1d, 0xdd, 0x46,
		0xe2, 0x8b, 0xe0, 0x75,
		0x83, 0x1b, 0xeb, 0x46,
		0xfe, 0x1b, 0x6b, 0x46,
		0xe0, 0x9d, 0xe4, 0x49,
		0x11, 0xf0, 0x10, 0x1d,
		0xea, 0x8d, 0xe3, 0x64,
		0xc6, 0x49, 0x09, 0xf1,
		0x07, 0xc5, 0xa0, 0x73,
		0xb1, 0x48, 0xa0, 0x9b,
		0x02, 0xc5, 0x00, 0xbd,
		0xe6, 0x04, 0xa0, 0xd1,
		0x02, 0xc5, 0x00, 0xbd,
		0xfe, 0x04, 0x02, 0xc5,
		0x00, 0xbd, 0x30, 0x05
};

	pla_ocp_write(tp, 0xf800, 0xff, sizeof(patch_data), patch_data);
	ocp_data = __cpu_to_le32(0x80000000);
	pla_ocp_write(tp, 0xfc24, 0xcc, sizeof(ocp_data), &ocp_data);

	ocp_data = __cpu_to_le32(0x06e9170b);
	pla_ocp_write(tp, 0xfc28, 0xff, sizeof(ocp_data), &ocp_data);
	ocp_data = __cpu_to_le32(0x13870989);
	pla_ocp_write(tp, 0xfc2c, 0xff, sizeof(ocp_data), &ocp_data);
	ocp_data = __cpu_to_le32(0x061d01b7);
	pla_ocp_write(tp, 0xfc30, 0xff, sizeof(ocp_data), &ocp_data);

	ocp_data = __cpu_to_le32(0x00200000);
	pla_ocp_write(tp, 0xe420, 0xcc, sizeof(ocp_data), &ocp_data);
	ocp_data = __cpu_to_le32(0x00000018);
	pla_ocp_write(tp, 0xe420, 0x33, sizeof(ocp_data), &ocp_data);
	ocp_data = __cpu_to_le32(0x047b0000);
	pla_ocp_write(tp, 0xfc34, 0xcc, sizeof(ocp_data), &ocp_data);
}

static void patch4(rtl8152_t *tp)
{
	__le32	ocp_data;

	usb_ocp_read(tp, 0xd428, sizeof(ocp_data), &ocp_data);
	ocp_data |= __cpu_to_le32(0x8000);
	usb_ocp_write(tp, 0xd428, 0x22, sizeof(ocp_data), &ocp_data);
	ocp_data = __cpu_to_le32(0x02100000);
	usb_ocp_write(tp, 0xc0cc, 0xcc, sizeof(ocp_data), &ocp_data);
	usb_ocp_read(tp, 0xd428, sizeof(ocp_data), &ocp_data);
	ocp_data &= __cpu_to_le32(~0x8000);
	usb_ocp_write(tp, 0xd428, 0x22, sizeof(ocp_data), &ocp_data);
}

static void r8152b_enable_eee(rtl8152_t *tp)
{
	__le32	ocp_data;

	pla_ocp_read(tp, 0xe040, sizeof(ocp_data), &ocp_data);
	ocp_data |= __cpu_to_le32(0x0003);
	pla_ocp_write(tp, 0xe040, 0x33, sizeof(ocp_data), &ocp_data);
	ocp_data = __cpu_to_le32(0x2000);
	pla_ocp_write(tp, 0xe86c, 0x33, sizeof(ocp_data), &ocp_data);
	ocp_data = __cpu_to_le32(0xf31f);
	pla_ocp_write(tp, 0xb080, 0x33, sizeof(ocp_data), &ocp_data);
	ocp_data = __cpu_to_le32(0x76300000);
	pla_ocp_write(tp, 0xb090, 0xcc, sizeof(ocp_data), &ocp_data);
	ocp_data = __cpu_to_le32(0x1566);
	pla_ocp_write(tp, 0xb094, 0x33, sizeof(ocp_data), &ocp_data);
	ocp_data = __cpu_to_le32(0xa000);
	pla_ocp_write(tp, 0xe86c, 0x33, sizeof(ocp_data), &ocp_data);
	ocp_data = __cpu_to_le32(0x00070000);
	pla_ocp_write(tp, 0xb418, 0xcc, sizeof(ocp_data), &ocp_data);
	ocp_data = __cpu_to_le32(0x003c);
	pla_ocp_write(tp, 0xb41c, 0x33, sizeof(ocp_data), &ocp_data);
	ocp_data = __cpu_to_le32(0x40070000);
	pla_ocp_write(tp, 0xb418, 0xcc, sizeof(ocp_data), &ocp_data);
	ocp_data = __cpu_to_le32(0x0002);
	pla_ocp_write(tp, 0xb41c, 0x33, sizeof(ocp_data), &ocp_data);
	ocp_data = 0;
	pla_ocp_write(tp, 0xb418, 0xcc, sizeof(ocp_data), &ocp_data);
}

static void r8152b_enable_fc(rtl8152_t *tp)
{
	__le32	ocp_data;

	ocp_data = __cpu_to_le32(0x0000a000);
	pla_ocp_write(tp, 0xe86c, 0x33, sizeof(ocp_data), &ocp_data);
	pla_ocp_read(tp, 0xb408, sizeof(ocp_data), &ocp_data);
	ocp_data |= __cpu_to_le32(0x00000c00);
	pla_ocp_write(tp, 0xb408, 0x33, sizeof(ocp_data), &ocp_data);
}

static void r8152b_firmware(rtl8152_t *tp)
{
	__le32	ocp_data;
	int	i;
	static u16 ram_code1[] = {
		0x9700, 0x7fe0, 0x4c00, 0x4007,
		0x4400, 0x4800, 0x7c1f, 0x4c00,
		0x5310, 0x6000, 0x7c07, 0x6800,
		0x673e, 0x0000, 0x0000, 0x571f,
		0x5ffb, 0xaa05, 0x5b58, 0x7d80,
		0x6100, 0x3019, 0x5b64, 0x7d80,
		0x6080, 0xa6f8, 0xdcdb, 0x0015,
		0xb915, 0xb511, 0xd16b, 0x000f,
		0xb40f, 0xd06b, 0x000d, 0xb206,
		0x7c01, 0x5800, 0x7c04, 0x5c00,
		0x3011, 0x7c01, 0x5801, 0x7c04,
		0x5c04, 0x3019, 0x30a5, 0x3127,
		0x31d5, 0x7fe0, 0x4c60, 0x7c07,
		0x6803, 0x7d00, 0x6900, 0x65a0,
		0x0000, 0x0000, 0xaf03, 0x6015,
		0x303e, 0x6017, 0x57e0, 0x580c,
		0x588c, 0x7fdd, 0x5fa2, 0x4827,
		0x7c1f, 0x4c00, 0x7c1f, 0x4c10,
		0x8400, 0x7c30, 0x6020, 0x48bf,
		0x7c1f, 0x4c00, 0x7c1f, 0x4c01,
		0x7c07, 0x6803, 0xb806, 0x7c08,
		0x6800, 0x0000, 0x0000, 0x305c,
		0x7c08, 0x6808, 0x0000, 0x0000,
		0xae06, 0x7c02, 0x5c02, 0x0000,
		0x0000, 0x3067, 0x8e05, 0x7c02,
		0x5c00, 0x0000, 0x0000, 0xad06,
		0x7c20, 0x5c20, 0x0000, 0x0000,
		0x3072, 0x8d05, 0x7c20, 0x5c00,
		0x0000, 0x0000, 0xa008, 0x7c07,
		0x6800, 0xb8db, 0x7c07, 0x6803,
		0xd9b3, 0x00d7, 0x7fe0, 0x4c80,
		0x7c08, 0x6800, 0x0000, 0x0000,
		0x7c23, 0x5c23, 0x481d, 0x7c1f,
		0x4c00, 0x7c1f, 0x4c02, 0x5310,
		0x81ff, 0x30f5, 0x7fe0, 0x4d00,
		0x4832, 0x7c1f, 0x4c00, 0x7c1f,
		0x4c10, 0x7c08, 0x6000, 0xa49e,
		0x7c07, 0x6800, 0xb89b, 0x7c07,
		0x6803, 0xd9b3, 0x00f9, 0x7fe0,
		0x4d20, 0x7e00, 0x6200, 0x3001,
		0x7fe0, 0x4dc0, 0xd09d, 0x0002,
		0xb4fe, 0x7fe0, 0x4d80, 0x7c04,
		0x6004, 0x7c07, 0x6802, 0x6728,
		0x0000, 0x0000, 0x7c08, 0x6000,
		0x486c, 0x7c1f, 0x4c00, 0x7c1f,
		0x4c01, 0x9503, 0x7e00, 0x6200,
		0x571f, 0x5fbb, 0xaa05, 0x5b58,
		0x7d80, 0x6100, 0x30c2, 0x5b64,
		0x7d80, 0x6080, 0xcdab, 0x0063,
		0xcd8d, 0x0061, 0xd96b, 0x005f,
		0xd0a0, 0x00d7, 0xcba0, 0x0003,
		0x80ec, 0x30cf, 0x30dc, 0x7fe0,
		0x4ce0, 0x4832, 0x7c1f, 0x4c00,
		0x7c1f, 0x4c08, 0x7c08, 0x6008,
		0x8300, 0xb902, 0x30a5, 0x308a,
		0x7fe0, 0x4da0, 0x65a8, 0x0000,
		0x0000, 0x56a0, 0x590c, 0x7ffd,
		0x5fa2, 0xae06, 0x7c02, 0x5c02,
		0x0000, 0x0000, 0x30f0, 0x8e05,
		0x7c02, 0x5c00, 0x0000, 0x0000,
		0xcba4, 0x0004, 0xcd8d, 0x0002,
		0x80f1, 0x7fe0, 0x4ca0, 0x7c08,
		0x6408, 0x0000, 0x0000, 0x7d00,
		0x6800, 0xb603, 0x7c10, 0x6010,
		0x7d1f, 0x551f, 0x5fb3, 0xaa07,
		0x7c80, 0x5800, 0x5b58, 0x7d80,
		0x6100, 0x310f, 0x7c80, 0x5800,
		0x5b64, 0x7d80, 0x6080, 0x4827,
		0x7c1f, 0x4c00, 0x7c1f, 0x4c10,
		0x8400, 0x7c10, 0x6000, 0x7fe0,
		0x4cc0, 0x5fbb, 0x4824, 0x7c1f,
		0x4c00, 0x7c1f, 0x4c04, 0x8200,
		0x7ce0, 0x5400, 0x6728, 0x0000,
		0x0000, 0x30cf, 0x3001, 0x7fe0,
		0x4e00, 0x4007, 0x4400, 0x5310,
		0x7c07, 0x6800, 0x673e, 0x0000,
		0x0000, 0x570f, 0x5fff, 0xaa05,
		0x585b, 0x7d80, 0x6100, 0x313b,
		0x5867, 0x7d80, 0x6080, 0x9403,
		0x7e00, 0x6200, 0xcda3, 0x00e7,
		0xcd85, 0x00e5, 0xd96b, 0x00e3,
		0x96e3, 0x7c07, 0x6800, 0x673e,
		0x0000, 0x0000, 0x7fe0, 0x4e20,
		0x96db, 0x8b04, 0x7c08, 0x5008,
		0xab03, 0x7c08, 0x5000, 0x7c07,
		0x6801, 0x677e, 0x0000, 0x0000,
		0xdb7c, 0x00ec, 0x0000, 0x7fe1,
		0x4f40, 0x4837, 0x4418, 0x41c7,
		0x7fe0, 0x4e40, 0x7c40, 0x5400,
		0x7c1f, 0x4c01, 0x7c1f, 0x4c01,
		0x8fbf, 0xd2a0, 0x004b, 0x9204,
		0xa042, 0x3168, 0x3127, 0x7fe1,
		0x4f60, 0x489c, 0x4628, 0x7fe0,
		0x4e60, 0x7e28, 0x4628, 0x7c40,
		0x5400, 0x7c01, 0x5800, 0x7c04,
		0x5c00, 0x41e8, 0x7c1f, 0x4c01,
		0x7c1f, 0x4c01, 0x8fa5, 0xb241,
		0xa02a, 0x3182, 0x7fe0, 0x4ea0,
		0x7c02, 0x4402, 0x4448, 0x4894,
		0x7c1f, 0x4c01, 0x7c1f, 0x4c03,
		0x4824, 0x7c1f, 0x4c07, 0x41ef,
		0x41ff, 0x4891, 0x7c1f, 0x4c07,
		0x7c1f, 0x4c17, 0x8400, 0x8ef8,
		0x41c7, 0x8f8a, 0x92d5, 0xa10f,
		0xd480, 0x0008, 0xd580, 0x00b8,
		0xa202, 0x319d, 0x7c04, 0x4404,
		0x319d, 0xd484, 0x00f3, 0xd484,
		0x00f1, 0x3127, 0x7fe0, 0x4ee0,
		0x7c40, 0x5400, 0x4488, 0x41cf,
		0x3127, 0x7fe0, 0x4ec0, 0x48f3,
		0x7c1f, 0x4c01, 0x7c1f, 0x4c09,
		0x4508, 0x41c7, 0x8fb0, 0xd218,
		0x00ae, 0xd2a4, 0x009e, 0x31be,
		0x7fe0, 0x4e80, 0x4832, 0x7c1f,
		0x4c01, 0x7c1f, 0x4c11, 0x4428,
		0x7c40, 0x5440, 0x7c01, 0x5801,
		0x7c04, 0x5c04, 0x41e8, 0xa4b3,
		0x31d3, 0x7fe0, 0x4f20, 0x7c07,
		0x6800, 0x673e, 0x0000, 0x0000,
		0x570f, 0x5fff, 0xaa04, 0x585b,
		0x6100, 0x31e4, 0x5867, 0x6080,
		0xbcf1, 0x3001
	};

	ocp_data = __cpu_to_le32(0x2000);
	pla_ocp_write(tp, 0xe86c, 0x33, sizeof(ocp_data), &ocp_data);
	ocp_data = __cpu_to_le32(0x70700000);
	pla_ocp_write(tp, 0xb090, 0xcc, sizeof(ocp_data), &ocp_data);
	ocp_data = __cpu_to_le32(0x0600);
	pla_ocp_write(tp, 0xb098, 0x33, sizeof(ocp_data), &ocp_data);
	for (i = 0; i < ARRAY_SIZE(ram_code1); i++) {
		ocp_data = __cpu_to_le32(ram_code1[i] << 16);
		pla_ocp_write(tp, 0xb098, 0xcc, sizeof(ocp_data), &ocp_data);
	}
	ocp_data = __cpu_to_le32(0x0200);
	pla_ocp_write(tp, 0xb098, 0x33, sizeof(ocp_data), &ocp_data);
	ocp_data = __cpu_to_le32(0x70300000);
	pla_ocp_write(tp, 0xb090, 0xcc, sizeof(ocp_data), &ocp_data);
}

static void r8152b_hw_phy_cfg(rtl8152_t *tp)
{
	__le32	ocp_data;

	ocp_data = __cpu_to_le32(0xa000);
	pla_ocp_write(tp, 0xe86c, 0x33, sizeof(ocp_data), &ocp_data);
	ocp_data = __cpu_to_le32(0x1000);
	pla_ocp_write(tp, 0xb400, 0x33, sizeof(ocp_data), &ocp_data);

	r8152b_disable_aldps(tp);

	if (tp->version == RTL_VER_01)
		r8152b_firmware(tp);
}

static void r8152b_init(rtl8152_t *tp)
{
	__le32	ocp_data;
	int	i;

	rtl_clear_bp(tp);

	if (tp->version == RTL_VER_01) {
		patch1(tp);
		patch4(tp);

		pla_ocp_read(tp, 0xdd90, sizeof(ocp_data), &ocp_data);
		ocp_data &= __cpu_to_le32(~0x07000000);
		pla_ocp_write(tp, 0xdd90, 0x88, sizeof(ocp_data), &ocp_data);
	}

	r8152b_hw_phy_cfg(tp);

	usb_ocp_read(tp, 0xd800, sizeof(ocp_data), &ocp_data);
	ocp_data &= __cpu_to_le32(~0x100);
	usb_ocp_write(tp, 0xd800, 0x22, sizeof(ocp_data), &ocp_data);

	usb_ocp_read(tp, 0xd430, sizeof(ocp_data), &ocp_data);
	ocp_data &= __cpu_to_le32(~0x010000);
	usb_ocp_write(tp, 0xd430, 0x44, sizeof(ocp_data), &ocp_data);

	r8152b_exit_oob(tp);

	pla_ocp_read(tp, 0xe84c, sizeof(ocp_data), &ocp_data);
	ocp_data |= __cpu_to_le32(0x00c0);
	pla_ocp_write(tp, 0xe84c, 0x11, sizeof(ocp_data), &ocp_data);
	ocp_data = __cpu_to_le32(0x07014f07);
	pla_ocp_write(tp, 0xe0c0, 0xff, sizeof(ocp_data), &ocp_data);
	ocp_data = __cpu_to_le32(0x000f0000);
	pla_ocp_write(tp, 0xe020, 0xcc, sizeof(ocp_data), &ocp_data);

	r8152b_enable_eee(tp);
	r8152b_enable_aldps(tp);
	r8152b_enable_fc(tp);

	ocp_data = __cpu_to_le32(0x0000a000);
	pla_ocp_write(tp, 0xe86c, 0x33, sizeof(ocp_data), &ocp_data);
	ocp_data = __cpu_to_le32(0x00009200);
	pla_ocp_write(tp, 0xb400, 0x33, sizeof(ocp_data), &ocp_data);
	for (i = 0; i < 100; i++) {
		udelay(100);
		pla_ocp_read(tp, 0xb400, sizeof(ocp_data), &ocp_data);
		if (!(__le32_to_cpu(ocp_data) & 0x8000))
			break;
	}

	// disable rx aggregation
	usb_ocp_read(tp, 0xd404, sizeof(ocp_data), &ocp_data);
	ocp_data |= __cpu_to_le32(0x00100000);
	usb_ocp_write(tp, 0xd404, 0xcc, sizeof(ocp_data), &ocp_data);

//	usb_ocp_read(tp, 0xd428, sizeof(ocp_data), &ocp_data);
//	ocp_data |= __cpu_to_le32(0x8000);
//	usb_ocp_write(tp, 0xd428, 0x22, sizeof(ocp_data), &ocp_data);
//	usb_ocp_read(tp, 0xc0b4, sizeof(ocp_data), &ocp_data);
//	ocp_data &= __cpu_to_le32(~0x00ff0000);
//	ocp_data |= __cpu_to_le32(0x00160000);
//	usb_ocp_write(tp, 0xc0b4, 0x44, sizeof(ocp_data), &ocp_data);
//	usb_ocp_read(tp, 0xc0b8, sizeof(ocp_data), &ocp_data);
//	ocp_data &= __cpu_to_le32(~0x000000ff);
//	ocp_data |= __cpu_to_le32(0x00000002);
//	usb_ocp_write(tp, 0xc0b8, 0x11, sizeof(ocp_data), &ocp_data);
//	usb_ocp_read(tp, 0xd428, sizeof(ocp_data), &ocp_data);
//	ocp_data &= __cpu_to_le32(~0x8000);
//	usb_ocp_write(tp, 0xd428, 0x22, sizeof(ocp_data), &ocp_data);
}

static int rtl8152_suspend(struct usb_interface *intf, pm_message_t message)
{
	rtl8152_t *tp = usb_get_intfdata(intf);

	netif_device_detach(tp->netdev);

	if (netif_running(tp->netdev))
		cancel_delayed_work_sync(&tp->schedule);

	rtl8152_down(tp);
	usb_control_msg(tp->udev, usb_sndctrlpipe(tp->udev, 0),
			USB_REQ_SET_FEATURE, USB_RECIP_DEVICE,
			USB_DEVICE_REMOTE_WAKEUP, 0, NULL, 0,
			500);

	return 0;
}

static int rtl8152_resume(struct usb_interface *intf)
{
	rtl8152_t *tp = usb_get_intfdata(intf);

	usb_control_msg(tp->udev, usb_sndctrlpipe(tp->udev, 0),
			USB_REQ_CLEAR_FEATURE, USB_RECIP_DEVICE,
			USB_DEVICE_REMOTE_WAKEUP, 0, NULL, 0,
			500);

	r8152b_init(tp);
	netif_device_attach(tp->netdev);
	if (netif_running(tp->netdev)) {
		rtl8152_enable(tp);
		rtl8152_set_speed(tp, AUTONEG_ENABLE, SPEED_100, DUPLEX_FULL);
		set_bit(RTL8152_SET_RX_MODE, &tp->flags);
		schedule_delayed_work(&tp->schedule, 0);
	}

	return 0;
}

static int rtl8152_set_speed(rtl8152_t *tp, u8 autoneg, u16 speed, u8 duplex)
{
	u16 bmcr, anar;
	int ret = 0;

	cancel_delayed_work_sync(&tp->schedule);
	write_mii_word(tp, 0x1f, 0x0000);
	anar = read_mii_word(tp, MII_ADVERTISE);
	anar &= ~(ADVERTISE_10HALF | ADVERTISE_10FULL |
		  ADVERTISE_100HALF | ADVERTISE_100FULL);

	if (autoneg == AUTONEG_DISABLE) {
		if (speed == SPEED_10) {
			bmcr = 0;
			anar |= ADVERTISE_10HALF | ADVERTISE_10FULL;
		} else if (speed == SPEED_100) {
			bmcr = BMCR_SPEED100;
			anar |= ADVERTISE_100HALF | ADVERTISE_100FULL;
		} else {
			ret = -EINVAL;
			goto out;
		}

		if (duplex == DUPLEX_FULL)
			bmcr |= BMCR_FULLDPLX;
	} else {
		if (speed == SPEED_10) {
			if (duplex == DUPLEX_FULL) {
				anar |= ADVERTISE_10HALF | ADVERTISE_10FULL;
			} else {
				anar |= ADVERTISE_10HALF;
			}
		} else if (speed == SPEED_100) {
			if (duplex == DUPLEX_FULL) {
				anar |= ADVERTISE_10HALF | ADVERTISE_10FULL;
				anar |= ADVERTISE_100HALF | ADVERTISE_100FULL;
			} else {
				anar |= ADVERTISE_10HALF;
				anar |= ADVERTISE_100HALF;
			}
		} else {
			ret = -EINVAL;
			goto out;
		}

		bmcr = BMCR_ANENABLE | BMCR_ANRESTART;
	}

	write_mii_word(tp, MII_ADVERTISE, anar);
	write_mii_word(tp, MII_BMCR, bmcr);

out:
	schedule_delayed_work(&tp->schedule, 5 * HZ);

	return ret;
}

static void rtl8152_get_drvinfo(struct net_device *netdev, struct ethtool_drvinfo *info)
{
	rtl8152_t *tp = netdev_priv(netdev);

	strncpy(info->driver, MODULENAME, ETHTOOL_BUSINFO_LEN);
	strncpy(info->version, DRIVER_VERSION, ETHTOOL_BUSINFO_LEN);
	usb_make_path(tp->udev, info->bus_info, sizeof info->bus_info);
}

static int rtl8152_get_settings(struct net_device *netdev, struct ethtool_cmd *ecmd)
{
	rtl8152_t *tp = netdev_priv(netdev);
	u16 bmcr, bmsr, stat1000 = 0;

	ecmd->supported =
	    (SUPPORTED_10baseT_Half | SUPPORTED_10baseT_Full |
	     SUPPORTED_100baseT_Half | SUPPORTED_100baseT_Full |
	     SUPPORTED_Autoneg | SUPPORTED_MII);

	/* only supports twisted-pair */
	ecmd->port = PORT_MII;

	/* only supports internal transceiver */
	ecmd->transceiver = XCVR_INTERNAL;
	ecmd->phy_address = 32;
//	ecmd->mdio_support = ETH_MDIO_SUPPORTS_C22;
	ecmd->advertising = ADVERTISED_MII;

	write_mii_word(tp, 0x1f, 0x0000);
	bmcr = read_mii_word(tp, MII_BMCR);
	bmsr = read_mii_word(tp, MII_BMSR);
	stat1000 = read_mii_word(tp, MII_STAT1000);

	if (bmcr & BMCR_ANENABLE) {
		int advert;

		ecmd->advertising |= ADVERTISED_Autoneg;
		ecmd->autoneg = AUTONEG_ENABLE;

		advert = read_mii_word(tp, MII_ADVERTISE);
		if (advert & ADVERTISE_10HALF)
			ecmd->advertising |= ADVERTISED_10baseT_Half;
		if (advert & ADVERTISE_10FULL)
			ecmd->advertising |= ADVERTISED_10baseT_Full;
		if (advert & ADVERTISE_100HALF)
			ecmd->advertising |= ADVERTISED_100baseT_Half;
		if (advert & ADVERTISE_100FULL)
			ecmd->advertising |= ADVERTISED_100baseT_Full;
		if (advert & ADVERTISE_PAUSE_CAP)
			ecmd->advertising |= ADVERTISED_Pause;
		if (advert & ADVERTISE_PAUSE_ASYM)
			ecmd->advertising |= ADVERTISED_Asym_Pause;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,31)
		if (bmsr & BMSR_ANEGCOMPLETE) {
			advert = read_mii_word(tp, MII_LPA);
			if (advert & LPA_LPACK)
				ecmd->lp_advertising |= ADVERTISED_Autoneg;
			if (advert & ADVERTISE_10HALF)
				ecmd->lp_advertising |=
					ADVERTISED_10baseT_Half;
			if (advert & ADVERTISE_10FULL)
				ecmd->lp_advertising |=
					ADVERTISED_10baseT_Full;
			if (advert & ADVERTISE_100HALF)
				ecmd->lp_advertising |=
					ADVERTISED_100baseT_Half;
			if (advert & ADVERTISE_100FULL)
				ecmd->lp_advertising |=
					ADVERTISED_100baseT_Full;

			if (stat1000 & LPA_1000HALF)
				ecmd->lp_advertising |=
					ADVERTISED_1000baseT_Half;
			if (stat1000 & LPA_1000FULL)
				ecmd->lp_advertising |=
					ADVERTISED_1000baseT_Full;
		} else {
			ecmd->lp_advertising = 0;
		}
#endif
	} else {
		ecmd->autoneg = AUTONEG_DISABLE;
	}

	if (tp->speed & _100bps) {
		ecmd->speed = SPEED_100;
	} else if (tp->speed & _10bps) {
		ecmd->speed = SPEED_10;
	}

	ecmd->duplex = (tp->speed & FullDup) ? DUPLEX_FULL : DUPLEX_HALF;

	return 0;
}

static int rtl8152_set_settings(struct net_device *dev, struct ethtool_cmd *cmd)
{
	rtl8152_t *tp = netdev_priv(dev);

	return rtl8152_set_speed(tp, cmd->autoneg, cmd->speed, cmd->duplex);
}

static struct ethtool_ops ops = {
	.get_drvinfo = rtl8152_get_drvinfo,
	.get_settings = rtl8152_get_settings,
	.set_settings = rtl8152_set_settings,
	.get_link = ethtool_op_get_link,
};

static int rtltool_ioctl(rtl8152_t *tp, struct ifreq *ifr)
{
	struct rtltool_cmd my_cmd, *myptr;
	struct usb_device_info *uinfo;
	struct usb_device *udev;
	__le32	ocp_data;
	void	*buffer;
	int	ret;

	myptr = (struct rtltool_cmd *)ifr->ifr_data;
	if (copy_from_user(&my_cmd, myptr, sizeof(my_cmd)))
		return -EFAULT;

	ret = 0;

	switch (my_cmd.cmd) {
	case RTLTOOL_PLA_OCP_READ_DWORD:
		pla_ocp_read(tp, (u16)my_cmd.offset, sizeof(ocp_data), &ocp_data);
		my_cmd.data = __le32_to_cpu(ocp_data);

		if (copy_to_user(myptr, &my_cmd, sizeof(my_cmd))) {
			ret = -EFAULT;
			break;
		}
		break;

	case RTLTOOL_PLA_OCP_WRITE_DWORD:
		ocp_data = __cpu_to_le32(my_cmd.data);
		pla_ocp_write(tp, (u16)my_cmd.offset, (u16)my_cmd.byteen, sizeof(ocp_data), &ocp_data);
		break;

	case RTLTOOL_USB_OCP_READ_DWORD:
		usb_ocp_read(tp, (u16)my_cmd.offset, sizeof(ocp_data), &ocp_data);
		my_cmd.data = __le32_to_cpu(ocp_data);

		if (copy_to_user(myptr, &my_cmd, sizeof(my_cmd))) {
			ret = -EFAULT;
			break;
		}
		break;


	case RTLTOOL_USB_OCP_WRITE_DWORD:
		ocp_data = __cpu_to_le32(my_cmd.data);
		usb_ocp_write(tp, (u16)my_cmd.offset, (u16)my_cmd.byteen, sizeof(ocp_data), &ocp_data);
		break;

	case RTLTOOL_PLA_OCP_READ:
		buffer = kmalloc(my_cmd.data, GFP_KERNEL);
		if (!buffer) {
			ret = -ENOMEM;
			break;
		}

		pla_ocp_read(tp, (u16)my_cmd.offset, my_cmd.data, buffer);

		if (copy_to_user(myptr->buf, buffer, my_cmd.data))
			ret = -EFAULT;

		kfree(buffer);
		break;

	case RTLTOOL_PLA_OCP_WRITE:
		buffer = kmalloc(my_cmd.data, GFP_KERNEL);
		if (!buffer) {
			ret = -ENOMEM;
			break;
		}

		if (copy_from_user(buffer, myptr->buf, my_cmd.data)) {
			ret = -EFAULT;
			kfree(buffer);
			break;
		}

		pla_ocp_write(tp, (u16)my_cmd.offset, (u16)my_cmd.byteen, my_cmd.data, buffer);
		kfree(buffer);
		break;

	case RTLTOOL_USB_OCP_READ:
		buffer = kmalloc(my_cmd.data, GFP_KERNEL);
		if (!buffer) {
			ret = -ENOMEM;
			break;
		}

		usb_ocp_read(tp, (u16)my_cmd.offset, my_cmd.data, buffer);

		if (copy_to_user(myptr->buf, buffer, my_cmd.data))
			ret = -EFAULT;

		kfree(buffer);
		break;

	case RTLTOOL_USB_OCP_WRITE:
		buffer = kmalloc(my_cmd.data, GFP_KERNEL);
		if (!buffer) {
			ret = -ENOMEM;
			break;
		}

		if (copy_from_user(buffer, myptr->buf, my_cmd.data)) {
			ret = -EFAULT;
			kfree(buffer);
			break;
		}

		usb_ocp_write(tp, (u16)my_cmd.offset, (u16)my_cmd.byteen, my_cmd.data, buffer);
		kfree(buffer);
		break;

	case RTLTOOL_USB_INFO:
		uinfo = (struct usb_device_info *)&my_cmd.nic_info;
		udev = tp->udev;
		uinfo->idVendor = udev->descriptor.idVendor;
		uinfo->idProduct = udev->descriptor.idProduct;
		uinfo->bcdDevice = udev->descriptor.bcdDevice;
		memcpy(uinfo->devpath, udev->devpath, sizeof(udev->devpath));
		pla_ocp_read(tp, IDR, sizeof(uinfo->dev_addr), uinfo->dev_addr);

		if (copy_to_user(myptr, &my_cmd, sizeof(my_cmd)))
			ret = -EFAULT;

		break;
	default:
		ret = -EOPNOTSUPP;
		break;
	}

	return ret;
}

static int rtl8152_ioctl(struct net_device *netdev, struct ifreq *rq, int cmd)
{
	rtl8152_t *tp = netdev_priv(netdev);
	struct mii_ioctl_data *data = if_mii(rq);
	int res = 0;

	switch (cmd) {
	case SIOCGMIIPHY:
		data->phy_id = 32; /* Internal PHY */
		break;

	case SIOCGMIIREG:
		write_mii_word(tp, 0x1f, 0x0000);
		data->val_out = read_mii_word(tp, data->reg_num);
		break;

	case SIOCSMIIREG:
		if (!capable(CAP_NET_ADMIN)) {
			res = -EPERM;
			break;
		}
		write_mii_word(tp, 0x1F, 0x0000);
		write_mii_word(tp, data->reg_num, data->val_in);
		break;

	case SIOCDEVPRIVATE:
		res = rtltool_ioctl(tp, rq);
		break;

	default:
		res = -EOPNOTSUPP;
	}

	return res;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,29)
static const struct net_device_ops rtl8152_netdev_ops = {
	.ndo_open		= rtl8152_open,
	.ndo_stop		= rtl8152_close,
	.ndo_do_ioctl		= rtl8152_ioctl,
	.ndo_start_xmit		= rtl8152_start_xmit,
	.ndo_tx_timeout 	= rtl8152_tx_timeout,
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,1,0)
	.ndo_set_multicast_list = rtl8152_set_rx_mode,
#else
	.ndo_set_rx_mode	= rtl8152_set_rx_mode,
#endif
	.ndo_set_mac_address	= rtl8152_set_mac_address,

	.ndo_change_mtu		= eth_change_mtu,
	.ndo_validate_addr	= eth_validate_addr,
};
#endif

static void r8152b_get_version(rtl8152_t *tp)
{
	__le32	ocp_data;
	u16	version;

	pla_ocp_read(tp, 0xe610, sizeof(ocp_data), &ocp_data);
	version = (u16)((__le32_to_cpu(ocp_data) >> 16) & 0x7cf0);

	switch (version) {
	case 0x4c00:
		tp->version = RTL_VER_01;
		break;
	case 0x4c10:
		tp->version = RTL_VER_02;
		break;
	default:
		printk("Unknown version 0x%04x\n", version);
		break;
	}
}

static int rtl8152_probe(struct usb_interface *intf,
			 const struct usb_device_id *id)
{
	struct usb_device *udev = interface_to_usbdev(intf);
	rtl8152_t *tp;
	struct net_device *netdev;
/***add by nition s at 2013-06-20************/
#if 0
	int ret = 0;
	ret = gpio_request(RK30_PIN0_PD6, NULL);
	if (ret != 0) {
		gpio_free(RK30_PIN0_PD6);
	}
	gpio_direction_output(RK30_PIN0_PD6, 1);
	gpio_set_value(RK30_PIN0_PD6, GPIO_HIGH);
#endif
/***add by nition s at 2013-06-20************/
	if (udev->actconfig->desc.bConfigurationValue != 1 ) {
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)
		printk("The kernel too old to set configuration!!!");
#else
		usb_driver_set_configuration(udev, 1);
#endif
		return -ENODEV;
	}

	usb_reset_device(udev);
	netdev = alloc_etherdev(sizeof(rtl8152_t));
	if (!netdev) {
		printk("Out of memory");
		return -ENOMEM;
	}

	tp = netdev_priv(netdev);
	tp->msg_enable = 0x7FFF;

	tp->intr_buff = kmalloc(INTBUFSIZE, GFP_KERNEL);
	if (!tp->intr_buff) {
		free_netdev(netdev);
		return -ENOMEM;
	}

	tasklet_init(&tp->tl, rx_fixup, (unsigned long)tp);
	INIT_DELAYED_WORK(&tp->schedule, rtl_work_func_t);

	tp->udev = udev;
	tp->netdev = netdev;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,29)
	netdev->netdev_ops = &rtl8152_netdev_ops;
#else
	netdev->open = rtl8152_open;
	netdev->stop = rtl8152_close;
	netdev->get_stats = rtl8152_get_stats;
	netdev->hard_start_xmit = rtl8152_start_xmit;
	netdev->tx_timeout = rtl8152_tx_timeout;
//	netdev->change_mtu = eth_change_mtu;
	netdev->set_mac_address = rtl8152_set_mac_address;
	netdev->do_ioctl = rtl8152_ioctl;
	netdev->set_multicast_list = rtl8152_set_rx_mode;
#endif /* HAVE_NET_DEVICE_OPS */
	netdev->watchdog_timeo = RTL8152_TX_TIMEOUT;
	netdev->features &= ~NETIF_F_IP_CSUM;
	SET_ETHTOOL_OPS(netdev, &ops);
	tp->intr_interval = 100;	/* 100ms */
	tp->speed = 0;

	r8152b_get_version(tp);
	r8152b_init(tp);
	set_ethernet_addr(tp);

	if (!alloc_all_urbs(tp)) {
		printk("out of memory");
		goto out;
	}

	tp->rx_skb = netdev_alloc_skb(netdev, RTL8152_MTU + sizeof(RxDesc));
	if (!tp->rx_skb)
		goto out1;

	usb_set_intfdata(intf, tp);
	SET_NETDEV_DEV(netdev, &intf->dev);


	if (register_netdev(netdev) != 0) {
		printk("couldn't register the device");
		goto out2;
	}

	dev_info(&intf->dev, "%s: This product is covered by one or more "
			"of the following patents:\n\t\tUS6,570,884, "
			"US6,115,776, and US6,327,625.\n", netdev->name);

	return 0;

out2:
	usb_set_intfdata(intf, NULL);
	dev_kfree_skb(tp->rx_skb);
out1:
	free_all_urbs(tp);
out:
	kfree(tp->intr_buff);
	free_netdev(netdev);
	return -EIO;
}

static void rtl8152_unload(rtl8152_t *tp)
{
	__le32	ocp_data;

	usb_ocp_read(tp, 0xd800, sizeof(ocp_data), &ocp_data);
	ocp_data |= __cpu_to_le32(0x100);
	usb_ocp_write(tp, 0xd800, 0x22, sizeof(ocp_data), &ocp_data);

	usb_ocp_read(tp, 0xd430, sizeof(ocp_data), &ocp_data);
	ocp_data &= __cpu_to_le32(~0x010000);
	usb_ocp_write(tp, 0xd430, 0x44, sizeof(ocp_data), &ocp_data);

	usb_ocp_read(tp, 0xb800, sizeof(ocp_data), &ocp_data);
	ocp_data &= __cpu_to_le32(~0x07f0);
	usb_ocp_write(tp, 0xb800, 0x33, sizeof(ocp_data), &ocp_data);
}

static void rtl8152_disconnect(struct usb_interface *intf)
{
	rtl8152_t *tp = usb_get_intfdata(intf);

	usb_set_intfdata(intf, NULL);
	if (tp) {
		set_bit(RTL8152_UNPLUG, &tp->flags);
		tasklet_kill(&tp->tl);
		unregister_netdev(tp->netdev);
		rtl8152_unload(tp);
		free_all_urbs(tp);
		if (tp->rx_skb)
			dev_kfree_skb(tp->rx_skb);
		kfree(tp->intr_buff);
		free_netdev(tp->netdev);
	}
}

/* table of devices that work with this driver */
static struct usb_device_id rtl8152_table[] = {
	{USB_DEVICE_VER(VENDOR_ID_REALTEK, PRODUCT_ID_RTL8152, 0x2000, 0x2000)},
//	{USB_DEVICE(VENDOR_ID_REALTEK, PRODUCT_ID_RTL8152)},
	{}
};

MODULE_DEVICE_TABLE(usb, rtl8152_table);

static struct usb_driver rtl8152_driver = {
	.name =		MODULENAME,
	.probe =	rtl8152_probe,
	.disconnect =	rtl8152_disconnect,
	.id_table =	rtl8152_table,
	.suspend =	rtl8152_suspend,
	.resume =	rtl8152_resume
};

static int __init usb_rtl8152_init(void)
{
	return usb_register(&rtl8152_driver);
}

static void __exit usb_rtl8152_exit(void)
{
	usb_deregister(&rtl8152_driver);
}

module_init(usb_rtl8152_init);
module_exit(usb_rtl8152_exit);

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");
