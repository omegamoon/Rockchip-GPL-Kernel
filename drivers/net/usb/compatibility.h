#ifndef LINUX_COMPATIBILITY_H
#define LINUX_COMPATIBILITY_H

/*
 * Definition and macro
 */

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,37)
	#define skb_checksum_none_assert(skb_ptr)	(skb_ptr)->ip_summed = CHECKSUM_NONE
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35)
	#define pci_dev_run_wake(pdev_ptr)		1
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,34)
	#define netdev_mc_count(netdev)			((netdev)->mc_count)
	#define netdev_mc_empty(netdev)			(netdev_mc_count(netdev) == 0)

	#define netif_printk(priv, type, level, netdev, fmt, args...)	\
	do {					  			\
		if (netif_msg_##type(priv))				\
			printk(level "%s: " fmt,(netdev)->name , ##args); \
	} while (0)

	#define netif_emerg(priv, type, netdev, fmt, args...)		\
		netif_printk(priv, type, KERN_EMERG, netdev, fmt, ##args)
	#define netif_alert(priv, type, netdev, fmt, args...)		\
		netif_printk(priv, type, KERN_ALERT, netdev, fmt, ##args)
	#define netif_crit(priv, type, netdev, fmt, args...)		\
		netif_printk(priv, type, KERN_CRIT, netdev, fmt, ##args)
	#define netif_err(priv, type, netdev, fmt, args...)		\
		netif_printk(priv, type, KERN_ERR, netdev, fmt, ##args)
	#define netif_warn(priv, type, netdev, fmt, args...)		\
		netif_printk(priv, type, KERN_WARNING, netdev, fmt, ##args)
	#define netif_notice(priv, type, netdev, fmt, args...)		\
		netif_printk(priv, type, KERN_NOTICE, netdev, fmt, ##args)
	#define netif_info(priv, type, netdev, fmt, args...)		\
		netif_printk(priv, type, KERN_INFO, (netdev), fmt, ##args)
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,32)
	#define pm_request_resume(para)
	#define pm_runtime_set_suspended(para)
	#define pm_schedule_suspend(para1, para2)
	#define pm_runtime_get_sync(para)
	#define pm_runtime_put_sync(para)
	#define pm_runtime_put_noidle(para)
	#define pm_runtime_idle(para)
	#define pm_runtime_set_active(para)
	#define pm_runtime_enable(para)
	#define pm_runtime_disable(para)
	typedef int netdev_tx_t;
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,24)
	#define napi_enable(napi_ptr)			netif_poll_enable(tp->dev)
	#define napi_disable(napi_ptr)			netif_poll_disable(tp->dev)
	#define napi_schedule(napi_ptr)			netif_rx_schedule(tp->dev)
	#define napi_complete(napi_ptr)			netif_rx_complete(tp->dev)
	#define netif_napi_del(napi_ptr)
	#define netif_napi_add(ndev, napi_ptr, function, weight_t) \
		ndev->poll = function; \
		ndev->weight = weight_t;
	typedef unsigned long				uintptr_t;
	#define DMA_BIT_MASK(value) \
		(value < 64 ? ((1ULL << value) - 1) : 0xFFFFFFFFFFFFFFFFULL)
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,23)
	#define NETIF_F_IPV6_CSUM			16
	#define cancel_delayed_work_sync		cancel_delayed_work
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,22)
	#define ip_hdr(skb_ptr)				(skb_ptr)->nh.iph
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,21)
	#define vlan_group_set_device(vlgrp, vid, value) \
		if (vlgrp) \
			vlgrp->vlan_devices[vid] = value;
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,20)
	#define delayed_work				work_struct
	#define INIT_DELAYED_WORK(a,b)			INIT_WORK(a,b,tp)
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)
	#define CHECKSUM_PARTIAL			CHECKSUM_HW
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,18)
	#define skb_is_gso(skb_ptr)			skb_shinfo(skb_ptr)->tso_size
	#define netdev_alloc_skb(dev, len)		dev_alloc_skb(len)
	#define IRQF_SHARED				SA_SHIRQ
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,16)
#ifndef __LINUX_MUTEX_H
	#define mutex					semaphore
	#define mutex_lock				down
	#define mutex_unlock				up
	#define mutex_lock_interruptible		down_interruptible
	#define mutex_init				init_MUTEX
#endif
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,14)
	#define ADVERTISED_Pause			(1 << 13)
	#define ADVERTISED_Asym_Pause			(1 << 14)
#endif /* LINUX_VERSION_CODE < KERNEL_VERSION(2,6,14) */
#endif /* LINUX_VERSION_CODE < KERNEL_VERSION(2,6,16) */
#endif /* LINUX_VERSION_CODE < KERNEL_VERSION(2,6,18) */
#endif /* LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19) */
#endif /* LINUX_VERSION_CODE < KERNEL_VERSION(2,6,20) */
#endif /* LINUX_VERSION_CODE < KERNEL_VERSION(2,6,21) */
#endif /* LINUX_VERSION_CODE < KERNEL_VERSION(2,6,22) */
#endif /* LINUX_VERSION_CODE < KERNEL_VERSION(2,6,23) */
#endif /* LINUX_VERSION_CODE < KERNEL_VERSION(2,6,24) */
#endif /* LINUX_VERSION_CODE < KERNEL_VERSION(2,6,32) */
#endif /* LINUX_VERSION_CODE < KERNEL_VERSION(2,6,34) */
#endif /* LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35) */
#endif /* LINUX_VERSION_CODE < KERNEL_VERSION(2,6,37) */

#ifndef FALSE
	#define TRUE	1
	#define FALSE	0
#endif

/*
 * inline function
 */

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,33)
static inline struct sk_buff *netdev_alloc_skb_ip_align(struct net_device *dev,
							unsigned int length)
{
	struct sk_buff *skb = netdev_alloc_skb(dev, length + NET_IP_ALIGN);

	if (NET_IP_ALIGN && skb)
		skb_reserve(skb, NET_IP_ALIGN);
	return skb;
}
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,22)
static inline void skb_copy_from_linear_data(const struct sk_buff *skb,
					     void *to,
					     const unsigned int len)
{
	memcpy(to, skb->data, len);
}
#endif /* LINUX_VERSION_CODE < KERNEL_VERSION(2,6,22) */
#endif /* LINUX_VERSION_CODE < KERNEL_VERSION(2,6,33) */

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27) && LINUX_VERSION_CODE > KERNEL_VERSION(2,6,23)
/**
 *  netif_napi_del - remove a napi context
 *  @napi: napi context
 *
 *  netif_napi_del() removes a napi context from the network device napi list
 */
static inline void netif_napi_del(struct napi_struct *napi)
{
#ifdef CONFIG_NETPOLL
        list_del(&napi->dev_list);
#endif
}
#endif /* LINUX_VERSION_CODE < KERNEL_VERSION(2,6,24) */

#endif /* LINUX_COMPATIBILITY_H */
