/*
 * Copyright (C) 2010-2012 ARM Limited. All rights reserved.
 * 
 * This program is free software and is provided to you under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation, and any use by you of this program is subject to the terms of such GNU licence.
 * 
 * A copy of the licence is included with the program, and can also be obtained from Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#ifndef __ARCH_CONFIG_H__
#define __ARCH_CONFIG_H__

/* Configuration for Rockchip RK3x */
//#include "mach/irqs.h"
//#include "mach/io.h"

// Omegamoon >> Taken from /arch/arm/mach-rk3188/include/mach/irqs.h
#define IRQ_GPU_PP                      39
#define IRQ_GPU_MMU                     40
#define IRQ_GPU_GP                      44

#define SW_INT_IRQNO_GPU_GP             (IRQ_GPU_GP)
#define SW_INT_IRQNO_GPU_GPMMU          (IRQ_GPU_MMU)
#define SW_INT_IRQNO_GPU_PP0            (IRQ_GPU_PP)
#define SW_INT_IRQNO_GPU_PPMMU0         (IRQ_GPU_MMU)
#define SW_INT_IRQNO_GPU_PMU            (0)
#define SW_INT_IRQNO_GPU_PP1            (IRQ_GPU_PP)
#define SW_INT_IRQNO_GPU_PPMMU1         (IRQ_GPU_MMU)

static _mali_osk_resource_t arch_configuration [] =
{
/*
	{
		.type = PMU,
		.description = "Mali-400 PMU",
		.base = RK30_GPU_PHYS + 0x2000,
		.irq = SW_INT_IRQNO_GPU_PMU,
		.mmu_id = 0
	},
*/
	{
		.type = MALI400GP,
		.description = "Mali-400 GP",
		.base = RK30_GPU_PHYS,
		.irq = SW_INT_IRQNO_GPU_GP,
		.mmu_id = 1
	},
	{
		.type = MALI400PP,
		.base = RK30_GPU_PHYS + 0x8000,
		.irq = SW_INT_IRQNO_GPU_PP0,
		.description = "Mali-400 PP0",
		.mmu_id = 2
	},
	{
		.type = MALI400PP,
		.base = RK30_GPU_PHYS + 0xA000,
		.irq = SW_INT_IRQNO_GPU_PP1,
		.description = "Mali-400 PP1",
		.mmu_id = 3
	},
	{
		.type = MMU,
		.base = RK30_GPU_PHYS + 0x3000,
		.irq = SW_INT_IRQNO_GPU_GPMMU,
		.description = "Mali-400 MMU for GP",
		.mmu_id = 1
	},
	{
		.type = MMU,
		.base = RK30_GPU_PHYS + 0x4000,
		.irq = SW_INT_IRQNO_GPU_PPMMU0,
		.description = "Mali-400 MMU for PP0",
		.mmu_id = 2
	},
	{
		.type = MMU,
		.base = RK30_GPU_PHYS + 0x5000,
		.irq = SW_INT_IRQNO_GPU_PPMMU1,
		.description = "Mali-400 MMU for PP1",
		.mmu_id = 3
	},
	{
		.type = OS_MEMORY,
		.description = "OS Memory",
//		.cpu_usage_adjust = 0x40000000,
		.alloc_order = 0, /* Highest preference for this memory */
		.size = 192 * 1024 * 1024, /* 64 MB */
		.flags = _MALI_CPU_WRITEABLE | _MALI_CPU_READABLE | _MALI_MMU_READABLE | _MALI_MMU_WRITEABLE | _MALI_GP_READABLE | _MALI_GP_WRITEABLE
	},
	{
		.type = MALI400L2,
		.base = RK30_GPU_PHYS + 0x1000,
		.description = "Mali-400 L2 cache"
	},
};

#endif /* __ARCH_CONFIG_H__ */
