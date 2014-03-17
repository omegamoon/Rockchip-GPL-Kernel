#ifndef __PLAT_MEMORY_H
#define __PLAT_MEMORY_H

#include <linux/version.h>

/*
 * Physical DRAM offset.
 */
#define PLAT_PHYS_OFFSET	UL(0x60000000)

#define CONSISTENT_DMA_SIZE	SZ_8M

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 34))
#define dmac_clean_range(start, end)	dmac_map_area(start, end - start, DMA_TO_DEVICE)
#define dmac_inv_range(start, end)	dmac_unmap_area(start, end - start, DMA_FROM_DEVICE)
#endif

/* >>> Omegamoon - Taken from sunxi linux kernel
  define __phys_to_bus(x) (((x)>=PLAT_PHYS_OFFSET)?(x)-PLAT_PHYS_OFFSET:(x))
  define __bus_to_phys(x) (((x)<PLAT_PHYS_OFFSET)?(x)+PLAT_PHYS_OFFSET:(x))
  <<< Omegamoon
*/
#define __phys_to_bus(x) (x)
#define __bus_to_phys(x) (x)

#endif
