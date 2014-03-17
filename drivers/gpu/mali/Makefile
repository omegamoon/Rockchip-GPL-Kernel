# Configuration for A10

ifdef CONFIG_ARCH_SUN7I
CONFIG:=ca7-virtex820-m400-2
MALI_PLATFORM_FILE:=platform/${CONFIG}/mali_platform.c
export MALI_PLATFORM_FILE
else
ifdef CONFIG_PLAT_RK
CONFIG:=rockchip-rk3x
TARGET_PLATFORM:=rockchip
MALI_PLATFORM_FILE:=platform/${CONFIG}/mali_platform.c
export MALI_PLATFORM_FILE
else
# CONFIG:=ca8-virtex820-m400-1
# TARGET_PLATFORM:=mali400-pmu
CONFIG:=rockchip-rk3x
TARGET_PLATFORM:=rockchip
endif
endif

USING_MMU:=1
USING_UMP:=1
USING_PMM:=1
BUILD:=release
# TARGET_PLATFORM:=mali400-pmu
TARGET_PLATFORM:=rockchip
ifneq ($(CONFIG_TRACEPOINTS),y)
USING_PROFILING:=0
export USING_PROFILING
endif
export CONFIG USING_MMU USING_UMP USING_PMM BUILD TARGET_PLATFORM

obj-$(CONFIG_MALI) += ump/ mali/
