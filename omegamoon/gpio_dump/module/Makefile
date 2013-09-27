#############################################################################
#
#	Makefile for building the gpio kernel module
#
#############################################################################

#ifneq ($(KERNELRELEASE),)
# call from kernel build system

obj-m	:= user-gpio-drv.o

#else

PWD       	= $(shell pwd)

OVEROTOP	?= /home/gumstix/overo-oe
#CROSS_COMPILE	?= $(OVEROTOP)/tmp/cross/armv7a/bin/arm-angstrom-linux-gnueabi-
CROSS_COMPILE	?= /usr/bin/arm-linux-gnueabi-
KERNEL_PATH     ?= /home/fede/ANDROID/rockchip-rk30xx-mk808
ARCH		?= arm

export ARCH CROSS_COMPILE

KBUILD_FLAGS 	= -C $(KERNEL_PATH) M=$(PWD)

default: modules

modules:
	echo $(MAKE) $(KBUILD_FLAGS) modules
	$(MAKE) $(KBUILD_FLAGS) modules

clean:
	rm -rf *.o *~ *.ko *.mod.c .*.cmd .tmp* modules.order Module.symvers
#endif

