Linux kernel for Rockchip SOCs - www.omegamoon.com
--------------
QX1 work on progress (not ended yet)
**Build instructions:**
- Run "make mrproper" to make sure you have no stale .o files and dependencies lying around
- Run "./build_omegamoon_mk908" to compile the kernel for the MK908 using the prebuild toolchain
  
**Flash instructions for the MK908 device using Linux (use at own risk!):**
- Connect one end of the USB cable to your PC
- Press the reset button using a paperclip, and while pressed, connect the USB cable to the OTG USB port
- Release the reset button
- Run "./flash_omegamoon_kernel" to flash the kernel to the device
- When ready, the device will be rebooted automatically

**Revision history:**
2013-09-27:
- Upgraded to Linux kernel version 3.0.72+
- Merged with rockchip git
- Tested with Kingnovel K-R42 and Tronsmart MK908

- Based on the 'netxeon' Linux kernel version 3.0.36+
- Added linaro 4.8.2 toolchain
- Added build scripts
- Added linux flash tools + flash script for mk908 (use at own risk!)
- Minor config changes

