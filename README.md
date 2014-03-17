Linux kernel for Rockchip SOCs - www.omegamoon.com
--------------

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
2014-03-17:
- Added basic RK3066 MK808B support (needs testing)
2014-03-10:
- Sound fixed on RK1000 (Radxa)
- USB Hub fixed (Radxa)
- Fixed networking
2014-03-08:
- Fixed 1080p resolution
2014-02-14:
- Backporting DRM from sunxi (v3.4.67) linux 
- Backporting Mali from sunxi (v3.4.67) linux 
- Merging in tablet video drivers from Tronsmart KitKat kernel
- Added Mali support
- Added rk30-ipp 1.003 driver
2013-12-08:
- Upgraded to Linux kernel version 3.0.101+
- Added Asus BT400 Bluetooth 4.0 support
- Added Radxa Rock support
- Added Minix Neo X7 support
- Added various configs for booting linux from SD
2013-09-27:
- Upgraded to Linux kernel version 3.0.72+
- Merged with rockchip git
- Tested with Kingnovel K-R42 and Tronsmart MK908

- Based on the 'netxeon' Linux kernel version 3.0.36+
- Added linaro 4.8.2 toolchain
- Added build scripts
- Added linux flash tools + flash script for mk908 (use at own risk!)
- Minor config changes

