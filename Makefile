obj-m += mcp4728.o
PWD := $(CURDIR)

# Yocto SDK Toolchain cross compile for arm (kirkstone) has a bug that does not allow to
# to build out of tree modules.
# I used the compiler and kernel source inside temporary directories.
KERNEL_BUILD_DIR=/home/colla/ext_usb_disk/yocto/toradex/build/tmp/work/colibri_imx7_emmc-tdx-linux-gnueabi/linux-toradex-mainline-rt/6.1.26-rt8+gitAUTOINC+48e5445f07-r0/build/
CC=/home/colla/ext_usb_disk/yocto/toradex/build/tmp/work/colibri_imx7_emmc-tdx-linux-gnueabi/linux-toradex-mainline-rt/6.1.26-rt8+gitAUTOINC+48e5445f07-r0/recipe-sysroot-native//usr/bin/arm-tdx-linux-gnueabi/arm-tdx-linux-gnueabi-gcc

all:
	$(MAKE) -C $(KERNEL_BUILD_DIR) CC="$(CC)" M=$(PWD) ARCH=arm modules

clean:
	$(MAKE) -C $(KERNEL_BUILD_DIR) CC="$(CC)"  M=$(PWD) ARCH=arm clean
