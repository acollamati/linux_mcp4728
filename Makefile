obj-m += mcp4728.o
PWD := $(CURDIR)

KERNEL_BUILD_DIR=/lib/modules/5.15.0-78-generic/build
CC=gcc

all:
	$(MAKE) -C $(KERNEL_BUILD_DIR) CC="$(CC)" M=$(PWD)  modules

clean:
	$(MAKE) -C $(KERNEL_BUILD_DIR) CC="$(CC)"  M=$(PWD)  clean
