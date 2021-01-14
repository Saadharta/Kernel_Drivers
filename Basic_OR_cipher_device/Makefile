ifneq ($(KERNELRELEASE),)
	obj-m := cipher_driver.o
else
	KERNEL_DIR ?= /lib/modules/$(shell uname -r)/build
	PWD := $(shell pwd)
default:
	$(MAKE) -Wall -Werror -C ${KERNEL_DIR} M=$(PWD) modules
endif
