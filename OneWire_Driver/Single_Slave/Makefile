ifneq ($(KERNELRELEASE),)
	obj-m := OW_Single_Slave_Driver.o
else
	KERNEL_DIR ?= /lib/modules/$(shell uname -r)/build
	PWD := $(shell pwd)
default:
	$(MAKE) -C ${KERNEL_DIR} M=$(PWD) modules
endif
