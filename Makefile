ifneq ($(KERNELVERSION),)
	ccflags-y += -Iinclude/drm -Wall
	obj-m := lx.o
	lx-y := lx_drv.o lx_i2c.o
else
	KERNELDIR ?= /lib/modules/$(shell uname -r)/build
	PWD := $(shell pwd)

3.0:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules

clean:
	$(RM) *.o *.ko *.mod.c Module.symvers .lx_* modules.order
	$(RM) -r .tmp_versions
endif
