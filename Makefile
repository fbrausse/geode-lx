ifneq ($(KERNELVERSION),)
	ccflags-y += -Iinclude/drm -Wall
	obj-m := lx.o
	lx-y := lx_drv.o lx_i2c.o
else
#	KERNELDIR ?= $(wildcard /home/kane/bin/linux-3.0)
#	KERNELDIR ?= /lib/modules/$(shell uname -r)/build
	KERNELDIR ?= $(if $(wildcard /home/kane/bin/linux-3.0),/home/kane/bin/linux-3.0,/lib/modules/$(shell uname -r)/build)
	PWD := $(shell pwd)

3.0:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules

clean:
	$(RM) *.o *.ko *.mod.c Module.symvers .lx* modules.order
	$(RM) -r .tmp_versions
endif
