
# ----------------------------------------
# Example how to cross compile the module:
# ----------------------------------------
#KERNELDIR=/home/hico/openwrt/8.09/build_dir/linux-ixp4xx_generic/linux-2.6.26.6/
#ARCH=arm
#CROSS_COMPILE=armeb-linux-


HICO_MODNAME=hcanpci

FW_UPDATE ?= 0
IRQ_TRACE ?= 0

ifneq ($(ARCH),)
    EXTRA_FLAGS+=ARCH=$(ARCH)
endif

ifneq ($(CROSS_COMPILE),)
    EXTRA_FLAGS+=CROSS_COMPILE=$(CROSS_COMPILE)
endif

ifneq ($(KERNELRELEASE),)
  obj-m := $(HICO_MODNAME).o
else
  KERNELDIR ?= /lib/modules/$(shell uname -r)/build
  PWD := $(shell pwd)

default: version
	$(MAKE) -C $(KERNELDIR) M=$(PWD) $(EXTRA_FLAGS) modules

clean:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) $(EXTRA_FLAGS) clean

version:
	@if [ -e .svn ]; then \
	    VERSION=`svnversion . | grep '^[0-9]\+$$'` ;\
	    if [ $$? -ne 0 ]; then \
		echo "Warning: Could not get a clean version!!">&2 ;\
		VERSION=0 ;\
	    fi ;\
	    echo "#define HCANPCI_DRIVER_VERSION $$VERSION" > version.h ;\
	fi


# Remove old  module if its in the kernel. after that, inster the module and
# make the device nodes
install:
	rmmod $(HICO_MODNAME) 2>/dev/null; \
	insmod $(HICO_MODNAME).ko irqtrace=$(IRQ_TRACE) fw_update=$(FW_UPDATE) && \
	./makenodes.sh

endif
