#
# Makefile for CS40L2X Haptics Driver
#

default: all

KBUILD_OPTIONS += CONFIG_SND_SOC_CS40L25=m
KBUILD_OPTIONS += CONFIG_SND_SOC_CS40L25_TABLE=y

all:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) modules $(KBUILD_OPTIONS)

modules_install:
	$(MAKE) INSTALL_MOD_STRIP=1 M=$(M) -C $(KERNEL_SRC) modules_install

clean::
	rm -f *.o *.ko *.mod.c *.mod.o *~ .*.cmd Module.symvers
	rm -rf .tmp_versions
