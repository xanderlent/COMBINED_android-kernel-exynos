#
# Makefile for ASOC MCU Mic codec driver.
#

default: all

KBUILD_OPTIONS := CONFIG_SND_SOC_MCU_MIC_CODEC=m

all:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) modules $(KBUILD_OPTIONS)

modules_install:
	$(MAKE) INSTALL_MOD_STRIP=1 M=$(M) -C $(KERNEL_SRC) modules_install

clean::
	rm -f *.o *.ko *.mod.c *.mod.o *~ .*.cmd Module.symvers
	rm -rf .tmp_versions
