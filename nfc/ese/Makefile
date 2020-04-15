#
# Makefile for ST54J NFC eSE driver.
#

default: all

KBUILD_OPTIONS := CONFIG_ESE_ST54J=m

all:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) modules $(KBUILD_OPTIONS)

modules_install:
	$(MAKE) INSTALL_MOD_STRIP=1 M=$(M) -C $(KERNEL_SRC) modules_install

clean::
	rm -f *.o *.ko *.mod.c *.mod.o *~ .*.cmd Module.symvers
	rm -rf .tmp_versions