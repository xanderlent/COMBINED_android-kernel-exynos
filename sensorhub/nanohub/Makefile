default: all

KBUILD_OPTIONS := CONFIG_NANOHUB=m
KBUILD_OPTIONS += CONFIG_NANOHUB_SPI=y

# Needed flags that aren't parsed in Kbuild
KBUILD_FLAGS := -DCONFIG_NANOHUB
KBUILD_FLAGS += -DCONFIG_NANOHUB_SPI

all:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) modules $(KBUILD_OPTIONS) KCFLAGS='$(KBUILD_FLAGS)'

modules_install:
	$(MAKE) INSTALL_MOD_STRIP=1 M=$(M) -C $(KERNEL_SRC) modules_install

clean::
	rm -f *.o *.ko *.mod.c *.mod.o *~ .*.cmd Module.symvers
	rm -rf .tmp_versions
