default: all

KBUILD_OPTIONS := CONFIG_TOUCHSCREEN_RAYDIUM=m

# TODO: Enable this option once the right firmware has
# been identified and integrated b/159821513
# KBUILD_FLAGS := -DTOUCHSCREEN_RAYDIUM_UPDATE_FW

all:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) modules $(KBUILD_OPTIONS) KCFLAGS='$(KBUILD_FLAGS)'

modules_install:
	$(MAKE) INSTALL_MOD_STRIP=1 M=$(M) -C $(KERNEL_SRC) modules_install

clean::
	rm -f *.o *.ko *.mod.c *.mod.o *~ .*.cmd Module.symvers
	rm -rf .tmp_versions
