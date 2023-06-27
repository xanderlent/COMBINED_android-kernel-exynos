#
# Makefile for CS35L41 Speaker Smart Amplifier Driver.
#

default: all

KBUILD_OPTIONS := SOUND_ROOT=$(shell cd $(KERNEL_SRC); readlink -e $(M))
KBUILD_OPTIONS += CONFIG_SND_SOC_CS35L41=m
KBUILD_OPTIONS += CONFIG_SND_SOC_CS35L41_TABLE=y
KBUILD_OPTIONS += CONFIG_SND_SOC_CS35L41_I2C=y
KBUILD_OPTIONS += CONFIG_SND_SOC_CS35L41_ADSP=y

all:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) modules $(KBUILD_OPTIONS)

modules_install:
	$(MAKE) INSTALL_MOD_STRIP=1 M=$(M) -C $(KERNEL_SRC) modules_install

clean::
	rm -f *.o *.ko *.mod.c *.mod.o *~ .*.cmd Module.symvers
	rm -rf .tmp_versions
