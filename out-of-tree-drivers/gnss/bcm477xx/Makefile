#
# Makefile for BCM477xx driver.
#

default: all

KBUILD_OPTIONS := GPS_ROOT=$(shell cd $(KERNEL_SRC); readlink -e $(M))
KBUILD_OPTIONS += CONFIG_GPS_BCM477XX=m
KBUILD_OPTIONS += CONFIG_GPS_BCM_TTY=n
KBUILD_OPTIONS += CONFIG_GPS_BCM_I2C=n
KBUILD_OPTIONS += CONFIG_BCM_GPS_SPI_DRIVER=y
KBUILD_OPTIONS += CONFIG_GPS_BCM_HOSTWAKE=n
KBUILD_OPTIONS += CONFIG_GPS_BCM_SENSORS_SSP=y

all:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) modules $(KBUILD_OPTIONS)
modules_install:
	$(MAKE) INSTALL_MOD_STRIP=1 M=$(M) -C $(KERNEL_SRC) modules_install

clean::
	rm -f *.o *.ko *.mod.c *.mod.o *~ .*.cmd Module.symvers
	rm -rf .tmp_versions
