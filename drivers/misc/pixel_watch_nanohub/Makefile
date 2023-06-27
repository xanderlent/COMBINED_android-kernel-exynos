#
# Makefile for nanohub
#

obj-$(CONFIG_PIXEL_WATCH_NANOHUB) += nanohub.o
nanohub-y := main.o comms.o
nanohub-$(CONFIG_PIXEL_WATCH_NANOHUB_SPI) += spi.o
nanohub-$(CONFIG_PIXEL_WATCH_NANOHUB_BL_NXP) += bl_nxp.o
nanohub-$(CONFIG_PIXEL_WATCH_NANOHUB_BL_ST) += bl_st.o
