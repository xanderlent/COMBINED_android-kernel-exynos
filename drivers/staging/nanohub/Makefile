#
# Makefile for nanohub
#

obj-$(CONFIG_NANOHUB) += nanohub.o
nanohub-$(CONFIG_CHRE_SENSORHUB_HAL) := main.o comms.o
nanohub-$(CONFIG_NANOHUB_MAILBOX) += chub.o chub_dbg.o chub_exynos.o ipc_chub.o ipc_chub_ap.o ipc_common.o ipc_hw.o chub_log.o
