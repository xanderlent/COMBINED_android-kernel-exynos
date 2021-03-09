#
# Copyright (c) 2013 Samsung Electronics Co., Ltd.
#		http://www.samsung.com
#
# Licensed under GPLv2
#

obj-$(CONFIG_EXYNOS_DPP) += dpp.o
dpp-y := dpp_drv.o
obj-$(CONFIG_EXYNOS_MIPI_DSIM) += dsim.o
dsim-y += dsim_drv.o
obj-$(CONFIG_SOC_EXYNOS9110) += ./cal_9110/dsim_reg.o ./cal_9110/dpp_reg.o ./cal_9110/decon_reg.o
obj-$(CONFIG_DPHY_APB_CONTROL) += dphy_apb_drv.o
obj-$(CONFIG_EXYNOS_DPU30) += decon.o
obj-$(CONFIG_EXYNOS_BTS) += bts.o
decon-y += decon_core.o decon_dsi.o helper.o win_update.o decon_wb.o cursor.o fence.o event_log.o format.o
obj-y += panels/
