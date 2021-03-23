# SPDX-License-Identifier: GPL-2.0
dts-dirs += exynos9110

dtb-$(CONFIG_ARCH_EXYNOS) += \
	exynos5433-tm2.dtb	\
	exynos5433-tm2e.dtb	\
	exynos7-espresso.dtb
dtb-$(CONFIG_SOC_EXYNOS9110) += exynos9110.dtb
dtb-$(CONFIG_SOC_EXYNOS9110) += r11.dtb

always		:= $(dtb-y) $(dtbo-y)
subdir-y	:= $(dts-dirs)
clean-files	:= *.dtb *.dtbo
