dtb-y += r11_dev1_combined.dtb
dtb-y += r11_proto1_combined.dtb
dtbo-y += r11_dev1_overlay.dtbo
dtbo-y += r11_proto1_overlay.dtbo

always		:= $(dtb-y) $(dtbo-y)
subdir-y	:= $(dts-dirs)
clean-files	:= *.dtb *.dtbo
