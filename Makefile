dtbo-y += r11_evt1_overlay.dtbo
dtbo-y += r11_evt1_btwifi_overlay.dtbo
dtbo-y += r11_evt11_overlay.dtbo
dtbo-y += r11_evt11_btwifi_overlay.dtbo

always		:= $(dtb-y) $(dtbo-y)
subdir-y	:= $(dts-dirs)
clean-files	:= *.dtb *.dtbo
