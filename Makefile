dtbo-y += r11_dev2_overlay.dtbo
dtbo-y += r11_dev2_no_nfc_overlay.dtbo
dtbo-y += r11_dev2_boe_overlay.dtbo
dtbo-y += r11_dev2_no_nfc_boe_overlay.dtbo
dtbo-y += r11_dev2_btwifi_overlay.dtbo
dtbo-y += r11_proto1_overlay.dtbo
dtbo-y += r11_proto1_btwifi_overlay.dtbo
dtbo-y += r11_proto11_no_nfc_overlay.dtbo

always		:= $(dtb-y) $(dtbo-y)
subdir-y	:= $(dts-dirs)
clean-files	:= *.dtb *.dtbo
