#ifndef __PIXEL_WATCH_NANOHUB_NANOHUB_H
#define __PIXEL_WATCH_NANOHUB_NANOHUB_H
#include <linux/types.h>
struct nanohub_flash_bank {
	int bank;
	u32 address;
	size_t length;
};
struct nanohub_platform_data {
	u32 wakeup_gpio;
	u32 nreset_gpio;
	u32 boot0_gpio;
	u32 irq1_gpio;
	u32 irq2_gpio;
	u32 spi_cs_gpio;
#ifdef CONFIG_PIXEL_WATCH_NANOHUB_BL_ST
	u32 bl_max_speed_hz;
	u32 bl_addr;
	u32 num_flash_banks;
	struct nanohub_flash_bank *flash_banks;
	u32 num_shared_flash_banks;
	struct nanohub_flash_bank *shared_flash_banks;
#endif
};
#endif /* __PIXEL_WATCH_NANOHUB_NANOHUB_H */
