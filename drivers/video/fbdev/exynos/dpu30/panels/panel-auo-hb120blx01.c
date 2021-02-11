// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2020 Google LLC
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <video/mipi_display.h>
#include "exynos_panel_drv.h"
#include "../dsim.h"

static int hb120blx01_suspend(struct exynos_panel_device *panel)
{
	struct dsim_device *dsim = get_dsim_drvdata(0);

	DPU_INFO_PANEL("%s +\n", __func__);
	mutex_lock(&panel->ops_lock);

	/* Display Off */
	dsim_write_data_seq(dsim, false, MIPI_DCS_SET_DISPLAY_OFF);

	/* Sleep In */
	dsim_write_data_seq(dsim, false, MIPI_DCS_ENTER_SLEEP_MODE);

	mutex_unlock(&panel->ops_lock);
	DPU_INFO_PANEL("%s -\n", __func__);
	return 0;
}

static int hb120blx01_displayon(struct exynos_panel_device *panel)
{
	struct dsim_device *dsim = get_dsim_drvdata(0);
	int ret;
	unsigned char buf[3];

	DPU_INFO_PANEL("%s +\n", __func__);
	mutex_lock(&panel->ops_lock);
	/* Page Select */
	dsim_write_data_seq(dsim, false, 0xfe, 0x01);
	/* Initialization sequence */
	dsim_write_data_seq(dsim, false, 0x0a, 0xf0);
	dsim_write_data_seq(dsim, false, 0xfe, 0x0a);
	dsim_write_data_seq(dsim, false, 0x29, 0x92);
	dsim_write_data_seq(dsim, false, 0xfe, 0x01);
	dsim_write_data_seq(dsim, false, 0x6d, 0x10);
	dsim_write_data_seq(dsim, false, 0xfe, 0x00);
	dsim_write_data_seq(dsim, false, MIPI_DCS_SET_COLUMN_ADDRESS,
			0x00, 0x02, 0x01, 0x87);
	dsim_write_data_seq(dsim, false, MIPI_DCS_SET_PAGE_ADDRESS,
			0x00, 0x00, 0x01, 0x85);
	dsim_write_data_seq(dsim, false, MIPI_DCS_SET_PARTIAL_AREA,
			0x00, 0x00, 0x01, 0x85);
	dsim_write_data_seq(dsim, false, 0x31, 0x00, 0x02, 0x01, 0x87);
	dsim_write_data_seq(dsim, false, MIPI_DCS_ENTER_PARTIAL_MODE);
	dsim_write_data_seq(dsim, false, MIPI_DCS_SET_TEAR_ON);
	dsim_write_data_seq(dsim, false, MIPI_DCS_WRITE_CONTROL_DISPLAY, 0x20);
	dsim_write_data_seq(dsim, false, MIPI_DCS_SET_PIXEL_FORMAT, 0x77);
	/* Sleep out , wait 70ms*/
	dsim_write_data_seq_delay(dsim, 150, MIPI_DCS_EXIT_SLEEP_MODE);

	/* Display on */
	dsim_write_data_seq_delay(dsim, 50, MIPI_DCS_SET_DISPLAY_ON);

	/* Power Mode read */
	ret = dsim_read_data(dsim, MIPI_DSI_DCS_READ,
			MIPI_DCS_GET_POWER_MODE, 3, buf);
	if (ret < 0) {
		dsim_err("Failed to read PM reg from panel\n");
	} else {
		dsim_info("=== Panel's PM Reg Value ===\n");
		dsim_info("* 0x0A : buf[0] = %x\n", buf[0]);
		dsim_info("* 0x0A : buf[1] = %x\n", buf[1]);
	}

	mutex_unlock(&panel->ops_lock);
	DPU_INFO_PANEL("%s -\n", __func__);
	return 0;
}

static int hb120blx01_doze(struct exynos_panel_device *panel)
{
	return 0;
}

static int hb120blx01_doze_suspend(struct exynos_panel_device *panel)
{
	return hb120blx01_doze(panel);
}

static int hb120blx01_set_light(struct exynos_panel_device *panel, u32 br_val)
{
	u8 data = 0;
	struct dsim_device *dsim = get_dsim_drvdata(panel->id);
	mutex_lock(&panel->ops_lock);
	/* WRDISBV(8bit): 1st DBV[7:0] */
	data = br_val & 0xFF;
	dsim_write_data_seq(dsim, false, 0x51, data);
	mutex_unlock(&panel->ops_lock);
	return 0;
}

struct exynos_panel_ops panel_hb120blx01_ops = {
	.id		= {0x2, 0xffffff, 0xffffff},
	.suspend	= hb120blx01_suspend,
	.displayon	= hb120blx01_displayon,
	.doze		= hb120blx01_doze,
	.doze_suspend	= hb120blx01_doze_suspend,
	.set_light	= hb120blx01_set_light,
};

