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

static int wf012fbm_suspend(struct exynos_panel_device *panel)
{
	struct dsim_device *dsim = get_dsim_drvdata(0);

	DPU_INFO_PANEL("%s +\n", __func__);
	mutex_lock(&panel->ops_lock);

	/* Page Select */
	dsim_write_data_seq(dsim, false, 0xff, 0x10);

	/* Display Off */
	dsim_write_data_seq(dsim, false, MIPI_DCS_SET_DISPLAY_OFF);

	/* Sleep In */
	dsim_write_data_seq(dsim, false, MIPI_DCS_ENTER_SLEEP_MODE);

	mutex_unlock(&panel->ops_lock);
	DPU_INFO_PANEL("%s -\n", __func__);
	return 0;
}

static int wf012fbm_displayon(struct exynos_panel_device *panel)
{
	struct dsim_device *dsim = get_dsim_drvdata(0);
	int ret;
	unsigned char buf[3];

	DPU_INFO_PANEL("%s +\n", __func__);
	mutex_lock(&panel->ops_lock);
	/* Page Select */
	dsim_write_data_seq(dsim, false, 0xff, 0x10);

	/* Sleep out , wait 70ms*/
	dsim_write_data_seq_delay(dsim, 70, MIPI_DCS_EXIT_SLEEP_MODE);

	/* Display on */
	dsim_write_data_seq(dsim, false, MIPI_DCS_SET_DISPLAY_ON);

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

	/* Exit Idle Mode */
	dsim_write_data_seq(dsim, false, MIPI_DCS_EXIT_IDLE_MODE);

	mutex_unlock(&panel->ops_lock);
	DPU_INFO_PANEL("%s -\n", __func__);
	return 0;
}

static int wf012fbm_doze(struct exynos_panel_device *panel)
{
	struct dsim_device *dsim = get_dsim_drvdata(0);

	DPU_INFO_PANEL("%s +\n", __func__);
	mutex_lock(&panel->ops_lock);
	/* Page select */
	dsim_write_data_seq(dsim, false, 0xff, 0x10);
	/* Idle Mode */
	dsim_write_data_seq(dsim, false, MIPI_DCS_ENTER_IDLE_MODE);
	mutex_unlock(&panel->ops_lock);
	DPU_INFO_PANEL("%s -\n", __func__);
	return 0;
}

static int wf012fbm_doze_suspend(struct exynos_panel_device *panel)
{
	return wf012fbm_doze(panel);
}

struct exynos_panel_ops panel_wf012fbm_ops = {
	.id		= {0x1, 0xffffff, 0xffffff},
	.suspend	= wf012fbm_suspend,
	.displayon	= wf012fbm_displayon,
	.doze		= wf012fbm_doze,
	.doze_suspend	= wf012fbm_doze_suspend,
};

