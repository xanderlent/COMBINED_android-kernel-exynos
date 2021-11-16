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
#include "../decon.h"

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
	dsim_write_data_seq_delay(dsim, 70, MIPI_DCS_ENTER_SLEEP_MODE);

	mutex_unlock(&panel->ops_lock);
	DPU_INFO_PANEL("%s -\n", __func__);
	return 0;
}

static int wf012fbm_displayon(struct exynos_panel_device *panel)
{
	struct dsim_device *dsim = get_dsim_drvdata(0);
	int ret;
	unsigned char buf[1];

	DPU_INFO_PANEL("%s +\n", __func__);
	mutex_lock(&panel->ops_lock);
	ret = dsim_read_data(dsim, MIPI_DSI_DCS_READ,
			MIPI_DCS_GET_POWER_MODE, sizeof(buf), buf);
	if (ret < 0) {
		dsim_err("Failed to read PM reg from panel\n");
	} else {
		dsim_info("=== Panel's PM Reg Value ===\n");
		dsim_info("* 0x0A : buf[0] = %x\n", buf[0]);
	}
	if (buf[0] != 0x08) {
		dsim_info("Detected unexpected panel status after reset, trying again\n");
		if (decon_reg_get_run_status(dsim->id)) {
			dsim_reset_panel(dsim);
			dpu_hw_recovery_process(get_decon_drvdata(dsim->id));
		} else {
			dsim_reset_panel(dsim);
			dsim_reg_recovery_process(dsim);
		}
		ret = dsim_read_data(dsim, MIPI_DSI_DCS_READ,
				MIPI_DCS_GET_POWER_MODE, sizeof(buf), buf);
		if (ret < 0) {
			dsim_err("Failed to read PM reg from panel (second try)\n");
		} else {
			dsim_info("=== Panel's PM Reg Value ===\n");
			dsim_info("* 0x0A : buf[0] = %x\n", buf[0]);
		}
	}

	/* Page Select */
	dsim_write_data_seq(dsim, false, 0xff, 0x10);

	/* Sleep out , wait 15ms*/
	dsim_write_data_seq_delay(dsim, 15, MIPI_DCS_EXIT_SLEEP_MODE);

	/* Set display to 30Hz, wait the rest of the 70ms after sleep out */
	/* These values were provided from the vendor. Page select: */
	dsim_write_data_seq(dsim, false, 0xff, 0x10);
	/* Write 0x81 to register 0x6F to set 30Hz */
	dsim_write_data_seq(dsim, false, 0x6F, 0x81);

	/* Write sequence for setting the PWM frequency if necessary*/
	ret = dsim_read_data(dsim, MIPI_DSI_DCS_READ,
			0xDA, sizeof(buf), buf);
	if (ret < 0) {
		dsim_err("Failed to read display type\n");
	} else {
		dsim_info("=== Panel's ID Reg Value ===\n");
		dsim_info("* 0xDA : buf[0] = %x\n", buf[0]);
		if (buf[0] == 0x1F || buf[0] == 0x2F) {
			dsim_info("Detect EVT display, sending PWM frequency settings\n");
			/* Page select 0x21 */
			dsim_write_data_seq(dsim, false, 0xff, 0x21);
			/* set PWM Freq.to 240Hz */
			dsim_write_data_seq(dsim, false, 0xfb, 0x01);
			dsim_write_data_seq(dsim, false, 0x8c, 0x22);
			/* set PWM duty @ HBM */
			dsim_write_data_seq(dsim, false, 0x7c, 0x00);
			dsim_write_data_seq(dsim, false, 0x7d, 0xcc);
			dsim_write_data_seq(dsim, false, 0x7e, 0x00);
			dsim_write_data_seq(dsim, false, 0x7f, 0x94);
			dsim_write_data_seq(dsim, false, 0x80, 0x00);
			dsim_write_data_seq(dsim, false, 0x81, 0x60);
			/* set PWM duty @ AOD */
			dsim_write_data_seq(dsim, false, 0x84, 0x01);
			dsim_write_data_seq(dsim, false, 0x85, 0x90);
			dsim_write_data_seq(dsim, false, 0x86, 0x01);
			dsim_write_data_seq(dsim, false, 0x87, 0x20);
			dsim_write_data_seq(dsim, false, 0x88, 0x00);
			dsim_write_data_seq(dsim, false, 0x89, 0xb0);
			/* Gamma reg settings for HBM */
			dsim_write_data_seq(dsim, false, 0xdd,
					0x0B, 0x00, 0x0B, 0xDF,
					0x0D, 0x2D, 0x00, 0x00,
					0x00, 0x00, 0x00, 0x00,
					0x00, 0x00, 0x00, 0x00);
			dsim_write_data_seq(dsim, false, 0xdc,
					0x06, 0xB0, 0x07, 0x43,
					0x07, 0xD1, 0x08, 0x63,
					0x08, 0xD8, 0x09, 0x58,
					0x09, 0xD6, 0x0A, 0x6A);
			dsim_write_data_seq(dsim, false, 0xdb,
					0x03, 0x72, 0x03, 0xB7,
					0x03, 0xF2, 0x04, 0x2F,
					0x04, 0x9F, 0x05, 0x08,
					0x05, 0x69, 0x06, 0x12);
			dsim_write_data_seq(dsim, false, 0xda,
					0x00, 0x01, 0x00, 0x88,
					0x00, 0xB1, 0x01, 0x5F,
					0x01, 0xEA, 0x02, 0x65,
					0x02, 0xC3, 0x03, 0x19);
			dsim_write_data_seq(dsim, false, 0xe1,
					0x0A, 0x7A, 0x0A, 0xEF,
					0x0B, 0x7E, 0x00, 0x00,
					0x00, 0x00, 0x00, 0x00,
					0x00, 0x00, 0x00, 0x00);
			dsim_write_data_seq(dsim, false, 0xe0,
					0x06, 0x7D, 0x07, 0x0A,
					0x07, 0x87, 0x08, 0x0B,
					0x08, 0x8D, 0x09, 0x08,
					0x09, 0x82, 0x09, 0xF9);
			dsim_write_data_seq(dsim, false, 0xdf,
					0x03, 0x4D, 0x03, 0x92,
					0x03, 0xD3, 0x04, 0x11,
					0x04, 0x7C, 0x04, 0xDE,
					0x05, 0x41, 0x05, 0xE1);
			dsim_write_data_seq(dsim, false, 0xde,
					0x00, 0x01, 0x00, 0xE9,
					0x01, 0x18, 0x01, 0x88,
					0x01, 0xF5, 0x02, 0x54,
					0x02, 0xB4, 0x03, 0x04);
			dsim_write_data_seq(dsim, false, 0xe5,
					0x0D, 0x6C, 0x0E, 0x32,
					0x0F, 0x4E, 0x00, 0x00,
					0x00, 0x00, 0x00, 0x00,
					0x00, 0x00, 0x00, 0x00);
			dsim_write_data_seq(dsim, false, 0xe4,
					0x08, 0x21, 0x08, 0xD2,
					0x09, 0x77, 0x0A, 0x1B,
					0x0A, 0xBF, 0x0B, 0x60,
					0x0B, 0xFE, 0x0C, 0xAC);
			dsim_write_data_seq(dsim, false, 0xe3,
					0x04, 0x4B, 0x04, 0xA2,
					0x04, 0xF0, 0x05, 0x37,
					0x05, 0xB9, 0x06, 0x2F,
					0x06, 0xA1, 0x07, 0x66);
			dsim_write_data_seq(dsim, false, 0xe2,
					0x00, 0x01, 0x00, 0xE9,
					0x01, 0x2C, 0x01, 0xEE,
					0x02, 0x96, 0x03, 0x14,
					0x03, 0x8E, 0x03, 0xF4);
			/* Gamma reg settings for AOD */
			dsim_write_data_seq(dsim, false, 0xeb,
					0x08, 0xD3, 0x09, 0x2F,
					0x09, 0x7E, 0x00, 0x00,
					0x00, 0x00, 0x00, 0x00,
					0x00, 0x00, 0x00, 0x00);
			dsim_write_data_seq(dsim, false, 0xea,
					0x05, 0xFE, 0x06, 0x6A,
					0x06, 0xC7, 0x07, 0x14,
					0x07, 0x73, 0x07, 0xCC,
					0x08, 0x21, 0x08, 0x77);
			dsim_write_data_seq(dsim, false, 0xe9,
					0x03, 0x87, 0x03, 0xB6,
					0x03, 0xEC, 0x04, 0x19,
					0x04, 0x7A, 0x04, 0xC9,
					0x05, 0x0B, 0x05, 0x8A);
			dsim_write_data_seq(dsim, false, 0xe8,
					0x00, 0x01, 0x01, 0x72,
					0x01, 0x73, 0x01, 0xA0,
					0x02, 0x3B, 0x02, 0xA6,
					0x02, 0xF8, 0x03, 0x47);
			dsim_write_data_seq(dsim, false, 0xef,
					0x08, 0x90, 0x08, 0xEB,
					0x09, 0x47, 0x00, 0x00,
					0x00, 0x00, 0x00, 0x00,
					0x00, 0x00, 0x00, 0x00);
			dsim_write_data_seq(dsim, false, 0xee,
					0x05, 0xD5, 0x06, 0x3B,
					0x06, 0x8D, 0x06, 0xF5,
					0x07, 0x48, 0x07, 0x94,
					0x07, 0xF4, 0x08, 0x46);
			dsim_write_data_seq(dsim, false, 0xed,
					0x03, 0x8A, 0x03, 0xB8,
					0x03, 0xE9, 0x04, 0x14,
					0x04, 0x58, 0x04, 0xA5,
					0x04, 0xEF, 0x05, 0x60);
			dsim_write_data_seq(dsim, false, 0xec,
					0x00, 0x01, 0x02, 0x15,
					0x02, 0x2F, 0x02, 0x6C,
					0x02, 0xA5, 0x02, 0xDE,
					0x03, 0x1F, 0x03, 0x55);
			dsim_write_data_seq(dsim, false, 0xf3,
					0x0A, 0x3E, 0x0A, 0xA8,
					0x0B, 0x24, 0x00, 0x00,
					0x00, 0x00, 0x00, 0x00,
					0x00, 0x00, 0x00, 0x00);
			dsim_write_data_seq(dsim, false, 0xf2,
					0x07, 0x04, 0x07, 0x7E,
					0x07, 0xE3, 0x08, 0x53,
					0x08, 0xBD, 0x09, 0x12,
					0x09, 0x85, 0x09, 0xE5);
			dsim_write_data_seq(dsim, false, 0xf1,
					0x04, 0x3B, 0x04, 0x77,
					0x04, 0xB5, 0x04, 0xE8,
					0x05, 0x40, 0x05, 0x9A,
					0x05, 0xF5, 0x06, 0x7C);
			dsim_write_data_seq(dsim, false, 0xf0,
					0x00, 0x01, 0x02, 0x15,
					0x02, 0x1F, 0x02, 0x6C,
					0x02, 0xF0, 0x03, 0x47,
					0x03, 0xAF, 0x03, 0xF9);
		}
	}

	/* Set for brightness ramp/dim timing */
	// Set for CMD2 page1 parameters
	dsim_write_data_seq(dsim, false, 0xff, 0x21);
	// No reload MTP
	dsim_write_data_seq(dsim, false, 0xfb, 0x01);
	// Ramp/dim timing: DIM_OTP = 1, step = 18
	// -> ramp/dim duration = 18 frames (300ms @60Hz, 600ms @30Hz)
	dsim_write_data_seq(dsim, false, 0x57, 0x92);
	dsim_write_data_seq(dsim, false, 0x58, 0x12);

	/* Page Select */
	dsim_write_data_seq_delay(dsim, 55, 0xff, 0x10);

	/* Display on */
	dsim_write_data_seq(dsim, false, MIPI_DCS_SET_DISPLAY_ON);

	/* Power Mode read */
	ret = dsim_read_data(dsim, MIPI_DSI_DCS_READ,
			MIPI_DCS_GET_POWER_MODE, sizeof(buf), buf);
	if (ret < 0) {
		dsim_err("Failed to read PM reg from panel\n");
	} else {
		dsim_info("=== Panel's PM Reg Value ===\n");
		dsim_info("* 0x0A : buf[0] = %x\n", buf[0]);
	}
	/* Self Det read */
	// For debugging display crash issue
	ret = dsim_read_data(dsim, MIPI_DSI_DCS_READ,
			0xDD, sizeof(buf), buf);
	if (ret < 0) {
		dsim_err("Failed to read SELF_DET reg from panel\n");
	} else {
		dsim_info("=== Panel's SELF_DET Reg Value ===\n");
		dsim_info("* 0xDD : buf[0] = %x\n", buf[0]);
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

static int wf012fbm_set_light(struct exynos_panel_device *panel, u32 br_val)
{
	u8 data;
	struct dsim_device *dsim = get_dsim_drvdata(0);
	mutex_lock(&panel->ops_lock);
	/* WRDISBV(8bit): 1st DBV[7:0] */
	data = br_val & 0xFF;
	dsim_write_data_seq(dsim, false, 0x51, data);
	mutex_unlock(&panel->ops_lock);
	return 0;
}

static int wf012fbm_enter_hbm(struct exynos_panel_device *panel)
{
	struct dsim_device *dsim = get_dsim_drvdata(0);
	DPU_INFO_PANEL("%s +\n", __func__);
	mutex_lock(&panel->ops_lock);
	/* Page select */
	dsim_write_data_seq(dsim, false, 0xff, 0x10);
	/* Enter High Brightness Mode */
	dsim_write_data_seq(dsim, false, 0x66, 0x02);

	mutex_unlock(&panel->ops_lock);
	DPU_INFO_PANEL("%s -\n", __func__);
	return 0;
}

static int wf012fbm_exit_hbm(struct exynos_panel_device *panel)
{
	struct dsim_device *dsim = get_dsim_drvdata(0);
	DPU_INFO_PANEL("%s +\n", __func__);
	mutex_lock(&panel->ops_lock);
	/* Page select */
	dsim_write_data_seq(dsim, false, 0xff, 0x10);
	/* Exit High Brightness Mode */
	dsim_write_data_seq(dsim, false, 0x66, 0x00);

	mutex_unlock(&panel->ops_lock);
	DPU_INFO_PANEL("%s -\n", __func__);
	return 0;
}

static int wf012fbm_exit_doze(struct exynos_panel_device *panel)
{
	struct dsim_device *dsim = get_dsim_drvdata(0);
	int ret;
	unsigned char buf[1];

	DPU_INFO_PANEL("%s +\n", __func__);
	mutex_lock(&panel->ops_lock);
	/* Page select */
	dsim_write_data_seq(dsim, false, 0xff, 0x10);
	/* Exit Idle Mode */
	dsim_write_data_seq(dsim, false, MIPI_DCS_EXIT_IDLE_MODE);

	/* Read brightness */
	ret = dsim_read_data(dsim, MIPI_DSI_DCS_READ, 0x51, sizeof(buf), buf);
	if (ret < 0) {
		dsim_err("Failed to read brightness reg from panel\n");
	} else {
		panel->bl->props.brightness = buf[0];
	}
	mutex_unlock(&panel->ops_lock);
	DPU_INFO_PANEL("%s -\n", __func__);
	return 0;
}

struct exynos_panel_ops panel_wf012fbm_ops = {
	.id		= {0x1, 0xffffff, 0xffffff},
	.suspend	= wf012fbm_suspend,
	.displayon	= wf012fbm_displayon,
	.doze		= wf012fbm_doze,
	.doze_suspend	= wf012fbm_doze_suspend,
	.set_light	= wf012fbm_set_light,
	.enter_hbm	= wf012fbm_enter_hbm,
	.exit_hbm	= wf012fbm_exit_hbm,
	.exit_doze	= wf012fbm_exit_doze,
};

