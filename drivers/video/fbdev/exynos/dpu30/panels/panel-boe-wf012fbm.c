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

#define PANEL_MAX_TRIES 3

// Look up table to convert between AOD and Interactive DBV that would
// yield the same display nits, defined in go/rh-eos-dbv-lut
const static u8 InteractiveToAodDbvLUT[] = {
	0,   3,   5,   5,   8,   15,  16,  21,  25,  30,  33,  38,  43,
	47,  51,  56,  59,  69,  69,  72,  77,  79,  81,  83,  86,  88,
	90,  92,  94,  96,  98,  101, 103, 106, 110, 112, 114, 116, 120,
	122, 125, 128, 130, 134, 136, 140, 143, 146, 149, 152, 155, 158,
	162, 166, 169, 174, 178, 182, 186, 188, 192, 196, 201, 205, 211,
	215, 220, 225, 228, 232, 237, 241, 246, 251
};

static u8 interactive_to_aod_brightness(u8 interactive_brightness)
{
	if (interactive_brightness >= sizeof(InteractiveToAodDbvLUT)) {
		// This means Interactive brightness (nits) is already saturated
		// if converted to Doze brightness (i.e. > max AOD 150 nits).
		// Let's return the maximum DBV here, which is 255.
		return 255;
	}
	return InteractiveToAodDbvLUT[interactive_brightness];
}

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
	usleep_range(70000, 70000);

	panel->doze_brightness_normalized = false;
	mutex_unlock(&panel->ops_lock);
	DPU_INFO_PANEL("%s -\n", __func__);
	return 0;
}

void panel_recovery(struct dsim_device *dsim)
{
	if (decon_reg_get_run_status(dsim->id)) {
		dsim_reset_panel(dsim);
		dpu_hw_recovery_process(get_decon_drvdata(dsim->id));
	} else {
		dsim_reset_panel(dsim);
		dsim_reg_recovery_process(dsim);
	}
}

int wf012fbm_panel_init(struct dsim_device *dsim)
{
	int ret;
	unsigned char buf[1];

	dsim_write_data_seq(dsim, false, 0xFF, 0x25);
	dsim_write_data_seq(dsim, false, 0xFB, 0x01);
	dsim_write_data_seq(dsim, false, 0xCE, 0x2C);

	/* Page Select */
	dsim_write_data_seq(dsim, false, 0xff, 0x10);

	/* Sleep out , wait 15ms*/
	dsim_write_data_seq(dsim, false, MIPI_DCS_EXIT_SLEEP_MODE);
	usleep_range(15000, 15000);

	/* Set display to 30Hz */
	/* These values were provided from the vendor. */
	/* Page select: */
	dsim_write_data_seq(dsim, false, 0xff, 0x10);
	/* Write 0x81 to register 0x6F to set 30Hz */
	dsim_write_data_seq(dsim, false, 0x6F, 0x81);

	/* Disable clock gating feature: b/204842000 */
	dsim_write_data_seq(dsim, false, 0xff, 0xf0);
	dsim_write_data_seq(dsim, false, 0xfb, 0x01);
	dsim_write_data_seq(dsim, false, 0x57, 0x30);
	dsim_write_data_seq(dsim, false, 0xff, 0x10);

	/* Disable master DBV control mode */
	dsim_write_data_seq(dsim, false, 0xff, 0x21);
	dsim_write_data_seq(dsim, false, 0xfb, 0x01);
	dsim_write_data_seq(dsim, false, 0x5d, 0x00);
	dsim_write_data_seq(dsim, false, 0xff, 0x10);

	/* No gradual DBV change during the mode transition */
	dsim_write_data_seq(dsim, false, 0xff, 0x25);
	dsim_write_data_seq(dsim, false, 0xfb, 0x01);
	dsim_write_data_seq(dsim, false, 0x74, 0xa1);
	dsim_write_data_seq(dsim, false, 0x76, 0x51);
	dsim_write_data_seq(dsim, false, 0xff, 0x10);

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
	/* Page select 0x21 */
	dsim_write_data_seq(dsim, false, 0xff, 0x21);
	/* Enable PWM dimming for Normal mode */
	dsim_write_data_seq(dsim, false, 0x5b, 0xb8);
	/* set PWM duty @ Normal */
	dsim_write_data_seq(dsim, false, 0x6e, 0x01);
	dsim_write_data_seq(dsim, false, 0x6f, 0x90);
	dsim_write_data_seq(dsim, false, 0x84, 0x01);
	dsim_write_data_seq(dsim, false, 0x85, 0x90);
	dsim_write_data_seq(dsim, false, 0x86, 0x01);
	dsim_write_data_seq(dsim, false, 0x87, 0x90);
	dsim_write_data_seq(dsim, false, 0x8b, 0x11);
	dsim_write_data_seq(dsim, false, 0x8d, 0x20);
	dsim_write_data_seq(dsim, false, 0x93, 0x11);
	/* Set 0 nits for AOD DBV0 and DBV1 */
	dsim_write_data_seq(dsim, false, 0x66, 0x01);

	/* Set for brightness ramp/dim timing */
	// Ramp/dim timing: DIM_OTP = 1, step = 18
	// -> ramp/dim duration = 18 frames (300ms @60Hz, 600ms @30Hz)
	dsim_write_data_seq(dsim, false, 0x57, 0x92);
	dsim_write_data_seq(dsim, false, 0x58, 0x12);

	/* Page Select */
	dsim_write_data_seq(dsim, false, 0xff, 0x10);
	usleep_range(55000, 55000);

	/* Display on */
	dsim_write_data_seq(dsim, false, MIPI_DCS_SET_DISPLAY_ON);

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

	/* Power Mode read */
	ret = dsim_read_data(dsim, MIPI_DSI_DCS_READ,
			MIPI_DCS_GET_POWER_MODE, sizeof(buf), buf);
	if (ret < 0) {
		dsim_err("Failed to read PM reg from panel\n");
		return -EINVAL;
	} else {
		dsim_info("=== Panel's PM Reg Value ===\n");
		dsim_info("* 0x0A : buf[0] = %x\n", buf[0]);
		if (buf[0] == 0x9c) {
			return 0;
		} else {
			return -EINVAL;
		}
	}
}

static int wf012fbm_displayon(struct exynos_panel_device *panel)
{
	struct dsim_device *dsim = get_dsim_drvdata(0);
	int ret;
	unsigned char buf[1];
	int retry;

	DPU_INFO_PANEL("%s +\n", __func__);
	mutex_lock(&panel->ops_lock);

	// Check initial state
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
		panel_recovery(dsim);
		ret = dsim_read_data(dsim, MIPI_DSI_DCS_READ,
				MIPI_DCS_GET_POWER_MODE, sizeof(buf), buf);
		if (ret < 0) {
			dsim_err("Failed to read PM reg from panel (second try)\n");
		} else {
			dsim_info("=== Panel's PM Reg Value ===\n");
			dsim_info("* 0x0A : buf[0] = %x\n", buf[0]);
		}
	}

	// Run initialization sequence up to 3 tries
	for (retry = 0; retry < PANEL_MAX_TRIES; retry++) {
		ret = wf012fbm_panel_init(dsim);
		if (ret == 0) {
			break;
		} else if (retry == 0) {
			dsim_err("Invalid display power mode, re-running init sequence\n");
		} else if (retry == 1) {
			dsim_err("Invalid display power mode on second try, resetting display\n");
			panel_recovery(dsim);
		} else {
			dsim_err("Display didn't recover after reset, giving up\n");
		}
	}
	/* Exit Idle Mode */
	dsim_write_data_seq(dsim, false, MIPI_DCS_EXIT_IDLE_MODE);
	panel->doze_brightness_normalized = false;

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
	/* Exit HBM in case it's on */
	dsim_write_data_seq(dsim, false, 0x66, 0x00);

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
	u8 interactive_brightness;
	u8 aod_brightness;
	struct dsim_device *dsim = get_dsim_drvdata(0);
	/*
	 * Only set brightness if it's not currently in doze mode as AP
	 * assumes DBVs are in 650 nits range while in fact it could be
	 * in 150 nits range if the device already entered doze.
	 *
	 * In order to prevent wrong brightness set due to this 650/150
	 * nits scaling, the MCU will be solely responsible for controlling
	 * display brightness during doze (150 nits range). Thus, this
	 * function would do nothing if the device is currently in doze.
	 */
	if (dsim->state != DSIM_STATE_DOZE) {
		mutex_lock(&panel->ops_lock);
		/* WRDISBV(8bit): 1st DBV[7:0] */
		interactive_brightness = br_val & 0xFF;
		aod_brightness =
			interactive_to_aod_brightness(interactive_brightness);
		dsim_write_data_seq(dsim, false, 0x51, interactive_brightness);
		dsim_write_data_seq(dsim, false, 0x61, aod_brightness);
		mutex_unlock(&panel->ops_lock);
		DPU_INFO_PANEL("%s: set brightness (i=%d, aod=%d)\n", __func__,
			       interactive_brightness, aod_brightness);
	} else {
		DPU_INFO_PANEL("%s: skip doze br=%d\n", __func__, br_val);
	}
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
	u8 brightness = 0;

	DPU_INFO_PANEL("%s +\n", __func__);
	mutex_lock(&panel->ops_lock);
	/* page select 0x10 */
	dsim_write_data_seq(dsim, false, 0xff, 0x10);
	/* Read interactive brightness */
	dsim_read_data(dsim, MIPI_DSI_DCS_READ, 0x51,
			sizeof(brightness), &brightness);
	if (brightness != 0) {
		panel->bl->props.brightness = brightness;
	}
	/* Exit Idle Mode */
	dsim_write_data_seq(dsim, false, MIPI_DCS_EXIT_IDLE_MODE);
	mutex_unlock(&panel->ops_lock);
	DPU_INFO_PANEL("%s - read br=%d\n", __func__, brightness);
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

