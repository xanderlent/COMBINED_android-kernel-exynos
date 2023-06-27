/*
 * Copyright (C) 2020 Novatek, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 */

#if NVT_TOUCH_MP

static uint32_t IC_X_CFG_SIZE = 6;
static uint32_t IC_Y_CFG_SIZE = 6;
static uint32_t IC_KEY_CFG_SIZE = 0;
static uint32_t X_Channel = 6;
static uint32_t Y_Channel = 6;
#if TOUCH_KEY_NUM > 0
static uint32_t Key_Channel = TOUCH_KEY_NUM;
#endif
static uint8_t AIN_X[10] = {0, 1, 2, 3, 4, 5};
static uint8_t AIN_Y[10] = {0, 1, 2, 3, 4, 5};
#if TOUCH_KEY_NUM > 0
static uint8_t AIN_KEY[8] = {0xFF, 0xFF, 0xFF, 0xFF};
#endif /* #if TOUCH_KEY_NUM > 0 */

static int32_t PS_Config_Lmt_Short_Rawdata_P[10 * 10] = {
	14008,14008,14008,14008,14008,14008,
	14008,14008,14008,14008,14008,14008,
	14008,14008,14008,14008,14008,14008,
	14008,14008,14008,14008,14008,14008,
	14008,14008,14008,14008,14008,14008,
	14008,14008,14008,14008,14008,14008,
#if TOUCH_KEY_NUM > 0
	14008,14008,14008,
#endif /* #if TOUCH_KEY_NUM > 0 */
};

static int32_t PS_Config_Lmt_Short_Rawdata_N[10 * 10] = {
	-65535,-65535,-65535,-65535,-65535,-65535,
	-65535,-65535,-65535,-65535,-65535,-65535,
	-65535,-65535,-65535,-65535,-65535,-65535,
	-65535,-65535,-65535,-65535,-65535,-65535,
	-65535,-65535,-65535,-65535,-65535,-65535,
	-65535,-65535,-65535,-65535,-65535,-65535,
#if TOUCH_KEY_NUM > 0
	10000,10000,10000,
#endif /* #if TOUCH_KEY_NUM > 0 */
};

static int32_t PS_Config_Lmt_Open_Rawdata_P[10 * 10] = {
	5120,5120,5120,5120,5120,5120,
	5120,5120,5120,5120,5120,5120,
	5120,5120,5120,5120,5120,5120,
	5120,5120,5120,5120,5120,5120,
	5120,5120,5120,5120,5120,5120,
	5120,5120,5120,5120,5120,5120,
#if TOUCH_KEY_NUM > 0
	5120,5120,5120,
#endif /* #if TOUCH_KEY_NUM > 0 */
};

static int32_t PS_Config_Lmt_Open_Rawdata_N[10 * 10] = {
	-511,-511,-511,-511,-511,-511,
	-511,-511,-511,-511,-511,-511,
	-511,-511,-511,-511,-511,-511,
	-511,-511,-511,-511,-511,-511,
	-511,-511,-511,-511,-511,-511,
	-511,-511,-511,-511,-511,-511,
#if TOUCH_KEY_NUM > 0
	-511,-511,-511,
#endif /* #if TOUCH_KEY_NUM > 0 */
};

static int32_t PS_Config_Lmt_FW_Rawdata_P[10 * 10] = {
	2560,2560,2560,2560,2560,2560,
	2560,2560,2560,2560,2560,2560,
	2560,2560,2560,2560,2560,2560,
	2560,2560,2560,2560,2560,2560,
	2560,2560,2560,2560,2560,2560,
	2560,2560,2560,2560,2560,2560,
#if TOUCH_KEY_NUM > 0
	2560,2560,2560,
#endif /* #if TOUCH_KEY_NUM > 0 */
};

static int32_t PS_Config_Lmt_FW_Rawdata_N[10 * 10] = {
	-65535,-65535,-65535,-65535,-65535,-65535,
	-65535,-65535,-65535,-65535,-65535,-65535,
	-65535,-65535,-65535,-65535,-65535,-65535,
	-65535,-65535,-65535,-65535,-65535,-65535,
	-65535,-65535,-65535,-65535,-65535,-65535,
	-65535,-65535,-65535,-65535,-65535,-65535,
#if TOUCH_KEY_NUM > 0
	240,240,240,
#endif /* #if TOUCH_KEY_NUM > 0 */
};

static int32_t PS_Config_Lmt_FW_CC_P[10 * 10] = {
	65535,65535,65535,65535,65535,65535,
	65535,65535,65535,65535,65535,65535,
	65535,65535,65535,65535,65535,65535,
	65535,65535,65535,65535,65535,65535,
	65535,65535,65535,65535,65535,65535,
	65535,65535,65535,65535,65535,65535,
#if TOUCH_KEY_NUM > 0
	314,314,314,
#endif /* #if TOUCH_KEY_NUM > 0 */
};

static int32_t PS_Config_Lmt_FW_CC_N[10 * 10] = {
	-65535,-65535,-65535,-65535,-65535,-65535,
	-65535,-65535,-65535,-65535,-65535,-65535,
	-65535,-65535,-65535,-65535,-65535,-65535,
	-65535,-65535,-65535,-65535,-65535,-65535,
	-65535,-65535,-65535,-65535,-65535,-65535,
	-65535,-65535,-65535,-65535,-65535,-65535,
#if TOUCH_KEY_NUM > 0
	0,0,0,
#endif /* #if TOUCH_KEY_NUM > 0 */
};

static int32_t PS_Config_Lmt_FW_Diff_P[10 * 10] = {
	75,75,75,75,75,75,
	75,75,75,75,75,75,
	75,75,75,75,75,75,
	75,75,75,75,75,75,
	75,75,75,75,75,75,
	75,75,75,75,75,75,
#if TOUCH_KEY_NUM > 0
	75,75,75,
#endif /* #if TOUCH_KEY_NUM > 0 */
};

static int32_t PS_Config_Lmt_FW_Diff_N[10 *10] = {
	-75,-75,-75,-75,-75,-75,
	-75,-75,-75,-75,-75,-75,
	-75,-75,-75,-75,-75,-75,
	-75,-75,-75,-75,-75,-75,
	-75,-75,-75,-75,-75,-75,
	-75,-75,-75,-75,-75,-75,
#if TOUCH_KEY_NUM > 0
	-75,-75,-75,
#endif /* #if TOUCH_KEY_NUM > 0 */
};

static int32_t PS_Config_Diff_Test_Frame = 50;

#endif /* #if NVT_TOUCH_MP */
