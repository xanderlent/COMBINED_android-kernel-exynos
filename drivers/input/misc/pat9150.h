/* drivers/input/misc/ots_pat9150/pat9150_linux_driver.h
 *
 * Copyright (c) 2016, The Linux Foundation. All rights reserved.
 *
 */

#ifndef __PAT9150_LINUX_DRIVER_H_
#define __PAT9150_LINUX_DRIVER_H_

#define PAT9150_DEV_NAME	"pixart_pat9150"
#define BUF_SIZE		2
#define RESET_DELAY_US		1000
#define VDD_VTG_MIN_UV		1700000
#define VDD_VTG_MAX_UV		1900000
#define VDD_ACTIVE_LOAD_UA	10000
#define VLD_VTG_MIN_UV		2700000
#define VLD_VTG_MAX_UV		3600000
#define VLD_ACTIVE_LOAD_UA	10000
#define DELAY_BETWEEN_REG_US	10000
#define PAT9150_I2C_RETRY	4

/* Register addresses */
#define PIXART_PAT9150_PRODUCT_ID1_REG		0x00
#define PIXART_PAT9150_PRODUCT_ID2_REG		0x01
#define PIXART_PAT9150_MOTION_STATUS_REG	0x02
#define PIXART_PAT9150_DELTA_X_LO_REG		0x03
#define PIXART_PAT9150_DELTA_X_HI_REG		0x04
#define PIXART_PAT9150_RESET_REG		0x08
#define PIXART_PAT9150_WRITE_PROTECT_REG	0x09
#define PIXART_PAT9150_SET_CPI_RES_X_REG	0x0A
#define PIXART_PAT9150_POWER_DOWN_ENH_REG	0x0D
#define PIXART_PAT9150_SLEEP1_REG		0x22
#define PIXART_PAT9150_SLEEP2_REG		0x23
#define PIXART_PAT9150_SLEEP3_REG		0x24
#define PIXART_PAT9150_SELECT_BANK_REG		0x7F

// Application note indicates this is used to
// set the motion pin to drive mode.
#define PIXART_PAT9150_MOTION_PIN_MODE_REG	0x30

/*Register configuration data */
#define PIXART_PAT9150_SENSOR_ID		0x31
#define PIXART_PAT9150_RESET			0x01
#define PIXART_PAT9125_MOTION_DATA_LENGTH	0x04
#define PIXART_PAT9150_BANK0			0x00
#define PIXART_PAT9150_DISABLE_WRITE_PROTECT	0x5A
#define PIXART_PAT9150_ENABLE_WRITE_PROTECT	0x00
#define PIXART_PAT9150_CPI_RESOLUTION_X		0x80
#define PIXART_PAT9125_LOW_VOLTAGE_SEGMENT	0x04
#define PIXART_PAT9150_VALID_MOTION_DATA	0x80
#define PIXART_PAT9150_POWER_DOWN_POWER_DOWN	0x80

// Upper nibble is the sampling period
// Lower nibble is the enter-mode time
// To get the actual value, add one and multiply by the the
// corresponding value in this table:
//
//           FREQ        ENM
// SLEEP1    4ms         32ms
// SLEEP2    64ms        20480ms
// SLEEP3    512ms       20480ms
#define PIXART_PAT9150_SLEEP1_FREQ_ENM		((0x7 << 4) | 0x7)
#define PIXART_PAT9150_SLEEP2_FREQ_ENM		((0x1 << 4) | 0x0)
#define PIXART_PAT9150_SLEEP3_FREQ_ENM		((0x3 << 4) | 0x0)

// Application note indicates this is used to
// change the motion pin to drive mode.
#define PIXART_PAT9150_MOTION_PIN_MODE_DRIVE	0xD8

#define PIXART_SAMPLING_PERIOD_US_MIN	4000
#define PIXART_SAMPLING_PERIOD_US_MAX	8000

#endif

