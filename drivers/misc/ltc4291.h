/*
 * LTC4291 IEEE 802.3at Quad Port Power-over-Ethernet PSE Controller
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __LTC4291_H__
#define __LTC4291_H__

#define LTC4291_PORTS_NUM	4

#define INTSTAT_REG		0x00 /* Events occured */
#define INTMASK_REG		0x01 /* Interrupt enable */
#define DETEVN_REG		0x05 /* Detect & Class event per port (CoR) */
#define FLTEVN_REG		0x07 /* Disconnect or Icut event (CoR) */
#define TSEVN_REG		0x09 /* Ilim or start fault event (CoR) */
#define STATUS_BASE		0x0C /* Detection & Class status */
#define OPMD_REG		0x12 /* Ports operating mode */
#define DISENA_REG		0x13 /* Disconnect enable */
#define DETENA_REG		0x14 /* Detect and class enable */
#define PWRPB_REG		0x19 /* Ports power control */
#define RSTPB_REG		0x1A /* Clear different port registers */
#define TEMPERATURE_REG		0x2C /* Chip temperature */
#define VEELSB_REG		0x2E /* 2 bytes, little endian */
#define FIRMWARE_REG		0x41 /* device firmware revision */
#define DEVID_REG		0x43 /* device ID, silicon revision */

/* Interrupts MASK*/
#define PWRENA_EN 	1 << 0
#define PWRGD_EN	1 << 1
#define DIS_EN 		1 << 2
#define DET_EN		1 << 3
#define CLASS_EN	1 << 4
#define TCUT_EN		1 << 5
#define TSTRAT_EN	1 << 6
#define SUPPLY_EN	1 << 7
#define ALLEN		0xFF
#define CLRAIN		1 << 7 /* clear all */

/* Events */
#define PWRENA_EVENT 	1 << 0
#define PWRGD_EVENT	1 << 1
#define DIS_EVENT 	1 << 2
#define DET_EVENT	1 << 3
#define CLASS_EVENT	1 << 4
#define TCUT_EVENT	1 << 5
#define TSTRAT_EVENT	1 << 6
#define SUPPLY_EVENT	1 << 7
/*
#define PEC	1 << 0
#define PGC	1 << 1
#define DISF	1 << 2
#define DETC	1 << 3
#define CLASC	1 << 4
#define IFAULT	1 << 5
#define STRTF	1 << 6
#define SUPF	1 << 7
*/

/* Input voltage */
#define INPUT_V_SZ		2
/* Power enable register */
#define P_ON			1
#define P_OFF			0
#define PT_PWR_ON_SHIFT		0
#define PT_PWR_OFF_SHIFT	4

/* Port current and voltage info */
#define PT_DC_REGS		0x30
#define PT_DC_INFO_SZ		16

/* Operating mode register */
#define PORT_MODE_OFF		0
#define PORT_MODE_MANUAL	1
#define PORT_MODE_SEMIAUTO	2
#define PORT_MODE_AUTO		3
#define PT_MODE_FIELD_WIDTH	2
#define PT_MODE_MASK		0x3
/* Disconnect, detect and class enable */
#define DC_EN			0x1
#define DET_CLAS_EN		0x11

/* Detection statuses */
#define CLASS_UNKNOWN		0
#define CLASS1			0x1
#define CLASS2			0x2
#define CLASS3			0x3
#define CLASS4			0x4
#define CLASS0			0x6
#define CLASS_OVERCURRENT	0x7
#define CLASS_MISMATCH		0x8
/* Classification statuses */
#define DETECT_R_VALID		0x4

#define PORT_SELECT( port_i ) (0x1 << (port_i))
#define ASCII_TO_DIGIT( ascii_digit ) (ascii_digit - 0x30) 

#define TEMP_LSB		652 /* 0.652 C */
#define VOLT_LSB		10100 /* 10.1 mV */
#define CURR_S250_LSB		410000   /* 410 uA */
#define CURR_S255_LSB		61039 /* 61.039 uA */
#define WA_LSB			33760 /*33.76 mW */

struct pt_dc_parm {
	s16 i;
	s16 v;
};

struct pt_dflt {
	int enable;
	int mode;
	int pwr;
};

struct ltc4291_platform_data {
	u32 irq;
	struct pt_dflt pt_df[LTC4291_PORTS_NUM];
};

struct ltc4291_data {
	struct i2c_client *client;
	struct ltc4291_platform_data pdata;
	struct pt_dc_parm pt_dc[LTC4291_PORTS_NUM];
};

#endif /* __LTC4291_H__ */
