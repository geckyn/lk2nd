/*
 * Toshiba DSI to DPI Interface
 *
 * Copyright (c) 2007-2014, The Linux Foundation. All rights reserved.
 * Copyright (C) 2007 Google Incorporated
 *
 * Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <debug.h>
#include <err.h>
#include <smem.h>
#include <msm_panel.h>
#include <board.h>
#include <mipi_dsi.h>
#include <pm8x41_hw.h>
#include <pm8x41.h>
#include "include/panel.h"
#include "panel_display.h"
#include <platform/gpio.h>
#include <platform/timer.h>
#include <stdint.h>
#include <msm_panel.h>
#include <mipi_dsi.h>
#include <sys/types.h>
#include <err.h>
#include <reg.h>
#include <debug.h>
#include <platform/iomap.h>
#include <platform/timer.h>
#include <platform/irqs.h>
#include <dev/gpio.h>
#include <clock.h>
#include <i2c_qup.h>
#include <blsp_qup.h>
#include <gsbi.h>
#include "mipi_tc358762_dsi2dpi.h"
#include "include/panel.h"
#include <target/display.h>
#include <regulator.h>
#include <pm8x41_hw.h>
#include <dev/keys.h>
#include <board.h>

#ifndef u32
#define u32 uint32_t
#endif
#ifndef u16
#define u16 uint16_t
#endif

static struct qup_i2c_dev *dev;

#define GPIO_STATE_LOW 0
#define GPIO_STATE_HIGH 2
#define RESET_GPIO_SEQ_LEN 3

#define TC358762_I2C_ADDRESS 0xB
#define DELAY_INIT_SEQ 0x0000

extern void gpio_tlmm_config(uint32_t gpio, uint8_t func,
			uint8_t dir, uint8_t pull,
			uint8_t drvstr, uint32_t enable);

/* #define _DEBUG_TC */

#ifdef _DEBUG_TC
static void dump_toshiba_registers(void);
#endif

static struct{
	u16 reg;
	u32 data;
} tc358762_init_seq[] = {
#if 0
	{ SYSPMCTRL, 0x00000000 },
	{ DELAY_INIT_SEQ,  5 },
	{ DSI_LANEENABLE,  0x00000007 },
	{ PPI_D0S_CLRSIPOCOUNT, 0x00000000},
	{ PPI_D1S_CLRSIPOCOUNT, 0x00000000},
	{ PPI_D0S_ATMR,  0x00000000 },
	{ PPI_D1S_ATMR,  0x00000000 },
	{ PPI_LPTXTIMECNT,  0x00000002 },
	{ 0x0450,  0x00000002 },
	{ 0x0454,  0x00000122 },
	{ LCDCTRL_PORT,  0x00100101 },
	{ SYSCTRL,   0x0000040F },
	{ PPI_STARTPPI,  0x00000001 },
	{ DSI_STARTDSI,  0x00000001 },
	{ DELAY_INIT_SEQ,  5 },
	{ 0x0468,  0x00000004 },
	{ 0x0470,  0x40300000 },
#else
	{0x47c,   0x00000000},
	{0x210 ,  0x00000003},
	{0x164 , 0x00000004},
	{0x168 ,  0x00000004},
	{0x144 ,  0x00000000 },
	{0x148 ,  0x00000000 },
	{0x0114,  0x00000002 },
	{0x0450,  0x00000001 },
	{0x0454,  0x00000122 },
	{0x0484,  0x00000000 },
	{0x0480,  0x0000001F },
	{0x0400,  0x00000000},
	{0x0410, 0x00000007 },
	{0x0418, 0x0000003C},
	{0x0440, 0x00000100},
	{0x0464,  0x00000205},
	{0x0104,  0x00000001},
	{0x0204,  0x00000001},
	{0x0468,   0x00000004},
	{0x0470,   0x50300000},
	{0x047C,   0x00000080},
	{0x047C,   0x00000000},
	{0x0414,   0x00000005},
	/* For 16 bit panel interface
	{0x47c,   0x00000000},
	{0x210 ,  0x00000007 },
	{0x164 , 0x00000003},
	{0x168 ,  0x00000003},
	{0x144 ,  0x00000000 },
	{0x148 ,  0x00000000 },
	{0x0114,  0x00000002 },
	{0x0450,  0x00000001 },
	{0x0454,  0x00000122 },
	{0x0484,  0x00000000 },
	{0x0480,  0x0000001F },
	{0x0400,  0x00000000},
	{0x0410, 0x00000007 },
	{0x0418, 0x0000003C},
	{0x0440, 0x00000110},
	{0x0464,  0x00000205},
	{0x0104,  0x00000001},
	{0x0204,  0x00000001},
	{0x0468,   0x00000004},
	{0x0470,   0x50300000},
	{0x047C,   0x00000080},
	{0x047C,   0x00000000},
	{0x0414,   0x00000005},
*/
#endif
};

int tc358762_i2c_read(uint16_t reg)
{
	int r;
	int err_return = -1;
	u32 value;
	uint8_t tx_data[] = {
		(reg >> 8) & 0xff,
		reg & 0xff,
	};
	uint8_t rx_data[4];
	struct i2c_msg msgs[] = {
		{
			.addr = TC358762_I2C_ADDRESS,
			.flags = I2C_M_WR,
			.buf = tx_data,
			.len = ARRAY_SIZE(tx_data),
		},
		{
			.addr = TC358762_I2C_ADDRESS,
			.flags = I2C_M_RD,
			.buf = rx_data,
			.len = ARRAY_SIZE(rx_data),
		},
	};
	r = qup_i2c_xfer(dev, msgs, ARRAY_SIZE(msgs));
	if (r < 0) {
		dprintf(CRITICAL, "%s: reg 0x%04x error %d\n", __func__,
				reg, r);
		return r;
	}

	if (r < ARRAY_SIZE(msgs)) {
		dprintf(CRITICAL, "%s: reg 0x%04x msgs %d\n", __func__,
				reg, r);
		return err_return;
	}

	value = rx_data[3] << 24 | rx_data[2] << 16 |
		rx_data[1] << 8 | rx_data[0];

	dprintf(INFO, "%s: reg 0x%04x value 0x%08x\n", __func__,
			reg, value);
	return 0;
}

static int qrd_tc_lcd_i2c_read(uint8_t addr)
{
	int ret = 0;
	/* Create a i2c_msg buffer, that is used to put the controller into read
	   mode and then to read some data. */
	struct i2c_msg msg_buf[] = {
		{TC358762_I2C_ADDRESS, I2C_M_WR, 1, &addr},
		{TC358762_I2C_ADDRESS, I2C_M_RD, 1, &ret}
	};

	ret = qup_i2c_xfer(dev, msg_buf, 2);
	if (ret < 0) {
		dprintf(CRITICAL, "qup_i2c_xfer error %d\n", ret);
		return ret;
	}
	return 0;
}

#ifdef _DEBUG_TC
static void dump_toshiba_registers(void)
{
	int i;
	u16 reg;
	u32 value;

	dprintf(CRITICAL, "inside function = %s\n", __func__);
	for (i = 0; i < ARRAY_SIZE(tc358762_init_seq); ++i) {
		reg = tc358762_init_seq[i].reg;
		if (reg == DELAY_INIT_SEQ)
			continue;
		/*if (qrd_tc_lcd_i2c_read (reg)) {
			dprintf(CRITICAL, "failed to read reg %x\n", reg);
		}*/
		if (tc358762_i2c_read(reg))
			dprintf(CRITICAL, "failed to read reg %x\n", reg);
	}
}
#endif



static int qrd_tc_lcd_i2c_write(uint8_t addr, uint8_t val)
{
	int ret = 0;
	uint8_t data_buf[] = { addr, val };

	/* Create a i2c_msg buffer, that is used to put the controller
	into write mode and then to write some data. */
	struct i2c_msg msg_buf[] = { {TC358762_I2C_ADDRESS,
				      I2C_M_WR, 2, data_buf}
	};

	ret = qup_i2c_xfer(dev, msg_buf, 1);
	if (ret < 0) {
		dprintf(CRITICAL, "qup_i2c_xfer error %d\n", ret);
		return ret;
	}
	return 0;
}



int tc358762_i2c_write(uint16_t reg, uint32_t value)
{
	int r;
	uint8_t tx_data[] = {
		/* NOTE: Register address big-endian, data little-endian. */
		(reg >> 8) & 0xff,
		reg & 0xff,
		value & 0xff,
		(value >> 8) & 0xff,
		(value >> 16) & 0xff,
		(value >> 24) & 0xff,
	};
	/* Send commands via I2C */
	struct i2c_msg msgs[] = {
		{
			.addr = TC358762_I2C_ADDRESS,
			.flags = 0, /* I2C_M_WR, */
			.buf = tx_data,
			.len = ARRAY_SIZE(tx_data),
		},
	};
	dprintf(SPEW, "I2c-addr=%04x reg=%x value = %08x\n",
			msgs[0].addr, reg, value);
	r = qup_i2c_xfer(dev, msgs, ARRAY_SIZE(msgs));

	if (r < 0) {
		dprintf(CRITICAL, "tc358762 write failed rret = %d", r);
		return r;
	}
	return 0;
}

static int tc358762_hw_reset(struct panel_reset_sequence *resetseq)
{
	dprintf(CRITICAL, "Toshiba:%s: Performing HW RESET on tc358762\n",
			__func__);

	regulator_enable(REG_LDO2 | REG_LDO6 | REG_LDO17);
	mdelay(20);

	gpio_tlmm_config(reset_gpio.pin_id, 0,
			reset_gpio.pin_direction, reset_gpio.pin_pull,
			reset_gpio.pin_strength, reset_gpio.pin_state);

	/* gpio_set(reset_gpio.pin_id, 2); */

	/*gpio_tlmm_config(clken_gpio.pin_id, 0,
			clken_gpio.pin_direction, clken_gpio.pin_pull,
			clken_gpio.pin_strength, clken_gpio.pin_state);

	gpio_set(clken_gpio.pin_id, 2);

	gpio_set(clken_gpio.pin_id, GPIO_STATE_HIGH);*/

	mdelay(5);
	gpio_set(reset_gpio.pin_id, GPIO_STATE_LOW);
	mdelay(20);
	gpio_set(reset_gpio.pin_id, GPIO_STATE_HIGH);


	return 0;
}

static void tc358762_write_init_config(void)
{
	int i;

	dprintf(CRITICAL, "Toshiba: %s:\n", __func__);
	for (i = 0; i < ARRAY_SIZE(tc358762_init_seq); ++i) {
		uint16_t reg = tc358762_init_seq[i].reg;
		uint32_t value = tc358762_init_seq[i].data;

		if (reg == DELAY_INIT_SEQ) {
			mdelay(value);
			continue;
		}
		/*if (qrd_tc_lcd_i2c_write(reg, value)) {
			dprintf(CRITICAL, "qrd_tc_lcd_i2c_write failed to write
				register =%x \n", tc358762_init_seq[i].reg);
		}*/
		if (tc358762_i2c_write(reg, value)) {
			dprintf(CRITICAL, "failed to write register =%x\n",
				tc358762_init_seq[i].reg);
		}
	}
}

int tc358762_init(struct panel_reset_sequence *resetseq)
{
	u32 value = 0;
	uint32_t addr_bb_clk1 = 0x5146;
	uint32_t addr_bb_clk2 = 0x5246;
	uint8_t bb_clk2_ctrl_val = 0x80;
	int soc_ver = board_soc_version();

	dprintf(CRITICAL, "Enter %s soc_ver %d\n", __func__, soc_ver);

	dprintf(CRITICAL, "%s bb_clk1 = %x bb_clk2 = %x\n", __func__,
		pm8x41_reg_read(0x5146), pm8x41_reg_read(0x5246));
	/* pm8x41_reg_write(0x5246, 0x80); */
	dprintf(CRITICAL, "%s bb_clk1 = %x bb_clk2 = %x\n", __func__,
		pm8x41_reg_read(0x5146), pm8x41_reg_read(0x5246));
	/*dev = qup_blsp_i2c_init(BLSP_ID_1, QUP_ID_4, 100000, 19200000);
	if(!dev) {
		dprintf(CRITICAL,"%s Failed to initialize i2c\n", __func__);
		return 0;
	}*/
	mdelay(5);
	/* clock and power init done in SBL, hence no need to do it here*/
	/* reset tc358762 bridge */
	tc358762_hw_reset(resetseq);

	dev = qup_blsp_i2c_init(BLSP_ID_1, QUP_ID_4, 100000, 50000000);
	if (!dev) {
		dprintf(CRITICAL, "%s Failed to initialize i2c\n", __func__);
		return 0;
	}
	mdelay(5);
	/* configure D2L chip DSI-RX configuration registers */
	tc358762_write_init_config();

	#ifdef _DEBUG_TC
		dump_toshiba_registers();
	#endif
	return 0;
}
