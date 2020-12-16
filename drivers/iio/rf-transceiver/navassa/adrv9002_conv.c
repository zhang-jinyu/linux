// SPDX-License-Identifier: GPL-2.0
/*
 * ADRV9002 RF Transceiver
 *
 * Copyright 2019 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 */
#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/string.h>
#include <linux/uaccess.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>

#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/clk-provider.h>

#include "adrv9002.h"
#include "adi_adrv9001_rxSettings_types.h"
#include "adi_adrv9001_ssi_types.h"


#define ADI_RX2_REG_OFF			0x1000
#define ADI_TX1_REG_OFF			0x2000
#define ADI_TX2_REG_OFF			0x4000
#define ADI_TX_REG_RATE			0x4c
#define ADI_TX_REG_CTRL_2		0x48
#define ADI_TX_REG_CHAN_CTRL_7(c)	(0x0418 + (c) * 0x40)
#define ADI_TX_REG_CTRL_1		0x44
#define R1_MODE				BIT(2)
#define TX_R1_MODE				BIT(5)

#define AIM_AXI_REG(off, addr)		((off) + (addr))
#define	NUM_LANES_MASK			GENMASK(12, 8)
#define NUM_LANES(x)			FIELD_PREP(NUM_LANES_MASK, x)
#define SDR_DDR_MASK			BIT(16)
#define SDR_DDR(x)			FIELD_PREP(SDR_DDR_MASK, x)

#define IS_CMOS(cfg)			((cfg) & (ADI_CMOS_OR_LVDS_N))


u32 adrv9002_axi_dds_rate_get(struct adrv9002_rf_phy *phy, const int chan)
{
	return -ENODEV;
}

int adrv9002_axi_interface_set(struct adrv9002_rf_phy *phy, const u8 n_lanes,
			       const u8 ssi_intf, const bool cmos_ddr,
			       const int channel)
{
	return -ENODEV;
}

int adrv9002_axi_intf_tune(struct adrv9002_rf_phy *phy, const bool tx, const int chann,
			   const adi_adrv9001_SsiType_e ssi_type, u8 *clk_delay, u8 *data_delay)
{
	return -ENODEV;
}

void adrv9002_axi_interface_enable(struct adrv9002_rf_phy *phy, const int chan, const bool en)
{
	return -ENODEV;
}

adi_adrv9001_SsiType_e adrv9002_axi_ssi_type_get(struct adrv9002_rf_phy *phy)
{
	return -ENODEV;
}

int adrv9002_axi_tx_test_pattern_cfg(struct adrv9002_rf_phy *phy, const int channel,
				     const adi_adrv9001_SsiTestModeData_e data)
{
	return -ENODEV;
}

int adrv9002_hdl_loopback(struct adrv9002_rf_phy *phy, bool enable)
{
	return -ENODEV;
}

int adrv9002_register_axi_converter(struct adrv9002_rf_phy *phy)
{
	struct spi_device *spi = phy->spi;

	spi_set_drvdata(spi, phy); /* Take care here */

	return 0;
}

struct adrv9002_rf_phy *adrv9002_spi_to_phy(struct spi_device *spi)
{
	return spi_get_drvdata(spi);
}

