// SPDX-License-Identifier: GPL-2.0+
/*
 * ADRF6780 driver
 *
 * Copyright 2021 Analog Devices Inc.
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/clk/clkscale.h>
#include <linux/clk-provider.h>
#include <linux/device.h>
#include <linux/iio/iio.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/regmap.h>
#include <linux/spi/spi.h>

#include <linux/iio/sysfs.h>

/* ADRF6780 Register Map */
#define ADRF6780_REG_CONTROL			0x00
#define ADRF6780_REG_ALARM_READBACK		0x01
#define ADRF6780_REG_ALARM_MASKS		0x02
#define ADRF6780_REG_ENABLE			0x03
#define ADRF6780_REG_LINEARIZE			0x04
#define ADRF6780_REG_LO_PATH			0x05
#define ADRF6780_REG_ADC_CONTROL 		0x06
#define ADRF6780_REG_ADC_OUTPUT			0x0C

/* ADRF6780_REG_CONTROL Map */
#define ADRF6780_PARITY_EN_MSK       		BIT(15)
#define ADRF6780_PARITY_EN(x)			FIELD_PREP(ADRF6780_PARITY_EN_MSK, x)
#define ADRF6780_SOFT_RESET_MSK			BIT(14)
#define ADRF6780_SOFT_RESET(x)         		FIELD_PREP(ADRF6780_SOFT_RESET_MSK, x)
#define ADRF6780_CHIP_ID_MSK			GENMASK(11, 4)
#define ADRF6780_CHIP_ID             		0xA
#define ADRF6780_CHIP_REVISION_MSK		GENMASK(3, 0)
#define ADRF6780_CHIP_REVISION(x)        	FIELD_PREP(ADRF6780_CHIP_REVISION_MSK, x)


/* ADRF6780_REG_ALARM_READBACK Map */
#define ADRF6780_PARITY_ERROR_MSK       	BIT(15)
#define ADRF6780_PARITY_ERROR(x)         	FIELD_PREP(ADRF6780_PARITY_ERROR_MSK, x)
#define ADRF6780_TOO_FEW_ERRORS_MSK		BIT(14)
#define ADRF6780_TOO_FEW_ERRORS(x)         	FIELD_PREP(ADRF6780_TOO_FEW_ERRORS_MSK, x)
#define ADRF6780_TOO_MANY_ERRORS_MSK		BIT(13)
#define ADRF6780_TOO_MANY_ERRORS(x)         	FIELD_PREP(ADRF6780_TOO_MANY_ERRORS_MSK, x)
#define ADRF6780_ADDRESS_RANGE_ERROR_MSK	BIT(12)
#define ADRF6780_ADDRESS_RANGE_ERROR(x)         FIELD_PREP(ADRF6780_ADDRESS_RANGE_ERROR_MSK, x)

/* ADRF6780_REG_ENABLE Map */
#define ADRF6780_VGA_BUFFER_EN_MSK     		BIT(8)
#define ADRF6780_VGA_BUFFER_EN(x)      		FIELD_PREP(ADRF6780_VGA_BUFFER_EN_MSK, x)
#define ADRF6780_DETECTOR_EN_MSK       		BIT(7)
#define ADRF6780_DETECTOR_EN(x)         	FIELD_PREP(ADRF6780_DETECTOR_EN_MSK, x)
#define ADRF6780_LO_BUFFER_EN_MSK		BIT(6)
#define ADRF6780_LO_BUFFER_EN(x)         	FIELD_PREP(ADRF6780_LO_BUFFER_EN_MSK, x)
#define ADRF6780_IF_MODE_EN_MSK			BIT(5)
#define ADRF6780_IF_MODE_EN(x)         		FIELD_PREP(ADRF6780_IF_MODE_EN_MSK, x)
#define ADRF6780_IQ_MODE_EN_MSK			BIT(4)
#define ADRF6780_IQ_MODE_EN(x)         		FIELD_PREP(ADRF6780_IQ_MODE_EN_MSK, x)
#define ADRF6780_LO_X2_EN_MSK			BIT(3)
#define ADRF6780_LO_X2_EN(x)       		FIELD_PREP(ADRF6780_LO_X2_EN_MSK, x)
#define ADRF6780_LO_PPF_EN_MSK			BIT(2)
#define ADRF6780_LO_PPF_EN(x)         		FIELD_PREP(ADRF6780_LO_PPF_EN_MSK, x)
#define ADRF6780_LO_EN_MSK			BIT(1)
#define ADRF6780_LO_EN(x)         		FIELD_PREP(ADRF6780_LO_EN_MSK, x)
#define ADRF6780_UC_BIAS_EN_MSK			BIT(0)
#define ADRF6780_UC_BIAS_EN(x)         		FIELD_PREP(ADRF6780_UC_BIAS_EN_MSK, x)

/* ADRF6780_REG_LINEARIZE Map */
#define ADRF6780_RDAC_LINEARIZE_MSK     	GENMASK(7, 0)
#define ADRF6780_RDAC_LINEARIZE(x)      	FIELD_PREP(ADRF6780_RDAC_LINEARIZE_MSK, x)

/* ADRF6780_REG_LO_PATH Map */
#define ADRF6780_LO_SIDEBAND_MSK     		BIT(10)
#define ADRF6780_LO_SIDEBAND(x)      		FIELD_PREP(ADRF6780_LO_SIDEBAND_MSK, x)
#define ADRF6780_Q_PATH_PHASE_ACCURACY_MSK     	GENMASK(7, 4)
#define ADRF6780_Q_PATH_PHASE_ACCURACY(x)  	FIELD_PREP(ADRF6780_Q_PATH_PHASE_ACCURACY_MSK, x)
#define ADRF6780_I_PATH_PHASE_ACCURACY_MSK     	GENMASK(3, 0)
#define ADRF6780_I_PATH_PHASE_ACCURACY(x)      	FIELD_PREP(ADRF6780_I_PATH_PHASE_ACCURACY_MSK, x)

/* ADRF6780_REG_ADC_CONTROL Map */
#define ADRF6780_VDET_OUTPUT_SELECT_MSK    	BIT(3)
#define ADRF6780_VDET_OUTPUT_SELECT(x)     	FIELD_PREP(ADRF6780_VDET_OUTPUT_SELECT_MSK, x)
#define ADRF6780_ADC_START_MSK     		BIT(2)
#define ADRF6780_ADC_START(x)      		FIELD_PREP(ADRF6780_ADC_START_MSK, x)
#define ADRF6780_ADC_EN_MSK     		BIT(1)
#define ADRF6780_ADC_EN(x)  			FIELD_PREP(ADRF6780_ADC_EN_MSK, x)
#define ADRF6780_ADC_CLOCK_EN_MSK     		BIT(0)
#define ADRF6780_ADC_CLOCK_EN(x)      		FIELD_PREP(ADRF6780_ADC_CLOCK_EN_MSK, x)

/* ADRF6780_REG_ADC_OUTPUT Map */
#define ADRF6780_ADC_STATUS_MSK     		BIT(1)
#define ADRF6780_ADC_STATUS(x)  		FIELD_PREP(ADRF6780_ADC_STATUS_MSK, x)
#define ADRF6780_ADC_VALUE_MSK     		GENMASK(7, 0)
#define ADRF6780_ADC_VALUE(x)      		FIELD_PREP(ADRF6780_ADC_VALUE_MSK, x)

enum supported_parts {
	ADRF6780,
};

struct adrf6780_dev {
	struct spi_device	*spi;
	struct clk 		*clkin;
	struct clock_scale	*clkscale;
	struct notifier_block	nb;
	u8			quad_se_mode;
	u64			clkin_freq;
	bool			parity_en;
	bool 			bus_locked;
	u8			data[3];
};

static void check_parity(u32 input, u32 *count)
{
	u32 i = 0;
	while(input) {
		i += input & 1;
		input >>= 1;
	}

	*count = i;
}

static int adrf6780_spi_read(struct adrf6780_dev *dev, unsigned int reg,
			      unsigned int *val)
{
	int ret;
	unsigned int cnt, p_bit, temp;
	struct spi_message m;
	struct spi_transfer t = {0};

	dev->data[0] = 0x80 | (reg << 1);

	t.rx_buf = dev->data;
	t.tx_buf = dev->data;
	t.len = 3;

	spi_message_init_with_transfers(&m, &t, 1);

	if (dev->bus_locked)
		ret = spi_sync_locked(dev->spi, &m);
	else
		ret = spi_sync(dev->spi, &m);

	if (ret < 0)
		return ret;

	temp = (dev->data[2] << 16) | (dev->data[1] << 8) | dev->data[0];

	if (dev->parity_en)
	{
		check_parity(temp, &cnt);
		p_bit = temp & 0x1;

		if ((!(cnt % 2) && p_bit) || ((cnt % 2) && !p_bit))
			return -EINVAL;
	}

	*val = (temp >> 1) & 0xFFFF;

	return ret;
}

static int adrf6780_spi_write(struct adrf6780_dev *dev,
				      unsigned int reg,
				      unsigned int val)
{
	unsigned int cnt;
	struct spi_message m;
	struct spi_transfer t = {0};

	val = (val << 1);

	if (dev->parity_en)
	{
		check_parity((reg << 17) | val , &cnt);

		if (cnt % 2 == 0)
			val |= 0x1;
	}

	t.tx_buf = dev->data;
	t.len = 3;

	dev->data[0] = (reg << 1) | (val >> 16);
	dev->data[1] = val >> 8;
	dev->data[2] = val;

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);

	if (dev->bus_locked)
		return spi_sync_locked(dev->spi, &m);

	return spi_sync(dev->spi, &m);
}

MODULE_AUTHOR("Antoniu Miclaus <antoniu.miclaus@analog.com");
MODULE_DESCRIPTION("Analog Devices ADRF6780");
MODULE_LICENSE("GPL v2");

