// SPDX-License-Identifier: GPL-2.0+
/*
 * ADMV8818 driver
 *
 * Copyright 2021 Analog Devices Inc.
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/clk-provider.h>
#include <linux/device.h>
#include <linux/iio/iio.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/spi/spi.h>

/* ADMV8818 Register Map */
#define ADMV8818_REG_SPI_CONFIG_A		0x0
#define	ADMV8818_REG_SPI_CONFIG_B		0x1
#define ADMV8818_REG_CHIPTYPE			0x3
#define ADMV8818_REG_PRODUCT_ID_L		0x4
#define ADMV8818_REG_PRODUCT_ID_H		0x5
#define ADMV8818_REG_FAST_LATCH_POINTER		0x10
#define ADMV8818_REG_FAST_LATCH_STOP		0x11
#define ADMV8818_REG_FAST_LATCH_START		0x12
#define ADMV8818_REG_FAST_LATCH_DIRECTION	0x13
#define ADMV8818_REG_FAST_LATCH_STATE		0x14
#define ADMV8818_REG_WR0_SW			0x20
#define ADMV8818_REG_WR0_FILTER			0x21
#define ADMV8818_REG_WR1_SW			0x22
#define ADMV8818_REG_WR1_FILTER			0x23
#define ADMV8818_REG_WR2_SW			0x24
#define ADMV8818_REG_WR2_FILTER			0x25
#define ADMV8818_REG_WR3_SW			0x26
#define ADMV8818_REG_WR3_FILTER			0x27
#define ADMV8818_REG_WR4_SW			0x28
#define ADMV8818_REG_WR4_FILTER			0x29
#define ADMV8818_REG_LUT0_SW			0x100
#define ADMV8818_REG_LUT0_FILTER		0x101
#define ADMV8818_REG_LUT127_SW			0x1FE
#define ADMV8818_REG_LUT127_FILTER		0x1FF

/* ADMV8818_REG_SPI_CONFIG_A Map */
#define ADMV8818_SOFTRESET_N_MSK		BIT(7)
#define ADMV8818_LSB_FIRST_N_MSK		BIT(6)
#define ADMV8818_ENDIAN_N_MSK			BIT(5)
#define ADMV8818_SDOACTIVE_N_MSK		BIT(4)
#define ADMV8818_SDOACTIVE_MSK			BIT(3)
#define ADMV8818_ENDIAN_MSK			BIT(2)
#define ADMV8818_LSBFIRST_MSK			BIT(1)
#define ADMV8818_SOFTRESET_MSK			BIT(0)

/* ADMV8818_REG_WR0_SW Map */
#define ADMV8818_SW_IN_SET_WR0_MSK		BIT(7)
#define ADMV8818_SW_OUT_SET_WR0_MSK		BIT(6)
#define ADMV8818_SW_IN_WR0_MSK			GENMASK(5, 3)
#define ADMV8818_SW_OUT_WR0_MSK			GENMASK(2, 0)

/* ADMV8818_REG_WR0_FILTER Map */
#define ADMV8818_HPF_WR0_MSK			GENMASK(7, 4)
#define ADMV8818_LPF_WR0_MSK			GENMASK(3, 0)

struct admv8818_dev {
	struct regmap		*regmap;
	struct clk		*clkin;
};

unsigned long long freq_range_hpf[4][2] = {
	{1750000000ull, 3550000000ull},
	{3400000000ull, 7250000000ull},
	{6600000000ull, 12000000000ull},
	{12500000000ull, 19900000000ull}
};

unsigned long long freq_range_lpf[4][2] = {
	{2050000000ull, 3850000000ull},
	{3350000000ull, 7250000000ull},
	{7000000000ull, 13000000000ull},
	{12550000000ull, 18500000000ull}
};

static const struct regmap_config admv8818_regmap_config = {
	.reg_bits = 15,
	.val_bits = 8,
	.read_flag_mask = BIT(15),
	.max_register = 0x1FF,
};

static int admv8818_rfin_band_select(struct admv8818_dev *dev)
{
	unsigned int hpf_step, lpf_step, hpf_band, lpf_band, i, j;
	u64 freq_step;
	int ret;

	u64 rate = clk_get_rate(dev->clkin);

	if (rate < freq_range_hpf[0][0] || rate > freq_range_lpf[3][3]) {
		hpf_band = 0;
		lpf_band = 0;
		hpf_step = 0;
		lpf_step = 0;
		goto reg_write;
	}

	for (i = 0; i < 4; i++) {
		if (rate > freq_range_hpf[i][0] && rate <freq_range_hpf[i][0]) {
			hpf_band = i + 1;
			freq_step = (freq_range_hpf[i][0] - freq_range_hpf[i][0]) / 16;

			for (j = 0; j < 16; j++)
				if(rate < (freq_range_hpf[i][0] + (freq_step * j)))
					hpf_step = j - 1;
		}
	}

	for (i = 0; i < 4; i++) {
		if (rate > freq_range_lpf[i][0] && rate <freq_range_lpf[i][0]) {
			lpf_band = i + 1;
			freq_step = (freq_range_lpf[i][0] - freq_range_lpf[i][0]) / 16;

			for (j = 0; j < 16; j++)
				if(rate < (freq_range_lpf[i][0] + (freq_step * j)))
					lpf_step = j;
		}
	}

reg_write:
	ret = regmap_write(dev->regmap, ADMV8818_REG_WR0_FILTER,
				FIELD_PREP(ADMV8818_HPF_WR0_MSK, hpf_step) |
				FIELD_PREP(ADMV8818_LPF_WR0_MSK, lpf_step));
	if (ret)
		return ret;

	return regmap_write(dev->regmap, ADMV8818_REG_WR0_SW,
				FIELD_PREP(ADMV8818_SW_IN_SET_WR0_MSK, 1) |
				FIELD_PREP(ADMV8818_SW_OUT_SET_WR0_MSK, 1) |
				FIELD_PREP(ADMV8818_SW_IN_WR0_MSK, hpf_band) |
				FIELD_PREP(ADMV8818_SW_OUT_WR0_MSK, lpf_band));
}

static int admv8818_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int *val, int *val2, long info)
{

	switch (info) {
	case IIO_CHAN_INFO_LOW_PASS_FILTER_3DB_FREQUENCY:
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_HIGH_PASS_FILTER_3DB_FREQUENCY:
		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}
}

static int admv8818_write_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     int val, int val2, long info)
{
	int ret;

	switch (info) {
	case IIO_CHAN_INFO_LOW_PASS_FILTER_3DB_FREQUENCY:
		return ret;
	case IIO_CHAN_INFO_HIGH_PASS_FILTER_3DB_FREQUENCY:
		return ret;
	default:
		return -EINVAL;
	}
}

static int admv8818_reg_access(struct iio_dev *indio_dev,
				unsigned int reg,
				unsigned int write_val,
				unsigned int *read_val)
{
	struct admv8818_dev *dev = iio_priv(indio_dev);

	if (read_val)
		return regmap_read(dev->regmap, reg, read_val);
	else
		return regmap_write(dev->regmap, reg, write_val);
}

static const struct iio_info admv8818_info = {
	.read_raw = admv8818_read_raw,
	.write_raw = admv8818_write_raw,
	.debugfs_reg_access = &admv8818_reg_access,
};

#define ADMV8818_CHAN(_channel) {				\
	.type = IIO_VOLTAGE,					\
	.output = 1,						\
	.indexed = 1,						\
	.channel = _channel,					\
	.info_mask_separate =					\
		BIT(IIO_CHAN_INFO_LOW_PASS_FILTER_3DB_FREQUENCY) | \
		BIT(IIO_CHAN_INFO_HIGH_PASS_FILTER_3DB_FREQUENCY) \
}

static const struct iio_chan_spec admv8818_channels[] = {
	ADMV8818_CHAN(0),
};

static void admv8818_clk_disable(void *data)
{
	clk_disable_unprepare(data);
}

static int admv8818_probe(struct spi_device *spi)
{
	struct iio_dev *indio_dev;
	struct regmap *regmap;
	struct admv8818_dev *dev;
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*dev));
	if (!indio_dev)
		return -ENOMEM;

	regmap = devm_regmap_init_spi(spi, &admv8818_regmap_config);
	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

	dev = iio_priv(indio_dev);
	dev->regmap = regmap;

	indio_dev->dev.parent = &spi->dev;
	indio_dev->info = &admv8818_info;
	indio_dev->name = "admv8818";
	indio_dev->channels = admv8818_channels;
	indio_dev->num_channels = ARRAY_SIZE(admv8818_channels);

	ret = clk_prepare_enable(dev->clkin);
	if (ret < 0)
		return ret;

	dev->clkin = devm_clk_get(&spi->dev, "rf_in");
	if (IS_ERR(dev->clkin))
		return PTR_ERR(dev->clkin);

	ret = clk_prepare_enable(dev->clkin);
	if (ret)
		return ret;

	ret = devm_add_action_or_reset(&spi->dev, admv8818_clk_disable, dev->clkin);
	if (ret)
		return ret;
	
	ret = admv8818_rfin_band_select(dev);
	if (ret)
		return ret;

	return devm_iio_device_register(&spi->dev, indio_dev);
}

static const struct spi_device_id admv8818_id[] = {
	{ "admv8818", 0 },
	{}
};
MODULE_DEVICE_TABLE(spi, admv8818_id);

static const struct of_device_id admv8818_of_match[] = {
	{ .compatible = "adi,admv8818" },
	{}
};
MODULE_DEVICE_TABLE(of, admv8818_of_match);

static struct spi_driver admv8818_driver = {
	.driver = {
		.name = "admv8818",
		.of_match_table = admv8818_of_match,
	},
	.probe = admv8818_probe,
	.id_table = admv8818_id,
};
module_spi_driver(admv8818_driver);

MODULE_AUTHOR("Antoniu Miclaus <antoniu.miclaus@analog.com");
MODULE_DESCRIPTION("Analog Devices ADMV8818");
MODULE_LICENSE("GPL v2");