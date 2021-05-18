// SPDX-License-Identifier: GPL-2.0+
/*
 * ADMV1014 driver
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
#include <linux/regmap.h>
#include <linux/spi/spi.h>

#include <linux/iio/sysfs.h>

/* ADMV1014 Register Map */
#define ADMV1014_REG_SPI_CONTROL		0x00
#define ADMV1014_REG_ALARM 			0x01
#define ADMV1014_REG_ALARM_MASKS		0x02
#define ADMV1014_REG_ENABLE			0x03
#define ADMV1014_REG_QUAD			0x04
#define ADMV1014_REG_LO_AMP_PHASE_ADJUST1	0x05
#define ADMV1014_REG_MIXER   			0x07
#define ADMV1014_REG_IF_AMP  			0x08
#define ADMV1014_REG_IF_AMP_BB_AMP		0x09
#define ADMV1014_REG_BB_AMP_AGC			0x0A
#define ADMV1014_REG_VVA_TEMP_COMP		0x0B

/* ADMV1014_REG_SPI_CONTROL Map */
#define ADMV1014_PARITY_EN_MSK       		BIT(15)
#define ADMV1014_PARITY_EN(x)         		FIELD_PREP(ADMV1014_PARITY_EN_MSK, x)
#define ADMV1014_SPI_SOFT_RESET_MSK		BIT(14)
#define ADMV1014_SPI_SOFT_RESET(x)         	FIELD_PREP(ADMV1014_SPI_SOFT_RESET_MSK, x)
#define ADMV1014_CHIP_ID_MSK			GENMASK(11, 4)
#define ADMV1014_CHIP_ID             		0x9
#define ADMV1014_REVISION_ID_MSK		GENMASK(3, 0)
#define ADMV1014_REVISION_ID(x)        		FIELD_PREP(ADMV1014_REVISION_ID_MSK, x)

/* ADMV1014_REG_ALARM Map */
#define ADMV1014_PARITY_ERROR_MSK       	BIT(15)
#define ADMV1014_PARITY_ERROR(x)         	FIELD_PREP(ADMV1014_PARITY_ERROR_MSK, x)
#define ADMV1014_TOO_FEW_ERRORS_MSK		BIT(14)
#define ADMV1014_TOO_FEW_ERRORS(x)         	FIELD_PREP(ADMV1014_TOO_FEW_ERRORS_MSK, x)
#define ADMV1014_TOO_MANY_ERRORS_MSK		BIT(13)
#define ADMV1014_TOO_MANY_ERRORS(x)         	FIELD_PREP(ADMV1014_TOO_MANY_ERRORS_MSK, x)
#define ADMV1014_ADDRESS_RANGE_ERROR_MSK	BIT(12)
#define ADMV1014_ADDRESS_RANGE_ERROR(x)         FIELD_PREP(ADMV1014_ADDRESS_RANGE_ERROR_MSK, x)

/* ADMV1014_REG_ENABLE Map */
#define ADMV1014_IBIAS_PD_MSK       		BIT(14)
#define ADMV1014_IBIAS_PD(x)         		FIELD_PREP(ADMV1014_IBIAS_PD_MSK, x)
#define ADMV1014_P1DB_COMPENSATION_MSK		GENMASK(13, 12)
#define ADMV1014_P1DB_COMPENSATION(x)         	FIELD_PREP(ADMV1014_P1DB_COMPENSATION_MSK, x)
#define ADMV1014_IF_AMP_PD_MSK			BIT(11)
#define ADMV1014_IF_AMP_PD(x)         		FIELD_PREP(ADMV1014_IF_AMP_PD_MSK, x)
#define ADMV1014_QUAD_BG_PD_MSK			BIT(9)
#define ADMV1014_QUAD_BG_PD(x)         		FIELD_PREP(ADMV1014_QUAD_BG_PD_MSK, x)
#define ADMV1014_BB_AMP_PD_MSK			BIT(8)
#define ADMV1014_BB_AMP_PD x)         		FIELD_PREP(ADMV1014_BB_AMP_PD_MSK, x)
#define ADMV1014_QUAD_IBIAS_PD_MSK		BIT(7)
#define ADMV1014_QUAD_IBIAS_PD(x)         	FIELD_PREP(ADMV1014_QUAD_IBIAS_PD_MSK, x)
#define ADMV1014_DET_EN_MSK			BIT(6)
#define ADMV1014_DET_EN(x)         		FIELD_PREP(ADMV1014_DET_EN_MSK, x)
#define ADMV1014_BG_PD_MSK			BIT(5)
#define ADMV1014_BG_PD(x)         		FIELD_PREP(ADMV1014_BG_PD_MSK, x)

/* ADMV1014_REG_QUAD Map */
#define ADMV1014_QUAD_SE_MODE_MSK		GENMASK(9, 6)
#define ADMV1014_QUAD_SE_MODE(x)         	FIELD_PREP(ADMV1014_QUAD_SE_MODE_MSK, x)
#define ADMV1014_QUAD_FILTERS_MSK		GENMASK(3, 0)
#define ADMV1014_QUAD_FILTERS(x)         	FIELD_PREP(ADMV1014_QUAD_FILTERS_MSK, x)


/* ADMV1014_LO_AMP_PHASE_ADJUST1 Map */
#define ADMV1014_LOAMP_PH_ADJ_I_FINE_MSK	GENMASK(15, 9)
#define ADMV1014_LOAMP_PH_ADJ_I_FINE(x)        	FIELD_PREP(ADMV1014_LOAMP_PH_ADJ_I_FINE_MSK, x)
#define ADMV1014_LOAMP_PH_ADJ_Q_FINE_MSK	GENMASK(8, 2)
#define ADMV1014_LOAMP_PH_ADJ_Q_FINE(x)        	FIELD_PREP(ADMV1014_LOAMP_PH_ADJ_Q_FINE_MSK, x)

/* ADMV1014_REG_MIXER Map */
#define ADMV1014_MIXER_VGATE_MSK		GENMASK(15, 9)
#define ADMV1014_MIXER_VGATE(x)        		FIELD_PREP(ADMV1014_MIXER_VGATE_MSK, x)
#define ADMV1014_DET_PROG_MSK			GENMASK(6, 0)
#define ADMV1014_DET_PROG(x)			FIELD_PREP(ADMV1014_DET_PROG_MSK, x)

/* ADMV1014_REG_IF_AMP Map */
#define ADMV1014_IF_AMP_COARSE_GAIN_I_MSK	GENMASK(11, 8)
#define ADMV1014_IF_AMP_COARSE_GAIN_I(x)  	FIELD_PREP(ADMV1014_IF_AMP_COARSE_GAIN_I_MSK, x)
#define ADMV1014_IF_AMP_FINE_GAIN_Q_MSK		GENMASK(7, 4)
#define ADMV1014_IF_AMP_FINE_GAIN_Q(x)		FIELD_PREP(ADMV1014_IF_AMP_FINE_GAIN_Q_MSK, x)
#define ADMV1014_IF_AMP_FINE_GAIN_I_MSK		GENMASK(7, 4)
#define ADMV1014_IF_AMP_FINE_GAIN_I(x)		FIELD_PREP(ADMV1014_IF_AMP_FINE_GAIN_I_MSK, x)

/* ADMV1014_REG_IF_AMP_BB_AMP Map */
#define ADMV1014_IF_AMP_COARSE_GAIN_Q_MSK	GENMASK(15, 12)
#define ADMV1014_IF_AMP_COARSE_GAIN_Q(x)  	FIELD_PREP(ADMV1014_IF_AMP_COARSE_GAIN_Q_MSK, x)
#define ADMV1014_BB_AMP_OFFSET_Q_MSK		GENMASK(9, 5)
#define ADMV1014_BB_AMP_OFFSET_Q(x)		FIELD_PREP(ADMV1014_BB_AMP_OFFSET_Q_MSK, x)
#define ADMV1014_BB_AMP_OFFSET_I_MSK		GENMASK(4, 0)
#define ADMV1014_BB_AMP_OFFSET_I(x)		FIELD_PREP(ADMV1014_BB_AMP_OFFSET_I_MSK, x)

/* ADMV1014_REG_BB_AMP_AGC Map */
#define ADMV1014_BB_AMP_REF_GEN_MSK		GENMASK(6, 3)
#define ADMV1014_IF_AMP_COARSE_GAIN_Q(x)  	FIELD_PREP(ADMV1014_IF_AMP_COARSE_GAIN_Q_MSK, x)
#define ADMV1014_BB_AMP_GAIN_CTRL_MSK		GENMASK(2, 1)
#define ADMV1014_BB_AMP_GAIN_CTRL(x)		FIELD_PREP(ADMV1014_BB_AMP_GAIN_CTRL_MSK, x)
#define ADMV1014_BB_SWITCH_HIGH_LOW_CM_MSK	BIT(0)
#define ADMV1014_BB_SWITCH_HIGH_LOW_CM(x)	FIELD_PREP(ADMV1014_BB_SWITCH_HIGH_LOW_CM_MSK, x)

/* ADMV1014_REG_VVA_TEMP_COMP Map */
#define ADMV1014_VVA_TEMP_COMP_MSK		GENMASK(15, 0)
#define ADMV1014_VVA_TEMP_COMP(x)  		FIELD_PREP(ADMV1014_VVA_TEMP_COMP_MSK, x)

#define ADMV1014_MAX_SPI_READ 3
#define ADMV1014_SPI_READ_BUFFER_SIZE (ADMV1014_MAX_SPI_READ + 1)

enum supported_parts {
	ADMV1014,
};

struct admv1014_dev {
	struct spi_device 	*spi;
	struct regmap		*regmap;
	struct clk 		*clkin;
	struct clock_scale	*clkscale;
	struct notifier_block	nb;
	u8			quad_se_mode;
	u64			clkin_freq;
	bool			parity_en;
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

static int admv1014_regmap_spi_read(void *context,
				  const void *reg, size_t reg_size,
				  void *val, size_t val_size)
{
	struct device *dev = context;
	struct spi_device *spi = to_spi_device(dev);
	u8 result[ADMV1014_SPI_READ_BUFFER_SIZE];

	if (val_size > ADMV1014_MAX_SPI_READ)
		return -EINVAL;

	return spi_write_then_read(spi, reg, 1, result, val_size + 1);
	// TODO: Bit shifting and parity check
}

static int admv1014_regmap_spi_write(void *context, const void *data,
				   size_t count)
{
	struct device *device = context;
	struct spi_device *spi = to_spi_device(device);
	struct iio_dev *indio_dev = dev_to_iio_dev(device);
	struct admv1014_dev *dev = iio_priv(indio_dev);
	u32 cnt, *buf;

	buf = data;
	*buf <<= 1;

	if (dev->parity_en)
	{
		check_parity(*buf, &cnt);

		if (cnt % 2 == 0)
			*buf |= 0x1;
	}

	return spi_write(spi, buf, count);
}

static int admv1014_regmap_spi_update_bits(void *context, unsigned int reg,
			       unsigned int mask, unsigned int val)
{
	struct device *device = context;
	struct spi_device *spi = to_spi_device(device);
	struct iio_dev *indio_dev = dev_to_iio_dev(device);
	struct admv1014_dev *dev = iio_priv(indio_dev);
	u32 data, temp;
	int status;

	status = regmap_read(dev->regmap, reg, &data);
	if (status < 0)
		return status;

	temp = data & ~mask;
	temp |= val & mask;

	return regmap_write(dev->regmap, reg, temp);
}

static const struct regmap_config admv1014_regmap_config = {
	.reg_bits = 8,
	.val_bits = 16,
	.read_flag_mask = BIT(7),
	.max_register = 0x0B,
};

static struct regmap_bus admv1014_regmap_bus = {
	.read = admv1014_regmap_spi_read,
	.write = admv1014_regmap_spi_write,
	.reg_update_bits = admv1014_regmap_spi_update_bits,
	.read_flag_mask = BIT(7),
	.max_raw_read = ADMV1014_MAX_SPI_READ,
};

enum admv1014_iio_dev_attr {
	IF_AMP_COARSE_GAIN_I,
	IF_AMP_COARSE_GAIN_Q,
	IF_AMP_FINE_GAIN_I,
	IF_AMP_FINE_GAIN_Q,
	LOAMP_PH_ADJ_I_FINE,
	LOAMP_PH_ADJ_Q_FINE,
};

static ssize_t admv1014_store(struct device *device,
			      struct device_attribute *attr,
			      const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(device);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct admv1014_dev *dev = iio_priv(indio_dev);
	u16 mask = 0, val = 0;
	u8 reg = 0;
	int ret = 0;

	ret = kstrtou16(buf, 10, &val);
	if (ret)
		return ret;

	switch ((u32)this_attr->address) {
	case IF_AMP_COARSE_GAIN_I:
		reg = ADMV1014_REG_IF_AMP;
		mask = ADMV1014_IF_AMP_COARSE_GAIN_I_MSK;
		val = ADMV1014_IF_AMP_COARSE_GAIN_I(val);
		break;
	case IF_AMP_COARSE_GAIN_Q:
		reg = ADMV1014_REG_IF_AMP_BB_AMP;
		mask = ADMV1014_IF_AMP_COARSE_GAIN_Q_MSK;
		val = ADMV1014_IF_AMP_COARSE_GAIN_Q(val);
		break;
	case IF_AMP_FINE_GAIN_I:
		reg = ADMV1014_REG_IF_AMP;
		mask = ADMV1014_IF_AMP_FINE_GAIN_I_MSK;
		val = ADMV1014_IF_AMP_FINE_GAIN_I(val);
		break;
	case IF_AMP_FINE_GAIN_Q:
		reg = ADMV1014_REG_IF_AMP;
		mask = ADMV1014_IF_AMP_FINE_GAIN_Q_MSK;
		val = ADMV1014_IF_AMP_FINE_GAIN_Q(val);
		break;
	case LOAMP_PH_ADJ_I_FINE:
		reg = ADMV1014_REG_LO_AMP_PHASE_ADJUST1;
		mask = ADMV1014_LOAMP_PH_ADJ_I_FINE_MSK;
		val = ADMV1014_LOAMP_PH_ADJ_I_FINE(val);
		break;
	case LOAMP_PH_ADJ_Q_FINE:
		reg = ADMV1014_REG_LO_AMP_PHASE_ADJUST1;
		mask = ADMV1014_LOAMP_PH_ADJ_Q_FINE_MSK;
		val = ADMV1014_LOAMP_PH_ADJ_Q_FINE(val);
		break;
	default:
		return -EINVAL;
	}

	ret = regmap_update_bits(dev->regmap, reg, mask, val);

	return ret ? ret : len;
}

static ssize_t admv1014_show(struct device *device,
			struct device_attribute *attr,
			char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(device);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct admv1014_dev *dev = iio_priv(indio_dev);
	int ret = 0;
	u16 mask = 0, data_shift = 0;
	u32 val = 0;
	u8 reg = 0;

	switch ((u32)this_attr->address) {
	case IF_AMP_COARSE_GAIN_I:
		reg = ADMV1014_REG_IF_AMP;
		mask = ADMV1014_IF_AMP_COARSE_GAIN_I_MSK;
		data_shift = 8;
		break;
	case IF_AMP_COARSE_GAIN_Q:
		reg = ADMV1014_REG_IF_AMP_BB_AMP;
		mask = ADMV1014_IF_AMP_COARSE_GAIN_Q_MSK;
		data_shift = 12;
		break;
	case IF_AMP_FINE_GAIN_I:
		reg = ADMV1014_REG_IF_AMP;
		mask = ADMV1014_IF_AMP_FINE_GAIN_I_MSK;
		data_shift = 4;
		break;
	case IF_AMP_FINE_GAIN_Q:
		reg = ADMV1014_REG_IF_AMP;
		mask = ADMV1014_IF_AMP_FINE_GAIN_Q_MSK;
		break;
	case LOAMP_PH_ADJ_I_FINE:
		reg = ADMV1014_REG_LO_AMP_PHASE_ADJUST1;
		mask = ADMV1014_LOAMP_PH_ADJ_I_FINE_MSK;
		data_shift = 9;
		break;
	case LOAMP_PH_ADJ_Q_FINE:
		reg = ADMV1014_REG_LO_AMP_PHASE_ADJUST1;
		mask = ADMV1014_LOAMP_PH_ADJ_Q_FINE_MSK;
		data_shift = 2;
		break;
	default:
		return -EINVAL;
	}

	ret = regmap_read(dev->regmap, reg, &val);
	if (ret < 0)
		return ret;

	val = (val & mask) >> data_shift;

	return sprintf(buf, "%d\n", val);
}

static IIO_DEVICE_ATTR(if_amp_coarse_gain_i, S_IRUGO | S_IWUSR,
		       admv1014_show,
		       admv1014_store,
		       IF_AMP_COARSE_GAIN_I);

static IIO_DEVICE_ATTR(if_amp_coarse_gain_q, S_IRUGO | S_IWUSR,
		       admv1014_show,
		       admv1014_store,
		       IF_AMP_COARSE_GAIN_Q);

static IIO_DEVICE_ATTR(if_amp_fine_gain_i, S_IRUGO | S_IWUSR,
		       admv1014_show,
		       admv1014_store,
		       IF_AMP_FINE_GAIN_I);

static IIO_DEVICE_ATTR(if_amp_fine_gain_q, S_IRUGO | S_IWUSR,
		       admv1014_show,
		       admv1014_store,
		       IF_AMP_FINE_GAIN_Q);

static IIO_DEVICE_ATTR(loamp_ph_adj_i_fine, S_IRUGO | S_IWUSR,
		       admv1014_show,
		       admv1014_store,
		       LOAMP_PH_ADJ_I_FINE);

static IIO_DEVICE_ATTR(loamp_ph_adj_q_fine, S_IRUGO | S_IWUSR,
		       admv1014_show,
		       admv1014_store,
		       LOAMP_PH_ADJ_Q_FINE);


static struct attribute *admv1014_attributes[] = {
	&iio_dev_attr_if_amp_coarse_gain_i.dev_attr.attr,
	&iio_dev_attr_if_amp_coarse_gain_q.dev_attr.attr,
	&iio_dev_attr_if_amp_fine_gain_i.dev_attr.attr,
	&iio_dev_attr_if_amp_fine_gain_q.dev_attr.attr,
	&iio_dev_attr_loamp_ph_adj_i_fine.dev_attr.attr,
	&iio_dev_attr_loamp_ph_adj_q_fine.dev_attr.attr,
	NULL
};

static const struct attribute_group admv1014_attribute_group = {
	.attrs = admv1014_attributes,
};

static int admv1014_reg_access(struct iio_dev *indio_dev,
				unsigned int reg,
				unsigned int write_val,
				unsigned int *read_val)
{
	struct admv1014_dev *dev = iio_priv(indio_dev);

	if (read_val)
		return regmap_read(dev->regmap, reg, read_val);
	else
		return regmap_write(dev->regmap, reg, write_val);
}

static const struct iio_info admv1014_info = {
	.debugfs_reg_access = &admv1014_reg_access,
	.attrs = &admv1014_attribute_group,
};

static int admv1014_freq_change(struct notifier_block *nb, unsigned long flags, void *data)
{
	struct admv1014_dev *dev = container_of(nb, struct admv1014_dev, nb);
	struct clk_notifier_data *cnd = data;

	/* cache the new rate */
	dev->clkin_freq = clk_get_rate_scaled(cnd->new_rate, dev->clkscale);

	return NOTIFY_OK;
}

static void admv1014_clk_notifier_unreg(void *data)
{
	struct admv1014_dev *dev = data;

	clk_notifier_unregister(dev->clkin, &dev->nb);
}

static int admv1014_init(struct admv1014_dev *dev)
{
	int ret;
	u32 chip_id;

	/* Perform a software reset */
	ret = regmap_update_bits(dev->regmap, ADMV1014_REG_SPI_CONTROL,
				 ADMV1014_SPI_SOFT_RESET_MSK,
				 ADMV1014_SPI_SOFT_RESET(1));
	if (ret < 0)
		return ret;

	ret = regmap_write(dev->regmap, ADMV1014_REG_VVA_TEMP_COMP, 0x727C);
	if (ret < 0)
		return ret;

	ret = regmap_update_bits(dev->regmap, ADMV1014_REG_ENABLE,
				 ADMV1014_P1DB_COMPENSATION_MSK,
				 ADMV1014_P1DB_COMPENSATION(3));
	if (ret < 0)
		return ret;

	ret = regmap_read(dev->regmap, ADMV1014_REG_SPI_CONTROL, &chip_id);
	if (ret < 0)
		return ret;

	chip_id = (chip_id & ADMV1014_CHIP_ID_MSK) >> 4;
	if (chip_id != ADMV1014_CHIP_ID)
		return -EINVAL;

	return regmap_update_bits(dev->regmap, ADMV1014_REG_QUAD,
				 ADMV1014_QUAD_SE_MODE_MSK,
				 ADMV1014_QUAD_SE_MODE(dev->quad_se_mode));

}

static void admv1014_clk_disable(void *data)
{
	struct admv1014_dev *dev = data;

	clk_disable_unprepare(dev->clkin);
}

static int admv1014_probe(struct spi_device *spi)
{
	struct iio_dev *indio_dev;
	struct regmap *regmap;
	struct admv1014_dev *dev;
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*dev));
	if (!indio_dev)
		return -ENOMEM;

	regmap = devm_regmap_init(&spi->dev, &admv1014_regmap_bus,
				  &spi->dev, &admv1014_regmap_config);
	if (IS_ERR(regmap)) {
		dev_err(&spi->dev, "invalid regmap");
		return PTR_ERR(regmap);
	}

	dev = iio_priv(indio_dev);
	dev->regmap = regmap;
	dev->spi = spi;

	ret = of_property_read_u8(spi->dev.of_node, "adi,quad-se-mode", &dev->quad_se_mode);
	if (ret < 0) {
		dev_err(&spi->dev, "adi,quad-se-mode property not defined!");
		return -EINVAL;
	}

	indio_dev->dev.parent = &spi->dev;
	indio_dev->info = &admv1014_info;
	indio_dev->name = "admv1014";

	dev->clkin = devm_clk_get(&spi->dev, "lo_in");
	if (IS_ERR(dev->clkin)) {
		return PTR_ERR(dev->clkin);
	}

	ret = clk_prepare_enable(dev->clkin);
	if (ret < 0) {
		return ret;
	}

	ret = devm_add_action_or_reset(&spi->dev, admv1014_clk_disable, dev);
	if (ret < 0) {
		return ret;
	}

	of_clk_get_scale(spi->dev.of_node, "lo_in", dev->clkscale);

	dev->clkin_freq = clk_get_rate_scaled(dev->clkin, dev->clkscale);
	dev->nb.notifier_call = admv1014_freq_change;
	ret = clk_notifier_register(dev->clkin, &dev->nb);
	if (ret)
		return ret;

	ret = devm_add_action_or_reset(&spi->dev, admv1014_clk_notifier_unreg, dev);
	if (ret < 0) {
		return ret;
	}

	// ret = admv1014_init(dev);
	// if (ret < 0) {
	// 	dev_err(&spi->dev, "admv1014 init failed\n");
	// 	return ret;
	// }

	return devm_iio_device_register(&spi->dev, indio_dev);
}

static const struct spi_device_id admv1014_id[] = {
	{ "admv1014", ADMV1014 },
	{}
};
MODULE_DEVICE_TABLE(spi, admv1014_id);

static const struct of_device_id admv1014_of_match[] = {
	{ .compatible = "adi,admv1014" },
	{},
};
MODULE_DEVICE_TABLE(of, admv1014_of_match);

static struct spi_driver admv1014_driver = {
	.driver = {
			.name = "admv1014",
			.of_match_table = admv1014_of_match,
		},
	.probe = admv1014_probe,
	.id_table = admv1014_id,
};
module_spi_driver(admv1014_driver);


MODULE_AUTHOR("Antoniu Miclaus <antoniu.miclaus@analog.com");
MODULE_DESCRIPTION("Analog Devices ADMV1014");
MODULE_LICENSE("GPL v2");