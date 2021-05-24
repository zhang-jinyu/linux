// SPDX-License-Identifier: GPL-2.0+
/*
 * ADMV1013 driver
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
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>

#include <linux/iio/sysfs.h>

/* ADMV1013 Register Map */
#define ADMV1013_REG_SPI_CONTROL		0x00
#define ADMV1013_REG_ALARM 			0x01
#define ADMV1013_REG_ALARM_MASKS		0x02
#define ADMV1013_REG_ENABLE			0x03
#define ADMV1013_REG_LO_AMP_I			0x05
#define ADMV1013_REG_LO_AMP_Q			0x06
#define ADMV1013_REG_OFFSET_ADJUST_I 		0x07
#define ADMV1013_REG_OFFSET_ADJUST_Q		0x08
#define ADMV1013_REG_QUAD			0x09
#define ADMV1013_REG_VVA_TEMP_COMP		0x0A

/* ADMV1013_REG_SPI_CONTROL Map */
#define ADMV1013_PARITY_EN_MSK       		BIT(15)
#define ADMV1013_PARITY_EN(x)         		FIELD_PREP(ADMV1013_PARITY_EN_MSK, x)
#define ADMV1013_SPI_SOFT_RESET_MSK		BIT(14)
#define ADMV1013_SPI_SOFT_RESET(x)         	FIELD_PREP(ADMV1013_SPI_SOFT_RESET_MSK, x)
#define ADMV1013_CHIP_ID_MSK			GENMASK(11, 4)
#define ADMV1013_CHIP_ID             		0xA
#define ADMV1013_REVISION_ID_MSK		GENMASK(3, 0)
#define ADMV1013_REVISION_ID(x)        		FIELD_PREP(ADMV1013_REVISION_ID_MSK, x)

/* ADMV1013_REG_ALARM Map */
#define ADMV1013_PARITY_ERROR_MSK       	BIT(15)
#define ADMV1013_PARITY_ERROR(x)         	FIELD_PREP(ADMV1013_PARITY_ERROR_MSK, x)
#define ADMV1013_TOO_FEW_ERRORS_MSK		BIT(14)
#define ADMV1013_TOO_FEW_ERRORS(x)         	FIELD_PREP(ADMV1013_TOO_FEW_ERRORS_MSK, x)
#define ADMV1013_TOO_MANY_ERRORS_MSK		BIT(13)
#define ADMV1013_TOO_MANY_ERRORS(x)         	FIELD_PREP(ADMV1013_TOO_MANY_ERRORS_MSK, x)
#define ADMV1013_ADDRESS_RANGE_ERROR_MSK	BIT(12)
#define ADMV1013_ADDRESS_RANGE_ERROR(x)         FIELD_PREP(ADMV1013_ADDRESS_RANGE_ERROR_MSK, x)

/* ADMV1013_REG_ENABLE Map */
#define ADMV1013_VGA_PD_MSK       		BIT(15)
#define ADMV1013_VGA_PD(x)         		FIELD_PREP(ADMV1013_VGA_PD_MSK, x)
#define ADMV1013_MIXER_PD_MSK       		BIT(14)
#define ADMV1013_MIXER_PD(x)         		FIELD_PREP(ADMV1013_MIXER_PD_MSK, x)
#define ADMV1013_QUAD_PD_MSK			GENMASK(13, 11)
#define ADMV1013_QUAD_PD(x)         		FIELD_PREP(ADMV1013_QUAD_PD_MSK, x)
#define ADMV1013_BG_PD_MSK			BIT(10)
#define ADMV1013_BG_PD(x)         		FIELD_PREP(ADMV1013_BG_PD_MSK, x)
#define ADMV1013_MIXER_IF_EN_MSK		BIT(7)
#define ADMV1013_MIXER_IF_EN(x)         	FIELD_PREP(ADMV1013_MIXER_IF_EN_MSK, x)
#define ADMV1013_DET_EN_MSK			BIT(5)
#define ADMV1013_DET_EN(x)         		FIELD_PREP(ADMV1013_DET_EN_MSK, x)

/* ADMV1013_REG_LO_AMP_I Map */
#define ADMV1013_LOAMP_PH_ADJ_I_FINE_MSK	GENMASK(13, 7)
#define ADMV1013_LOAMP_PH_ADJ_I_FINE(x)        	FIELD_PREP(ADMV1013_LOAMP_PH_ADJ_I_FINE_MSK, x)
#define ADMV1013_MIXER_VGATE_MSK		GENMASK(6, 0)
#define ADMV1013_MIXER_VGATE(x)         	FIELD_PREP(ADMV1013_MIXER_VGATE_MSK, x)

/* ADMV1013_REG_LO_AMP_Q Map */
#define ADMV1013_LOAMP_PH_ADJ_Q_FINE_MSK	GENMASK(13, 7)
#define ADMV1013_LOAMP_PH_ADJ_Q_FINE(x)        	FIELD_PREP(ADMV1013_LOAMP_PH_ADJ_Q_FINE_MSK, x)

/* ADMV1013_REG_OFFSET_ADJUST_I Map */
#define ADMV1013_MIXER_OFF_ADJ_I_P_MSK		GENMASK(15, 9)
#define ADMV1013_MIXER_OFF_ADJ_I_P(x)        	FIELD_PREP(ADMV1013_MIXER_OFF_ADJ_I_P_MSK, x)
#define ADMV1013_MIXER_OFF_ADJ_I_N_MSK		GENMASK(8, 2)
#define ADMV1013_MIXER_OFF_ADJ_I_N(x)        	FIELD_PREP(ADMV1013_MIXER_OFF_ADJ_I_N_MSK, x)

/* ADMV1013_REG_OFFSET_ADJUST_Q Map */
#define ADMV1013_MIXER_OFF_ADJ_Q_P_MSK		GENMASK(15, 9)
#define ADMV1013_MIXER_OFF_ADJ_Q_P(x)        	FIELD_PREP(ADMV1013_MIXER_OFF_ADJ_Q_P_MSK, x)
#define ADMV1013_MIXER_OFF_ADJ_Q_N_MSK		GENMASK(8, 2)
#define ADMV1013_MIXER_OFF_ADJ_Q_N(x)        	FIELD_PREP(ADMV1013_MIXER_OFF_ADJ_Q_N_MSK, x)

/* ADMV1013_REG_QUAD Map */
#define ADMV1013_QUAD_SE_MODE_MSK              GENMASK(9, 6)
#define ADMV1013_QUAD_SE_MODE(x)               FIELD_PREP(ADMV1013_QUAD_SE_MODE_MSK, x)
#define ADMV1013_QUAD_FILTERS_MSK              GENMASK(3, 0)
#define ADMV1013_QUAD_FILTERS(x)               FIELD_PREP(ADMV1013_QUAD_FILTERS_MSK, x)

/* ADMV1013_REG_VVA_TEMP_COMP Map */
#define ADMV1013_VVA_TEMP_COMP_MSK		GENMASK(15, 0)
#define ADMV1013_VVA_TEMP_COMP(x)  		FIELD_PREP(ADMV1013_VVA_TEMP_COMP_MSK, x)

enum supported_parts {
	ADMV1013,
};

struct admv1013_dev {
	struct regmap		*regmap;
	struct clk 		*clkin;
	struct clock_scale	*clkscale;
	struct regulator	*reg;
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

static int admv1013_spi_read(struct admv1013_dev *dev, unsigned int reg,
			      unsigned int *val)
{
	int ret;
	unsigned int cnt, p_bit, data;

	ret = regmap_read(dev->regmap, reg, &data);
	if (ret < 0)
		return ret;

	data = (reg << 17) | data;

	if (dev->parity_en)
	{
		check_parity(data, &cnt);
		p_bit = data & 0x1;

		if ((!(cnt % 2) && p_bit) || ((cnt % 2) && !p_bit))
			return -EINVAL;
	}

	*val = (data >> 1) & 0xFF;

	return ret;
}

static int admv1013_spi_write(struct admv1013_dev *dev,
				      unsigned int reg,
				      unsigned int val)
{
	unsigned int cnt;

	val = (val << 1);

	if (dev->parity_en)
	{
		check_parity(val, &cnt);

		if (cnt % 2 == 0)
			val |= 0x1;
	}

	return regmap_write(dev->regmap, reg, val);
}

static int admv1013_spi_update_bits(struct admv1013_dev *dev, unsigned int reg,
			       unsigned int mask, unsigned int val)
{
	int ret;
	unsigned int data, temp;

	ret = admv1013_spi_read(dev, reg, &data);
	if (ret < 0)
		return ret;

	temp = data & ~mask;
	temp |= val & mask;

	return admv1013_spi_write(dev, reg, temp);
}

static int admv1013_update_quad_filters(struct admv1013_dev *dev)
{
	unsigned int filt_raw;

	if (dev->clkin_freq <= 6600000000 && dev->clkin_freq <= 9200000000)
		filt_raw = 5;
	else if (dev->clkin_freq <= 5400000000 && dev->clkin_freq <= 8000000000)
		filt_raw = 10;
	else if (dev->clkin_freq <= 5400000000 && dev->clkin_freq <= 7000000000)
		filt_raw = 15;
	else
		filt_raw = 0;

	return admv1013_spi_update_bits(dev, ADMV1013_REG_QUAD,
					ADMV1013_QUAD_FILTERS_MSK,
					ADMV1013_QUAD_FILTERS(filt_raw));
}

static const struct regmap_config admv1013_regmap_config = {
	.reg_bits = 7,
	.val_bits = 17,
	.read_flag_mask = BIT(7),
	.max_register = 0x0B,
};

enum admv1013_iio_dev_attr {
	MIXER_OFF_ADJ_I_P,
	MIXER_OFF_ADJ_I_N,
	MIXER_OFF_ADJ_Q_P,
	MIXER_OFF_ADJ_Q_N,
	LOAMP_PH_ADJ_I_FINE,
	LOAMP_PH_ADJ_Q_FINE,
	VGA_PD,
	MIXER_PD,
	QUAD_PD,
	BG_PD,
	MIXER_IF_EN,
	DET_EN
};

static ssize_t admv1013_store(struct device *device,
			      struct device_attribute *attr,
			      const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(device);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct admv1013_dev *dev = iio_priv(indio_dev);
	u16 mask = 0, val = 0;
	u8 reg = 0;
	int ret = 0;

	ret = kstrtou16(buf, 10, &val);
	if (ret)
		return ret;

	switch ((u32)this_attr->address) {
	case MIXER_OFF_ADJ_I_P:
		reg = ADMV1013_REG_OFFSET_ADJUST_I;
		mask = ADMV1013_MIXER_OFF_ADJ_I_P_MSK;
		val = ADMV1013_MIXER_OFF_ADJ_I_P(val);
		break;
	case MIXER_OFF_ADJ_I_N:
		reg = ADMV1013_REG_OFFSET_ADJUST_I;
		mask = ADMV1013_MIXER_OFF_ADJ_I_N_MSK;
		val = ADMV1013_MIXER_OFF_ADJ_I_N(val);
		break;
	case MIXER_OFF_ADJ_Q_P:
		reg = ADMV1013_REG_OFFSET_ADJUST_Q;
		mask = ADMV1013_MIXER_OFF_ADJ_Q_P_MSK;
		val = ADMV1013_MIXER_OFF_ADJ_Q_P(val);
		break;
	case MIXER_OFF_ADJ_Q_N:
		reg = ADMV1013_REG_OFFSET_ADJUST_Q;
		mask = ADMV1013_MIXER_OFF_ADJ_Q_N_MSK;
		val = ADMV1013_MIXER_OFF_ADJ_Q_N(val);
		break;
	case LOAMP_PH_ADJ_I_FINE:
		reg = ADMV1013_REG_LO_AMP_I;
		mask = ADMV1013_LOAMP_PH_ADJ_I_FINE_MSK;
		val = ADMV1013_LOAMP_PH_ADJ_I_FINE(val);
		break;
	case LOAMP_PH_ADJ_Q_FINE:
		reg = ADMV1013_REG_LO_AMP_Q;
		mask = ADMV1013_LOAMP_PH_ADJ_Q_FINE_MSK;
		val = ADMV1013_LOAMP_PH_ADJ_Q_FINE(val);
		break;
	case VGA_PD:
		reg = ADMV1013_REG_ENABLE;
		mask = ADMV1013_VGA_PD_MSK;
		val = ADMV1013_VGA_PD(val);
		break;
	case MIXER_PD:
		reg = ADMV1013_REG_ENABLE;
		mask = ADMV1013_MIXER_PD_MSK;
		val = ADMV1013_MIXER_PD(val);
		break;
	case QUAD_PD:
		if (val != 0x0 || val != 0xf)
			return -EINVAL;

		reg = ADMV1013_REG_ENABLE;
		mask = ADMV1013_QUAD_PD_MSK;
		val = ADMV1013_QUAD_PD(val);
		break;
	case BG_PD:
		reg = ADMV1013_REG_ENABLE;
		mask = ADMV1013_BG_PD_MSK;
		val = ADMV1013_BG_PD(val);
		break;
	case MIXER_IF_EN:
		reg = ADMV1013_REG_ENABLE;
		mask = ADMV1013_MIXER_IF_EN_MSK;
		val = ADMV1013_MIXER_IF_EN(val);
		break;
	case DET_EN:
		reg = ADMV1013_REG_ENABLE;
		mask = ADMV1013_DET_EN_MSK;
		val = ADMV1013_DET_EN(val);
		break;
	default:
		return -EINVAL;
	}

	ret = admv1013_spi_update_bits(dev, reg, mask, val);

	return ret ? ret : len;
}

static ssize_t admv1013_show(struct device *device,
			struct device_attribute *attr,
			char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(device);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct admv1013_dev *dev = iio_priv(indio_dev);
	int ret = 0;
	u16 mask = 0, data_shift = 0;
	u32 val = 0;
	u8 reg = 0;

	switch ((u32)this_attr->address) {
	case MIXER_OFF_ADJ_I_P:
		reg = ADMV1013_REG_OFFSET_ADJUST_I;
		mask = ADMV1013_MIXER_OFF_ADJ_I_P_MSK;
		data_shift = 9;
		break;
	case MIXER_OFF_ADJ_I_N:
		reg = ADMV1013_REG_OFFSET_ADJUST_I;
		mask = ADMV1013_MIXER_OFF_ADJ_I_N_MSK;
		data_shift = 2;
		break;
	case MIXER_OFF_ADJ_Q_P:
		reg = ADMV1013_REG_OFFSET_ADJUST_Q;
		mask = ADMV1013_MIXER_OFF_ADJ_Q_P_MSK;
		data_shift = 9;
		break;
	case MIXER_OFF_ADJ_Q_N:
		reg = ADMV1013_REG_OFFSET_ADJUST_Q;
		mask = ADMV1013_MIXER_OFF_ADJ_Q_N_MSK;
		data_shift = 2;
		break;
	case LOAMP_PH_ADJ_I_FINE:
		reg = ADMV1013_REG_LO_AMP_I;
		mask = ADMV1013_LOAMP_PH_ADJ_I_FINE_MSK;
		data_shift = 7;
		break;
	case LOAMP_PH_ADJ_Q_FINE:
		reg = ADMV1013_REG_LO_AMP_Q;
		mask = ADMV1013_LOAMP_PH_ADJ_Q_FINE_MSK;
		data_shift = 7;
		break;
	case VGA_PD:
		reg = ADMV1013_REG_ENABLE;
		mask = ADMV1013_VGA_PD_MSK;
		data_shift = 15;
		break;
	case MIXER_PD:
		reg = ADMV1013_REG_ENABLE;
		mask = ADMV1013_MIXER_PD_MSK;
		data_shift = 14;
		break;
	case QUAD_PD:
		reg = ADMV1013_REG_ENABLE;
		mask = ADMV1013_QUAD_PD_MSK;
		data_shift = 11;
		break;
	case BG_PD:
		reg = ADMV1013_REG_ENABLE;
		mask = ADMV1013_BG_PD_MSK;
		data_shift = 10;
		break;
	case MIXER_IF_EN:
		reg = ADMV1013_REG_ENABLE;
		mask = ADMV1013_MIXER_IF_EN_MSK;
		data_shift = 7;
		break;
	case DET_EN:
		reg = ADMV1013_REG_ENABLE;
		mask = ADMV1013_DET_EN_MSK;
		data_shift = 5;
		break;
	default:
		return -EINVAL;
	}

	ret = admv1013_spi_read(dev, reg, &val);
	if (ret < 0)
		return ret;

	val = (val & mask) >> data_shift;

	return sprintf(buf, "%d\n", val);
}

static IIO_DEVICE_ATTR(mixer_off_adj_i_p, S_IRUGO | S_IWUSR,
		       admv1013_show,
		       admv1013_store,
		       MIXER_OFF_ADJ_I_P);

static IIO_DEVICE_ATTR(mixer_off_adj_i_n, S_IRUGO | S_IWUSR,
		       admv1013_show,
		       admv1013_store,
		       MIXER_OFF_ADJ_I_N);

static IIO_DEVICE_ATTR(mixer_off_adj_q_p, S_IRUGO | S_IWUSR,
		       admv1013_show,
		       admv1013_store,
		       MIXER_OFF_ADJ_Q_P);

static IIO_DEVICE_ATTR(mixer_off_adj_q_n, S_IRUGO | S_IWUSR,
		       admv1013_show,
		       admv1013_store,
		       MIXER_OFF_ADJ_Q_N);

static IIO_DEVICE_ATTR(loamp_ph_adj_i_fine, S_IRUGO | S_IWUSR,
		       admv1013_show,
		       admv1013_store,
		       LOAMP_PH_ADJ_I_FINE);

static IIO_DEVICE_ATTR(loamp_ph_adj_q_fine, S_IRUGO | S_IWUSR,
		       admv1013_show,
		       admv1013_store,
		       LOAMP_PH_ADJ_Q_FINE);

static IIO_DEVICE_ATTR(vga_pd, S_IRUGO | S_IWUSR,
		       admv1013_show,
		       admv1013_store,
		       VGA_PD);

static IIO_DEVICE_ATTR(mixer_pd, S_IRUGO | S_IWUSR,
		       admv1013_show,
		       admv1013_store,
		       MIXER_PD);

static IIO_DEVICE_ATTR(quad_pd, S_IRUGO | S_IWUSR,
		       admv1013_show,
		       admv1013_store,
		       QUAD_PD);

static IIO_DEVICE_ATTR(bg_pd, S_IRUGO | S_IWUSR,
		       admv1013_show,
		       admv1013_store,
		       BG_PD);

static IIO_DEVICE_ATTR(mixer_if_en, S_IRUGO | S_IWUSR,
		       admv1013_show,
		       admv1013_store,
		       MIXER_IF_EN);

static IIO_DEVICE_ATTR(det_en, S_IRUGO | S_IWUSR,
		       admv1013_show,
		       admv1013_store,
		       DET_EN);

static struct attribute *admv1013_attributes[] = {
	&iio_dev_attr_mixer_off_adj_i_p.dev_attr.attr,
	&iio_dev_attr_mixer_off_adj_i_n.dev_attr.attr,
	&iio_dev_attr_mixer_off_adj_q_p.dev_attr.attr,
	&iio_dev_attr_mixer_off_adj_q_n.dev_attr.attr,
	&iio_dev_attr_loamp_ph_adj_i_fine.dev_attr.attr,
	&iio_dev_attr_loamp_ph_adj_q_fine.dev_attr.attr,
	&iio_dev_attr_vga_pd.dev_attr.attr,
	&iio_dev_attr_mixer_pd.dev_attr.attr,
	&iio_dev_attr_quad_pd.dev_attr.attr,
	&iio_dev_attr_bg_pd.dev_attr.attr,
	&iio_dev_attr_mixer_if_en.dev_attr.attr,
	&iio_dev_attr_det_en.dev_attr.attr,
	NULL
};

static const struct attribute_group admv1013_attribute_group = {
	.attrs = admv1013_attributes,
};

static int admv1013_reg_access(struct iio_dev *indio_dev,
				unsigned int reg,
				unsigned int write_val,
				unsigned int *read_val)
{
	struct admv1013_dev *dev = iio_priv(indio_dev);

	if (read_val)
		return admv1013_spi_read(dev, reg, read_val);
	else
		return admv1013_spi_write(dev, reg, write_val);
}

static const struct iio_info admv1013_info = {
	.debugfs_reg_access = &admv1013_reg_access,
	.attrs = &admv1013_attribute_group,
};

static int admv1013_freq_change(struct notifier_block *nb, unsigned long flags, void *data)
{
	struct admv1013_dev *dev = container_of(nb, struct admv1013_dev, nb);
	struct clk_notifier_data *cnd = data;
	int ret;

	/* cache the new rate */
	dev->clkin_freq = clk_get_rate_scaled(cnd->clk, dev->clkscale);

	ret = admv1013_update_quad_filters(dev);
	if(ret < 0)
		return ret;

	return NOTIFY_OK;
}

static void admv1013_clk_notifier_unreg(void *data)
{
	struct admv1013_dev *dev = data;

	clk_notifier_unregister(dev->clkin, &dev->nb);
}

static int admv1013_init(struct admv1013_dev *dev)
{
	int ret;
	u32 vcm, mixer_vgate;
	u32 chip_id;

	/* Perform a software reset */
	ret = admv1013_spi_update_bits(dev, ADMV1013_REG_SPI_CONTROL,
				 ADMV1013_SPI_SOFT_RESET_MSK,
				 ADMV1013_SPI_SOFT_RESET(1));
	if (ret < 0)
		return ret;

	ret = admv1013_spi_write(dev, ADMV1013_REG_VVA_TEMP_COMP, 0xE700);
	if (ret < 0)
		return ret;

	ret = admv1013_spi_read(dev, ADMV1013_REG_SPI_CONTROL, &chip_id);
	if (ret < 0)
		return ret;

	chip_id = (chip_id & ADMV1013_CHIP_ID_MSK) >> 4;
	if (chip_id != ADMV1013_CHIP_ID)
		return -EINVAL;

	vcm = regulator_get_voltage(dev->reg);

	if(vcm >= 0 && vcm < 1800000)
		mixer_vgate = (2389 * vcm / 1000000 + 8100) / 100;
	else if (vcm > 1800000 && vcm < 2600000)
		mixer_vgate = (2375 * vcm / 1000000 + 125) / 100;
	
	ret = admv1013_spi_update_bits(dev, ADMV1013_REG_LO_AMP_I,
				 ADMV1013_MIXER_VGATE_MSK,
				 ADMV1013_MIXER_VGATE(mixer_vgate));
	if (ret < 0)
		return ret;

	ret = admv1013_spi_update_bits(dev, ADMV1013_REG_QUAD,
				 ADMV1013_QUAD_SE_MODE_MSK,
				 ADMV1013_QUAD_SE_MODE(dev->quad_se_mode));
	if (ret < 0)
		return ret;

	return admv1013_update_quad_filters(dev);
}

static void admv1013_clk_disable(void *data)
{
	struct admv1013_dev *dev = data;

	clk_disable_unprepare(dev->clkin);
}

static int admv1013_probe(struct spi_device *spi)
{
	struct iio_dev *indio_dev;
	struct regmap *regmap;
	struct admv1013_dev *dev;
	struct clock_scale dev_clkscale;
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*dev));
	if (!indio_dev)
		return -ENOMEM;

	regmap = devm_regmap_init_spi(spi, &admv1013_regmap_config);
	if (IS_ERR(regmap)) {
		dev_err(&spi->dev, "invalid regmap");
		return PTR_ERR(regmap);
	}

	dev = iio_priv(indio_dev);
	dev->regmap = regmap;

	ret = of_property_read_u8(spi->dev.of_node, "adi,quad-se-mode", &dev->quad_se_mode);
	if (ret < 0) {
		dev_err(&spi->dev, "adi,quad-se-mode property not defined!");
		return -EINVAL;
	}

	dev->reg = devm_regulator_get(&spi->dev, "cmv");
	if (IS_ERR(dev->reg))
		return PTR_ERR(dev->reg);

	ret = regulator_enable(dev->reg);
	if (ret < 0) {
		dev_err(&spi->dev, "Failed to enable specified Common-Mode Voltage!\n");
		return ret;
	}

	indio_dev->dev.parent = &spi->dev;
	indio_dev->info = &admv1013_info;
	indio_dev->name = "admv1013";

	dev->clkin = devm_clk_get(&spi->dev, "lo_in");
	if (IS_ERR(dev->clkin)) {
		return PTR_ERR(dev->clkin);
	}

	ret = clk_prepare_enable(dev->clkin);
	if (ret < 0) {
		return ret;
	}

	ret = devm_add_action_or_reset(&spi->dev, admv1013_clk_disable, dev);
	if (ret < 0) {
		return ret;
	}

	of_clk_get_scale(spi->dev.of_node, "lo_in", &dev_clkscale);

	dev->clkscale = &dev_clkscale;

	dev->clkin_freq = clk_get_rate_scaled(dev->clkin, dev->clkscale);
	dev->nb.notifier_call = admv1013_freq_change;
	ret = clk_notifier_register(dev->clkin, &dev->nb);
	if (ret)
		return ret;

	ret = devm_add_action_or_reset(&spi->dev, admv1013_clk_notifier_unreg, dev);
	if (ret < 0) {
		return ret;
	}

	ret = admv1013_init(dev);
	if (ret < 0) {
		dev_err(&spi->dev, "admv1013 init failed\n");
		return ret;
	}

	dev_info(&spi->dev, "ADMV1013 PROBED");

	return devm_iio_device_register(&spi->dev, indio_dev);
}

static int admv1013_remove(struct spi_device *spi)
{
	struct iio_dev *indio_dev = spi_get_drvdata(spi);
	struct admv1013_dev *dev = iio_priv(indio_dev);

	iio_device_unregister(indio_dev);

	regulator_disable(dev->reg);

	return 0;
}

static const struct spi_device_id admv1013_id[] = {
	{ "admv1013", ADMV1013 },
	{}
};
MODULE_DEVICE_TABLE(spi, admv1013_id);

static const struct of_device_id admv1013_of_match[] = {
	{ .compatible = "adi,admv1013" },
	{},
};
MODULE_DEVICE_TABLE(of, admv1013_of_match);

static struct spi_driver admv1013_driver = {
	.driver = {
			.name = "admv1013",
			.of_match_table = admv1013_of_match,
		},
	.probe = admv1013_probe,
	.remove = admv1013_remove,
	.id_table = admv1013_id,
};
module_spi_driver(admv1013_driver);


MODULE_AUTHOR("Antoniu Miclaus <antoniu.miclaus@analog.com");
MODULE_DESCRIPTION("Analog Devices ADMV1013");
MODULE_LICENSE("GPL v2");