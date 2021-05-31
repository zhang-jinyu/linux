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
#include <linux/delay.h>
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
#define ADRF6780_ADC_STATUS_MSK     		BIT(8)
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

static int adrf6780_spi_update_bits(struct adrf6780_dev *dev, unsigned int reg,
			       unsigned int mask, unsigned int val)
{
	int ret;
	unsigned int data, temp;

	ret = adrf6780_spi_read(dev, reg, &data);
	if (ret < 0)
		return ret;

	temp = data & ~mask;
	temp |= val & mask;

	return adrf6780_spi_write(dev, reg, temp);
}

static int adrf6780_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int *val, int *val2, long info)
{
	struct adrf6780_dev *dev = iio_priv(indio_dev);
	unsigned int temp;
	int ret;

	switch (info) {
	case IIO_CHAN_INFO_RAW:
		ret = adrf6780_spi_update_bits(dev, ADRF6780_REG_ENABLE,
						ADRF6780_DETECTOR_EN_MSK,
						ADRF6780_DETECTOR_EN(1));
		if (ret < 0)
			return ret;

		ret = adrf6780_spi_update_bits(dev, ADRF6780_REG_ADC_CONTROL,
						ADRF6780_ADC_EN_MSK,
						ADRF6780_ADC_EN(1));
		if (ret < 0)
			return ret;
		
		ret = adrf6780_spi_update_bits(dev, ADRF6780_REG_ADC_CONTROL,
						ADRF6780_ADC_CLOCK_EN_MSK,
						ADRF6780_ADC_CLOCK_EN(1));
		if (ret < 0)
			return ret;
		
		ret = adrf6780_spi_update_bits(dev, ADRF6780_REG_ADC_CONTROL,
						ADRF6780_ADC_START_MSK,
						ADRF6780_ADC_START(1));
		if (ret < 0)
			return ret;

		udelay(200);

		ret = adrf6780_spi_read(dev, ADRF6780_REG_ADC_OUTPUT, &temp);
		if (ret < 0)
			return ret;

		if (!(temp & ADRF6780_ADC_STATUS_MSK))
			return -EINVAL;
		
		ret = adrf6780_spi_update_bits(dev, ADRF6780_REG_ADC_CONTROL,
						ADRF6780_ADC_START_MSK,
						ADRF6780_ADC_START(0));
		
		ret = adrf6780_spi_read(dev, ADRF6780_REG_ADC_OUTPUT, &temp);
		if (ret < 0)
			return ret;
		
		*val = temp & ADRF6780_ADC_VALUE_MSK;

		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}
}

enum adrf6780_iio_dev_attr {
	RDAC_LINEARIZE,
	Q_PATH_PHASE_ACCURACY,
	I_PATH_PHASE_ACCURACY,
	LO_SIDEBAND,
	VGA_BUFFER_ENABLE,
	DETECTOR_ENABLE ,
	LO_BUFFER_ENABLE,
	IF_MODE_ENABLE,
	IQ_MODE_ENABLE,
	LO_X2_ENABLE,
	LO_PPF_ENABLE,
	LO_ENABLE,
	UC_BIAS_ENABLE
};

static ssize_t adrf6780_store(struct device *device,
			      struct device_attribute *attr,
			      const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(device);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct adrf6780_dev *dev = iio_priv(indio_dev);
	u16 mask = 0, val = 0;
	u8 reg = 0;
	int ret = 0;

	ret = kstrtou16(buf, 10, &val);
	if (ret)
		return ret;

	switch ((u32)this_attr->address) {
	case RDAC_LINEARIZE:
		reg = ADRF6780_REG_LINEARIZE;
		mask = ADRF6780_RDAC_LINEARIZE_MSK;
		break;
	case Q_PATH_PHASE_ACCURACY:
		reg = ADRF6780_REG_LO_PATH;
		mask = ADRF6780_Q_PATH_PHASE_ACCURACY_MSK;
		val = ADRF6780_Q_PATH_PHASE_ACCURACY(val);
		break;
	case I_PATH_PHASE_ACCURACY:
		reg = ADRF6780_REG_LO_PATH;
		mask = ADRF6780_I_PATH_PHASE_ACCURACY_MSK;
		val = ADRF6780_I_PATH_PHASE_ACCURACY(val);
		break;
	case LO_SIDEBAND:
		reg = ADRF6780_REG_LO_PATH;
		mask = ADRF6780_LO_SIDEBAND_MSK;
		val = ADRF6780_LO_SIDEBAND(val);
		break;
	case VGA_BUFFER_ENABLE:
		reg = ADRF6780_REG_ENABLE;
		mask = ADRF6780_VGA_BUFFER_EN_MSK;
		val = ADRF6780_VGA_BUFFER_EN(val);
		break;
	case DETECTOR_ENABLE:
		reg = ADRF6780_REG_ENABLE;
		mask = ADRF6780_DETECTOR_EN_MSK;
		val = ADRF6780_DETECTOR_EN(val);
		break;
	case LO_BUFFER_ENABLE:
		reg = ADRF6780_REG_ENABLE;
		mask = ADRF6780_LO_BUFFER_EN_MSK;
		val = ADRF6780_LO_BUFFER_EN(val);
		break;
	case IF_MODE_ENABLE:
		reg = ADRF6780_REG_ENABLE;
		mask = ADRF6780_IF_MODE_EN_MSK;
		val = ADRF6780_IF_MODE_EN(val);
		break;
	case IQ_MODE_ENABLE:
		reg = ADRF6780_REG_ENABLE;
		mask = ADRF6780_IQ_MODE_EN_MSK;
		val = ADRF6780_IQ_MODE_EN(val);
		break;
	case LO_X2_ENABLE:
		reg = ADRF6780_REG_ENABLE;
		mask = ADRF6780_LO_X2_EN_MSK;
		val = ADRF6780_LO_X2_EN(val);
		break;
	case LO_PPF_ENABLE:
		reg = ADRF6780_REG_ENABLE;
		mask = ADRF6780_LO_PPF_EN_MSK;
		val = ADRF6780_LO_PPF_EN(val);
		break;
	case LO_ENABLE:
		reg = ADRF6780_REG_ENABLE;
		mask = ADRF6780_LO_EN_MSK;
		val = ADRF6780_LO_EN(val);
		break;
	case UC_BIAS_ENABLE:
		reg = ADRF6780_REG_ENABLE;
		mask = ADRF6780_UC_BIAS_EN_MSK;
		val = ADRF6780_UC_BIAS_EN(val);
		break;
	default:
		return -EINVAL;
	}

	ret = adrf6780_spi_update_bits(dev, reg, mask, val);

	return ret ? ret : len;
}

static ssize_t adrf6780_show(struct device *device,
			struct device_attribute *attr,
			char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(device);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct adrf6780_dev *dev = iio_priv(indio_dev);
	int ret = 0;
	u16 mask = 0, data_shift = 0;
	u32 val = 0;
	u8 reg = 0;

	switch ((u32)this_attr->address) {
	case RDAC_LINEARIZE:
		reg = ADRF6780_REG_LINEARIZE;
		mask = ADRF6780_RDAC_LINEARIZE_MSK;
		break;
	case Q_PATH_PHASE_ACCURACY:
		reg = ADRF6780_REG_LO_PATH;
		mask = ADRF6780_Q_PATH_PHASE_ACCURACY_MSK;
		data_shift = 4;
		break;
	case I_PATH_PHASE_ACCURACY:
		reg = ADRF6780_REG_LO_PATH;
		mask = ADRF6780_I_PATH_PHASE_ACCURACY_MSK;
		break;
	case LO_SIDEBAND:
		reg = ADRF6780_REG_LO_PATH;
		mask = ADRF6780_LO_SIDEBAND_MSK;
		data_shift = 10;
		break;
	case VGA_BUFFER_ENABLE:
		reg = ADRF6780_REG_ENABLE;
		mask = ADRF6780_VGA_BUFFER_EN_MSK;
		data_shift = 8;
		break;
	case DETECTOR_ENABLE:
		reg = ADRF6780_REG_ENABLE;
		mask = ADRF6780_DETECTOR_EN_MSK;
		data_shift = 7;
		break;
	case LO_BUFFER_ENABLE:
		reg = ADRF6780_REG_ENABLE;
		mask = ADRF6780_LO_BUFFER_EN_MSK;
		data_shift = 6;
		break;
	case IF_MODE_ENABLE:
		reg = ADRF6780_REG_ENABLE;
		mask = ADRF6780_IF_MODE_EN_MSK;
		data_shift = 5;
		break;
	case IQ_MODE_ENABLE:
		reg = ADRF6780_REG_ENABLE;
		mask = ADRF6780_IQ_MODE_EN_MSK;
		data_shift = 4;
		break;
	case LO_X2_ENABLE:
		reg = ADRF6780_REG_ENABLE;
		mask = ADRF6780_LO_X2_EN_MSK;
		data_shift = 3;
		break;
	case LO_PPF_ENABLE:
		reg = ADRF6780_REG_ENABLE;
		mask = ADRF6780_LO_PPF_EN_MSK;
		data_shift = 2;
		break;
	case LO_ENABLE:
		reg = ADRF6780_REG_ENABLE;
		mask = ADRF6780_LO_EN_MSK;
		data_shift = 1;
		break;
	case UC_BIAS_ENABLE:
		reg = ADRF6780_REG_ENABLE;
		mask = ADRF6780_UC_BIAS_EN_MSK;
		break;
	default:
		return -EINVAL;
	}

	ret = adrf6780_spi_read(dev, reg, &val);
	if (ret < 0)
		return ret;

	val = (val & mask) >> data_shift;

	return sprintf(buf, "%d\n", val);
}

static IIO_DEVICE_ATTR(rdac_linearize, S_IRUGO | S_IWUSR,
		       adrf6780_show,
		       adrf6780_store,
		       RDAC_LINEARIZE);

static IIO_DEVICE_ATTR(q_path_phase_accuracy, S_IRUGO | S_IWUSR,
		       adrf6780_show,
		       adrf6780_store,
		       Q_PATH_PHASE_ACCURACY);

static IIO_DEVICE_ATTR(i_path_phase_accuracy, S_IRUGO | S_IWUSR,
		       adrf6780_show,
		       adrf6780_store,
		       I_PATH_PHASE_ACCURACY);

static IIO_DEVICE_ATTR(lo_sideband, S_IRUGO | S_IWUSR,
		       adrf6780_show,
		       adrf6780_store,
		       LO_SIDEBAND);

static IIO_DEVICE_ATTR(vga_buffer_enable, S_IRUGO | S_IWUSR,
		       adrf6780_show,
		       adrf6780_store,
		       VGA_BUFFER_ENABLE);

static IIO_DEVICE_ATTR(detector_enable, S_IRUGO | S_IWUSR,
		       adrf6780_show,
		       adrf6780_store,
		       DETECTOR_ENABLE);

static IIO_DEVICE_ATTR(lo_buffer_enable, S_IRUGO | S_IWUSR,
		       adrf6780_show,
		       adrf6780_store,
		       LO_BUFFER_ENABLE);

static IIO_DEVICE_ATTR(if_mode_enable, S_IRUGO | S_IWUSR,
		       adrf6780_show,
		       adrf6780_store,
		       IF_MODE_ENABLE);

static IIO_DEVICE_ATTR(iq_mode_enable, S_IRUGO | S_IWUSR,
		       adrf6780_show,
		       adrf6780_store,
		       IQ_MODE_ENABLE);

static IIO_DEVICE_ATTR(lo_x2_enable, S_IRUGO | S_IWUSR,
		       adrf6780_show,
		       adrf6780_store,
		       LO_X2_ENABLE);

static IIO_DEVICE_ATTR(lo_ppf_enable, S_IRUGO | S_IWUSR,
		       adrf6780_show,
		       adrf6780_store,
		       LO_PPF_ENABLE);

static IIO_DEVICE_ATTR(lo_enable, S_IRUGO | S_IWUSR,
		       adrf6780_show,
		       adrf6780_store,
		       LO_ENABLE);

static IIO_DEVICE_ATTR(uc_bias_enable, S_IRUGO | S_IWUSR,
		       adrf6780_show,
		       adrf6780_store,
		       UC_BIAS_ENABLE);

static struct attribute *adrf6780_attributes[] = {
	&iio_dev_attr_rdac_linearize.dev_attr.attr,
	&iio_dev_attr_q_path_phase_accuracy.dev_attr.attr,
	&iio_dev_attr_i_path_phase_accuracy.dev_attr.attr,
	&iio_dev_attr_lo_sideband.dev_attr.attr,
	&iio_dev_attr_vga_buffer_enable.dev_attr.attr,
	&iio_dev_attr_detector_enable.dev_attr.attr,
	&iio_dev_attr_lo_buffer_enable.dev_attr.attr,
	&iio_dev_attr_if_mode_enable.dev_attr.attr,
	&iio_dev_attr_iq_mode_enable.dev_attr.attr,
	&iio_dev_attr_lo_x2_enable.dev_attr.attr,
	&iio_dev_attr_lo_ppf_enable.dev_attr.attr,
	&iio_dev_attr_lo_enable.dev_attr.attr,
	&iio_dev_attr_uc_bias_enable.dev_attr.attr,
	NULL
};

static const struct attribute_group adrf6780_attribute_group = {
	.attrs = adrf6780_attributes,
};

static int adrf6780_reg_access(struct iio_dev *indio_dev,
				unsigned int reg,
				unsigned int write_val,
				unsigned int *read_val)
{
	struct adrf6780_dev *dev = iio_priv(indio_dev);
	int ret;

	mutex_lock(&indio_dev->mlock);
	spi_bus_lock(dev->spi->master);
	dev->bus_locked = true;

	if (read_val)
		ret = adrf6780_spi_read(dev, reg, read_val);
	else
		ret = adrf6780_spi_write(dev, reg, write_val);

	dev->bus_locked = false;
	spi_bus_unlock(dev->spi->master);
	mutex_unlock(&indio_dev->mlock);

	return ret;
}

static const struct iio_info adrf6780_info = {
	.read_raw = adrf6780_read_raw,
	.debugfs_reg_access = &adrf6780_reg_access,
	.attrs = &adrf6780_attribute_group,
};

static int adrf6780_freq_change(struct notifier_block *nb, unsigned long flags, void *data)
{
	struct adrf6780_dev *dev = container_of(nb, struct adrf6780_dev, nb);
	struct clk_notifier_data *cnd = data;

	/* cache the new rate */
	dev->clkin_freq = clk_get_rate_scaled(cnd->clk, dev->clkscale);

	return NOTIFY_OK;
}

static void adrf6780_clk_notifier_unreg(void *data)
{
	struct adrf6780_dev *dev = data;

	clk_notifier_unregister(dev->clkin, &dev->nb);
}

#define ADRF6780_CHAN(_channel) {			\
	.type = IIO_VOLTAGE,				\
	.output = 1,					\
	.indexed = 1,					\
	.channel = _channel,				\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW)	\
}

static const struct iio_chan_spec adrf6780_channels[] = {
	ADRF6780_CHAN(0),
};

static int adrf6780_init(struct adrf6780_dev *dev)
{
	int ret;
	u32 chip_id;

	/* Perform a software reset */
	ret = adrf6780_spi_update_bits(dev, ADRF6780_REG_CONTROL,
				 ADRF6780_SOFT_RESET_MSK,
				 ADRF6780_SOFT_RESET(1));
	if (ret < 0)
		return ret;
	
	ret = adrf6780_spi_update_bits(dev, ADRF6780_REG_CONTROL,
				 ADRF6780_PARITY_EN_MSK,
				 ADRF6780_PARITY_EN(dev->parity_en));
	if (ret < 0)
		return ret;

	ret = adrf6780_spi_read(dev, ADRF6780_REG_CONTROL, &chip_id);
	if (ret < 0)
		return ret;

	chip_id = (chip_id & ADRF6780_CHIP_ID_MSK) >> 4;
	if (chip_id != ADRF6780_CHIP_ID)
		return -EINVAL;

	return ret;
}

static void adrf6780_clk_disable(void *data)
{
	struct adrf6780_dev *dev = data;

	clk_disable_unprepare(dev->clkin);
}

static int adrf6780_probe(struct spi_device *spi)
{
	struct iio_dev *indio_dev;
	struct adrf6780_dev *dev;
	struct clock_scale dev_clkscale;
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*dev));
	if (!indio_dev)
		return -ENOMEM;

	dev = iio_priv(indio_dev);

	dev->parity_en = of_property_read_bool(spi->dev.of_node, "adi,parity-enable");

	indio_dev->dev.parent = &spi->dev;
	indio_dev->info = &adrf6780_info;
	indio_dev->name = "adrf6780";
	indio_dev->channels = adrf6780_channels;
	indio_dev->num_channels = ARRAY_SIZE(adrf6780_channels);

	dev->spi = spi;

	dev->clkin = devm_clk_get(&spi->dev, "lo_in");
	if (IS_ERR(dev->clkin)) {
		return PTR_ERR(dev->clkin);
	}

	ret = clk_prepare_enable(dev->clkin);
	if (ret < 0) {
		return ret;
	}

	ret = devm_add_action_or_reset(&spi->dev, adrf6780_clk_disable, dev);
	if (ret < 0) {
		return ret;
	}

	of_clk_get_scale(spi->dev.of_node, "lo_in", &dev_clkscale);

	dev->clkscale = &dev_clkscale;

	dev->clkin_freq = clk_get_rate_scaled(dev->clkin, dev->clkscale);
	dev->nb.notifier_call = adrf6780_freq_change;
	ret = clk_notifier_register(dev->clkin, &dev->nb);
	if (ret)
		return ret;

	ret = devm_add_action_or_reset(&spi->dev, adrf6780_clk_notifier_unreg, dev);
	if (ret < 0) {
		return ret;
	}

	return devm_iio_device_register(&spi->dev, indio_dev);
}


static int adrf6780_remove(struct spi_device *spi)
{
	struct iio_dev *indio_dev = spi_get_drvdata(spi);

	iio_device_unregister(indio_dev);

	return 0;
}

static const struct spi_device_id adrf6780_id[] = {
	{ "adrf6780", ADRF6780 },
	{}
};
MODULE_DEVICE_TABLE(spi, adrf6780_id);

static const struct of_device_id adrf6780_of_match[] = {
	{ .compatible = "adi,adrf6780" },
	{},
};
MODULE_DEVICE_TABLE(of, adrf6780_of_match);

static struct spi_driver adrf6780_driver = {
	.driver = {
			.name = "adrf6780",
			.of_match_table = adrf6780_of_match,
		},
	.probe = adrf6780_probe,
	.remove = adrf6780_remove,
	.id_table = adrf6780_id,
};
module_spi_driver(adrf6780_driver);

MODULE_AUTHOR("Antoniu Miclaus <antoniu.miclaus@analog.com");
MODULE_DESCRIPTION("Analog Devices ADRF6780");
MODULE_LICENSE("GPL v2");
