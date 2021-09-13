// SPDX-License-Identifier: GPL-2.0-only
/*
 * Aspeed AST2400/2500/2600 ADC
 *
 * Copyright (C) 2017 Google, Inc.
 * Copyright (C) 2021 Aspeed Technology Inc.
 */

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <linux/bitfield.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>

#include <linux/iio/iio.h>
#include <linux/iio/driver.h>
#include <linux/iopoll.h>

#define ASPEED_RESOLUTION_BITS		10
#define ASPEED_CLOCKS_PER_SAMPLE	12

#define ASPEED_REG_ENGINE_CONTROL	0x00
#define ASPEED_REG_INTERRUPT_CONTROL	0x04
#define ASPEED_REG_VGA_DETECT_CONTROL	0x08
#define ASPEED_REG_CLOCK_CONTROL	0x0C
#define ASPEED_REG_COMPENSATION_TRIM	0xC4
#define ASPEED_REG_MAX			0xCC

#define ASPEED_ADC_ENGINE_ENABLE		BIT(0)
#define ASPEED_ADC_OPERATION_MODE		GENMASK(3, 1)
#define ASPEED_ADC_OPERATION_MODE_POWER_DOWN	FIELD_PREP(ASPEED_ADC_OPERATION_MODE, 0)
#define ASPEED_ADC_OPERATION_MODE_STANDBY	FIELD_PREP(ASPEED_ADC_OPERATION_MODE, 1)
#define ASPEED_ADC_OPERATION_MODE_NORMAL	FIELD_PREP(ASPEED_ADC_OPERATION_MODE, 7)
#define ASPEED_ADC_CTRL_COMPENSATION		BIT(4)
#define ASPEED_ADC_AUTO_COMPENSATION		BIT(5)
#define ASPEED_ADC_REF_VOLTAGE			GENMASK(7, 6)
#define ASPEED_ADC_REF_VOLTAGE_2500mV		FIELD_PREP(ASPEED_ADC_REF_VOLTAGE, 0)
#define ASPEED_ADC_REF_VOLTAGE_1200mV		FIELD_PREP(ASPEED_ADC_REF_VOLTAGE, 1)
#define ASPEED_ADC_REF_VOLTAGE_EXT_HIGH		FIELD_PREP(ASPEED_ADC_REF_VOLTAGE, 2)
#define ASPEED_ADC_REF_VOLTAGE_EXT_LOW		FIELD_PREP(ASPEED_ADC_REF_VOLTAGE, 3)
#define ASPEED_ADC_BATTERY_SENSING_DIV		BIT(6)
#define ASPEED_ADC_BATTERY_SENSING_DIV_2_3	FIELD_PREP(ASPEED_ADC_BATTERY_SENSING_DIV, 0)
#define ASPEED_ADC_BATTERY_SENSING_DIV_1_3	FIELD_PREP(ASPEED_ADC_BATTERY_SENSING_DIV, 1)
#define ASPEED_ADC_CTRL_INIT_RDY		BIT(8)
#define ASPEED_ADC_CH7_MODE			BIT(12)
#define ASPEED_ADC_CH7_NORMAL			FIELD_PREP(ASPEED_ADC_CH7_MODE, 0)
#define ASPEED_ADC_CH7_BATTERY			FIELD_PREP(ASPEED_ADC_CH7_MODE, 1)
#define ASPEED_ADC_BATTERY_SENSING_ENABLE	BIT(13)
#define ASPEED_ADC_CTRL_CHANNEL			GENMASK(31, 16)
#define ASPEED_ADC_CTRL_CHANNEL_ENABLE(ch)	FIELD_PREP(ASPEED_ADC_CTRL_CHANNEL, BIT(ch))

#define ASPEED_ADC_INIT_POLLING_TIME	500
#define ASPEED_ADC_INIT_TIMEOUT		500000
#define ASPEED_ADC_DEF_SAMPLING_RATE	250000
#define ASPEED_ADC_MAX_RAW_DATA		GENMASK(9, 0)

struct aspeed_adc_trim_locate {
	const unsigned int offset;
	const unsigned int field;
};

enum aspeed_adc_version {
	aspeed_adc_ast2400,
	aspeed_adc_ast2500,
	aspeed_adc_ast2600,
};
struct aspeed_adc_model_data {
	enum aspeed_adc_version version;
	unsigned int min_sampling_rate;	// Hz
	unsigned int max_sampling_rate;	// Hz
	bool wait_init_sequence;
	unsigned int num_channels;
	const struct aspeed_adc_trim_locate *trim_locate;
};

struct adc_gain {
	u8 mult;
	u8 div;
};

struct aspeed_adc_data {
	struct device		*dev;
	void __iomem		*base;
	spinlock_t		clk_lock;
	struct clk_hw		*fixed_div_clk;
	struct clk_hw		*clk_prescaler;
	struct clk_hw		*clk_scaler;
	struct reset_control	*rst;
	int			vref;
	u32			sample_period_ns;
	int			cv;
	bool			battery_sensing;
	struct adc_gain		battery_mode_gain;
};

#define ASPEED_CHAN(_idx, _data_reg_addr) {			\
	.type = IIO_VOLTAGE,					\
	.indexed = 1,						\
	.channel = (_idx),					\
	.address = (_data_reg_addr),				\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),		\
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE) |	\
				BIT(IIO_CHAN_INFO_SAMP_FREQ),	\
}

static const struct iio_chan_spec aspeed_adc_iio_channels[] = {
	ASPEED_CHAN(0, 0x10),
	ASPEED_CHAN(1, 0x12),
	ASPEED_CHAN(2, 0x14),
	ASPEED_CHAN(3, 0x16),
	ASPEED_CHAN(4, 0x18),
	ASPEED_CHAN(5, 0x1A),
	ASPEED_CHAN(6, 0x1C),
	ASPEED_CHAN(7, 0x1E),
	ASPEED_CHAN(8, 0x20),
	ASPEED_CHAN(9, 0x22),
	ASPEED_CHAN(10, 0x24),
	ASPEED_CHAN(11, 0x26),
	ASPEED_CHAN(12, 0x28),
	ASPEED_CHAN(13, 0x2A),
	ASPEED_CHAN(14, 0x2C),
	ASPEED_CHAN(15, 0x2E),
};

static int aspeed_adc_set_trim_data(struct platform_device *pdev)
{
	struct device_node *syscon;
	struct regmap *scu;
	u32 scu_otp, trimming_val;
	struct iio_dev *indio_dev = platform_get_drvdata(pdev);
	struct aspeed_adc_data *data = iio_priv(indio_dev);
	const struct aspeed_adc_model_data *model_data =
		of_device_get_match_data(data->dev);

	syscon = of_find_node_by_name(NULL, "syscon");
	if (syscon == NULL) {
		dev_warn(data->dev, "Couldn't find syscon node\n");
		return -EOPNOTSUPP;
	}
	scu = syscon_node_to_regmap(syscon);
	if (IS_ERR(scu)) {
		dev_warn(data->dev, "Failed to get syscon regmap\n");
		return -EOPNOTSUPP;
	}
	if (model_data->trim_locate) {
		if (regmap_read(scu, model_data->trim_locate->offset,
				&scu_otp)) {
			dev_warn(data->dev,
				 "Failed to get adc trimming data\n");
			trimming_val = 0x8;
		} else {
			trimming_val = ((scu_otp) &
					(model_data->trim_locate->field)) >>
				       __ffs(model_data->trim_locate->field);
		}
	}
	dev_dbg(data->dev, "trimming val = %d, offset = %08x, fields = %08x\n",
		 trimming_val, model_data->trim_locate->offset,
		 model_data->trim_locate->field);
	writel(trimming_val, data->base + ASPEED_REG_COMPENSATION_TRIM);
	return 0;
}

static int aspeed_adc_compensation(struct platform_device *pdev)
{
	struct iio_dev *indio_dev = platform_get_drvdata(pdev);
	struct aspeed_adc_data *data = iio_priv(indio_dev);
	u32 index, adc_raw = 0;
	u32 adc_engine_control_reg_val =
		readl(data->base + ASPEED_REG_ENGINE_CONTROL);
	adc_engine_control_reg_val |=
		(ASPEED_ADC_OPERATION_MODE_NORMAL | ASPEED_ADC_ENGINE_ENABLE);

	/*
	 * Enable compensating sensing:
	 * After that, the input voltage of adc will force to half of the reference
	 * voltage. So the expected reading raw data will become half of the max
	 * value. We can get compensating value = 0x200 - adc read raw value.
	 * It is recommended to average at least 10 samples to get a final CV.
	 */
	writel(adc_engine_control_reg_val | ASPEED_ADC_CTRL_COMPENSATION |
		       ASPEED_ADC_CTRL_CHANNEL_ENABLE(0),
	       data->base + ASPEED_REG_ENGINE_CONTROL);
	/*
	 * After enable compensating sensing mode need to wait some time for adc stable
	 * Experiment result is 1ms.
	 */
	mdelay(1);

	for (index = 0; index < 16; index++) {
		/*
		 * Waiting for the sampling period ensures that the value acquired
		 * is fresh each time.
		 */
		ndelay(data->sample_period_ns);
		adc_raw += readw(data->base + aspeed_adc_iio_channels[0].address);
	}
	adc_raw >>= 4;
	data->cv = BIT(ASPEED_RESOLUTION_BITS - 1) - adc_raw;
	writel(adc_engine_control_reg_val,
	       data->base + ASPEED_REG_ENGINE_CONTROL);
	dev_dbg(data->dev, "compensating value = %d\n", data->cv);
	return 0;
}

static int aspeed_adc_set_sampling_rate(struct iio_dev *indio_dev, u32 rate)
{
	struct aspeed_adc_data *data = iio_priv(indio_dev);
	const struct aspeed_adc_model_data *model_data =
		of_device_get_match_data(data->dev);

	if (rate < model_data->min_sampling_rate ||
	    rate > model_data->max_sampling_rate)
		return -EINVAL;
	/* Each sampling needs 12 clocks to covert.*/
	clk_set_rate(data->clk_scaler->clk, rate * ASPEED_CLOCKS_PER_SAMPLE);

	rate = clk_get_rate(data->clk_scaler->clk);
	data->sample_period_ns = DIV_ROUND_UP_ULL(
		(u64)NSEC_PER_SEC * ASPEED_CLOCKS_PER_SAMPLE, rate);
	dev_dbg(data->dev, "Adc clock = %d sample period = %d ns", rate,
		data->sample_period_ns);
	return 0;
}

static int aspeed_adc_read_raw(struct iio_dev *indio_dev,
			       struct iio_chan_spec const *chan,
			       int *val, int *val2, long mask)
{
	struct aspeed_adc_data *data = iio_priv(indio_dev);
	u32 adc_engine_control_reg_val;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		if (data->battery_sensing && chan->channel == 7) {
			adc_engine_control_reg_val =
				readl(data->base + ASPEED_REG_ENGINE_CONTROL);
			writel(adc_engine_control_reg_val |
				       ASPEED_ADC_CH7_BATTERY |
				       ASPEED_ADC_BATTERY_SENSING_ENABLE,
			       data->base + ASPEED_REG_ENGINE_CONTROL);
			/*
			 * After enable battery sensing mode need to wait some time for adc stable
			 * Experiment result is 1ms.
			 */
			mdelay(1);
			*val = readw(data->base + chan->address) + data->cv;
			if (*val < 0)
				*val = 0;
			else if (*val >= ASPEED_ADC_MAX_RAW_DATA)
				*val = ASPEED_ADC_MAX_RAW_DATA;
			*val = (*val * data->battery_mode_gain.mult) /
			       data->battery_mode_gain.div;
			writel(adc_engine_control_reg_val,
			       data->base + ASPEED_REG_ENGINE_CONTROL);
		} else {
			*val = readw(data->base + chan->address) + data->cv;
			if (*val < 0)
				*val = 0;
			else if (*val >= ASPEED_ADC_MAX_RAW_DATA)
				*val = ASPEED_ADC_MAX_RAW_DATA;
		}
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_SCALE:
		*val = data->vref;
		*val2 = ASPEED_RESOLUTION_BITS;
		return IIO_VAL_FRACTIONAL_LOG2;

	case IIO_CHAN_INFO_SAMP_FREQ:
		*val = clk_get_rate(data->clk_scaler->clk) /
				ASPEED_CLOCKS_PER_SAMPLE;
		return IIO_VAL_INT;

	default:
		return -EINVAL;
	}
}

static int aspeed_adc_write_raw(struct iio_dev *indio_dev,
				struct iio_chan_spec const *chan,
				int val, int val2, long mask)
{
	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		return aspeed_adc_set_sampling_rate(indio_dev, val);

	case IIO_CHAN_INFO_SCALE:
	case IIO_CHAN_INFO_RAW:
		/*
		 * Technically, these could be written but the only reasons
		 * for doing so seem better handled in userspace.  EPERM is
		 * returned to signal this is a policy choice rather than a
		 * hardware limitation.
		 */
		return -EPERM;

	default:
		return -EINVAL;
	}
}

static int aspeed_adc_reg_access(struct iio_dev *indio_dev,
				 unsigned int reg, unsigned int writeval,
				 unsigned int *readval)
{
	struct aspeed_adc_data *data = iio_priv(indio_dev);

	if (!readval || reg % 4 || reg > ASPEED_REG_MAX)
		return -EINVAL;

	*readval = readl(data->base + reg);

	return 0;
}

static const struct iio_info aspeed_adc_iio_info = {
	.read_raw = aspeed_adc_read_raw,
	.write_raw = aspeed_adc_write_raw,
	.debugfs_reg_access = aspeed_adc_reg_access,
};

static int aspeed_adc_vref_config(struct platform_device *pdev)
{
	const struct aspeed_adc_model_data *model_data;
	struct iio_dev *indio_dev = platform_get_drvdata(pdev);
	struct aspeed_adc_data *data = iio_priv(indio_dev);
	int vref;
	u32 adc_engine_control_reg_val =
		readl(data->base + ASPEED_REG_ENGINE_CONTROL);

	model_data = of_device_get_match_data(&pdev->dev);
	switch (model_data->version) {
	case aspeed_adc_ast2400:
		vref = 2500;
		break;
	case aspeed_adc_ast2500:
		vref = 1800;
		break;
	case aspeed_adc_ast2600:
		if (of_property_read_u32(pdev->dev.of_node, "vref", &vref))
			vref = 2500;
		if (vref == 2500)
			writel(adc_engine_control_reg_val |
				       ASPEED_ADC_REF_VOLTAGE_2500mV,
			       data->base + ASPEED_REG_ENGINE_CONTROL);
		else if (vref == 1200)
			writel(adc_engine_control_reg_val |
				       ASPEED_ADC_REF_VOLTAGE_1200mV,
			       data->base + ASPEED_REG_ENGINE_CONTROL);
		else if ((vref >= 1550) && (vref <= 2700))
			writel(adc_engine_control_reg_val |
				       ASPEED_ADC_REF_VOLTAGE_EXT_HIGH,
			       data->base + ASPEED_REG_ENGINE_CONTROL);
		else if ((vref >= 900) && (vref <= 1650))
			writel(adc_engine_control_reg_val |
				       ASPEED_ADC_REF_VOLTAGE_EXT_LOW,
			       data->base + ASPEED_REG_ENGINE_CONTROL);
		else {
			dev_err(&pdev->dev, "Vref not support");
			return -EOPNOTSUPP;
		}
		break;
	default:
		dev_err(&pdev->dev, "ADC version not recognized");
		return -EOPNOTSUPP;
	}
	data->vref = vref;
	return 0;
}

static int aspeed_adc_probe(struct platform_device *pdev)
{
	struct iio_dev *indio_dev;
	struct aspeed_adc_data *data;
	const struct aspeed_adc_model_data *model_data;
	const char *clk_parent_name;
	int ret;
	u32 adc_engine_control_reg_val;
	char scaler_clk_name[32];
	char fixed_div_clk_name[32];

	model_data = of_device_get_match_data(&pdev->dev);
	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*data));
	if (!indio_dev)
		return -ENOMEM;

	data = iio_priv(indio_dev);
	data->dev = &pdev->dev;
	dev_set_drvdata(data->dev, indio_dev);

	data->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(data->base))
		return PTR_ERR(data->base);

	/* Register ADC clock prescaler with source specified by device tree. */
	spin_lock_init(&data->clk_lock);
	clk_parent_name = of_clk_get_parent_name(pdev->dev.of_node, 0);
	if (model_data->version <= aspeed_adc_ast2500) {
		/* ADC clock period = PCLK * 2 * (ADC0C[31:17] + 1) * (ADC0C[9:0] + 1) */
		data->fixed_div_clk = clk_hw_register_fixed_factor(
			&pdev->dev, "fixed-div", clk_parent_name, 0, 1, 2);
		if (IS_ERR(data->fixed_div_clk))
			return PTR_ERR(data->fixed_div_clk);
		data->clk_prescaler = clk_hw_register_divider(
			&pdev->dev, "prescaler", "fixed-div", 0,
			data->base + ASPEED_REG_CLOCK_CONTROL, 17, 15, 0,
			&data->clk_lock);
		if (IS_ERR(data->clk_prescaler))
			return PTR_ERR(data->clk_prescaler);

		/*
		 * Register ADC clock scaler downstream from the prescaler. Allow rate
		 * setting to adjust the prescaler as well.
		 */
		data->clk_scaler = clk_hw_register_divider(
					&pdev->dev, "scaler", "prescaler",
					CLK_SET_RATE_PARENT,
					data->base + ASPEED_REG_CLOCK_CONTROL,
					0, 10, 0, &data->clk_lock);
		if (IS_ERR(data->clk_scaler)) {
			ret = PTR_ERR(data->clk_scaler);
			goto scaler_error;
		}
	} else {
		/* ADC clock period = period of PCLK * 2 * (ADC0C[15:0] + 1) */
		snprintf(fixed_div_clk_name, sizeof(fixed_div_clk_name), "fixed-div-%s",
			 pdev->name);
		data->fixed_div_clk = clk_hw_register_fixed_factor(
			&pdev->dev, fixed_div_clk_name, clk_parent_name, 0, 1, 2);
		if (IS_ERR(data->fixed_div_clk))
			return PTR_ERR(data->fixed_div_clk);
		snprintf(scaler_clk_name, sizeof(scaler_clk_name), "scaler-%s",
			 pdev->name);
		data->clk_scaler = clk_hw_register_divider(
			&pdev->dev, scaler_clk_name, clk_parent_name, 0,
			data->base + ASPEED_REG_CLOCK_CONTROL, 0, 16, 0,
			&data->clk_lock);
		if (IS_ERR(data->clk_scaler)) {
			ret = PTR_ERR(data->clk_scaler);
			goto scaler_error;
		}
	}

	data->rst = devm_reset_control_get_shared(&pdev->dev, NULL);
	if (IS_ERR(data->rst)) {
		dev_err(&pdev->dev,
			"invalid or missing reset controller device tree entry");
		ret = PTR_ERR(data->rst);
		goto reset_error;
	}
	reset_control_deassert(data->rst);

	if (model_data->wait_init_sequence) {
		/* Enable engine in normal mode. */
		writel(ASPEED_ADC_OPERATION_MODE_NORMAL | ASPEED_ADC_ENGINE_ENABLE,
		       data->base + ASPEED_REG_ENGINE_CONTROL);

		/* Wait for initial sequence complete. */
		ret = readl_poll_timeout(data->base + ASPEED_REG_ENGINE_CONTROL,
					 adc_engine_control_reg_val,
					 adc_engine_control_reg_val &
					 ASPEED_ADC_CTRL_INIT_RDY,
					 ASPEED_ADC_INIT_POLLING_TIME,
					 ASPEED_ADC_INIT_TIMEOUT);
		if (ret)
			goto poll_timeout_error;
	}
	if (of_find_property(data->dev->of_node, "aspeed,trim-data-valid", NULL))
		aspeed_adc_set_trim_data(pdev);
	ret = aspeed_adc_vref_config(pdev);
	if (ret)
		goto vref_config_error;
	if (of_find_property(data->dev->of_node, "battery-sensing", NULL)) {
		if (model_data->version >= aspeed_adc_ast2600) {
			data->battery_sensing = 1;
			if (readl(data->base + ASPEED_REG_ENGINE_CONTROL) &
			    ASPEED_ADC_BATTERY_SENSING_DIV_1_3) {
				data->battery_mode_gain.mult = 3;
				data->battery_mode_gain.div = 1;
			} else {
				data->battery_mode_gain.mult = 3;
				data->battery_mode_gain.div = 2;
			}
		} else
			dev_warn(&pdev->dev,
				 "Failed to enable battey-sensing mode\n");
	}
	ret = clk_prepare_enable(data->clk_scaler->clk);
	if (ret)
		goto clk_enable_error;
	aspeed_adc_set_sampling_rate(indio_dev, ASPEED_ADC_DEF_SAMPLING_RATE);
	aspeed_adc_compensation(pdev);
	adc_engine_control_reg_val =
		readl(data->base + ASPEED_REG_ENGINE_CONTROL);
	/* Start all channels in normal mode. */
	adc_engine_control_reg_val |= ASPEED_ADC_CTRL_CHANNEL |
				     ASPEED_ADC_OPERATION_MODE_NORMAL |
				     ASPEED_ADC_ENGINE_ENABLE;
	writel(adc_engine_control_reg_val,
		data->base + ASPEED_REG_ENGINE_CONTROL);

	indio_dev->name = dev_name(&pdev->dev);
	indio_dev->dev.parent = &pdev->dev;
	indio_dev->info = &aspeed_adc_iio_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = aspeed_adc_iio_channels;
	indio_dev->num_channels = model_data->num_channels;

	ret = iio_device_register(indio_dev);
	if (ret)
		goto iio_register_error;

	return 0;

iio_register_error:
	writel(ASPEED_ADC_OPERATION_MODE_POWER_DOWN,
		data->base + ASPEED_REG_ENGINE_CONTROL);
	clk_disable_unprepare(data->clk_scaler->clk);
vref_config_error:
clk_enable_error:
poll_timeout_error:
	reset_control_assert(data->rst);
reset_error:
	clk_hw_unregister_divider(data->clk_scaler);
scaler_error:
	if (model_data->version <= aspeed_adc_ast2500)
		clk_hw_unregister_divider(data->clk_prescaler);
	clk_hw_unregister_fixed_factor(data->fixed_div_clk);
	return ret;
}

static int aspeed_adc_remove(struct platform_device *pdev)
{
	struct iio_dev *indio_dev = platform_get_drvdata(pdev);
	struct aspeed_adc_data *data = iio_priv(indio_dev);
	const struct aspeed_adc_model_data *model_data;

	model_data = of_device_get_match_data(&pdev->dev);
	iio_device_unregister(indio_dev);
	writel(ASPEED_ADC_OPERATION_MODE_POWER_DOWN,
		data->base + ASPEED_REG_ENGINE_CONTROL);
	clk_disable_unprepare(data->clk_scaler->clk);
	reset_control_assert(data->rst);
	clk_hw_unregister_divider(data->clk_scaler);
	if (model_data->version <= aspeed_adc_ast2500)
		clk_hw_unregister_divider(data->clk_prescaler);
	clk_hw_unregister_fixed_factor(data->fixed_div_clk);

	return 0;
}

static const struct aspeed_adc_trim_locate ast2500_adc_trim = {
	.offset = 0x154,
	.field = GENMASK(31, 28),
};

static const struct aspeed_adc_trim_locate ast2600_adc0_trim = {
	.offset = 0x5d0,
	.field = GENMASK(3, 0),
};

static const struct aspeed_adc_trim_locate ast2600_adc1_trim = {
	.offset = 0x5d0,
	.field = GENMASK(7, 4),
};

static const struct aspeed_adc_model_data ast2400_model_data = {
	.version = aspeed_adc_ast2400,
	.min_sampling_rate = 10000,
	.max_sampling_rate = 500000,
	.num_channels = 16,
};

static const struct aspeed_adc_model_data ast2500_model_data = {
	.version = aspeed_adc_ast2500,
	.min_sampling_rate = 10000,
	.max_sampling_rate = 500000,
	.wait_init_sequence = true,
	.num_channels = 16,
	.trim_locate = &ast2500_adc_trim,
};

static const struct aspeed_adc_model_data ast2600_adc0_model_data = {
	.version = aspeed_adc_ast2600,
	.min_sampling_rate = 10000,
	.max_sampling_rate = 500000,
	.wait_init_sequence = true,
	.num_channels = 8,
	.trim_locate = &ast2600_adc0_trim,
};

static const struct aspeed_adc_model_data ast2600_adc1_model_data = {
	.version = aspeed_adc_ast2600,
	.min_sampling_rate = 10000,
	.max_sampling_rate = 500000,
	.wait_init_sequence = true,
	.num_channels = 8,
	.trim_locate = &ast2600_adc1_trim,
};

static const struct of_device_id aspeed_adc_matches[] = {
	{ .compatible = "aspeed,ast2400-adc", .data = &ast2400_model_data },
	{ .compatible = "aspeed,ast2500-adc", .data = &ast2500_model_data },
	{ .compatible = "aspeed,ast2600-adc0", .data = &ast2600_adc0_model_data },
	{ .compatible = "aspeed,ast2600-adc1", .data = &ast2600_adc1_model_data },
	{},
};
MODULE_DEVICE_TABLE(of, aspeed_adc_matches);

static struct platform_driver aspeed_adc_driver = {
	.probe = aspeed_adc_probe,
	.remove = aspeed_adc_remove,
	.driver = {
		.name = KBUILD_MODNAME,
		.of_match_table = aspeed_adc_matches,
	}
};

module_platform_driver(aspeed_adc_driver);

MODULE_AUTHOR("Rick Altherr <raltherr@google.com>");
MODULE_DESCRIPTION("Aspeed AST2400/2500/2600 ADC Driver");
MODULE_LICENSE("GPL");
