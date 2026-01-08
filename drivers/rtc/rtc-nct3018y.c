// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2022 Nuvoton Technology Corporation

#include <linux/bcd.h>
#include <linux/bitfield.h>
#include <linux/clk-provider.h>
#include <linux/err.h>
#include <linux/hwmon.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/rtc.h>
#include <linux/slab.h>
#include <linux/regmap.h>

#define NCT3018Y_REG_SC		0x00 /* seconds */
#define NCT3018Y_REG_SCA	0x01 /* alarm */
#define NCT3018Y_REG_MN		0x02
#define NCT3018Y_REG_MNA	0x03 /* alarm */
#define NCT3018Y_REG_HR		0x04
#define NCT3018Y_REG_HRA	0x05 /* alarm */
#define NCT3018Y_REG_DW		0x06
#define NCT3018Y_REG_DM		0x07
#define NCT3018Y_REG_MO		0x08
#define NCT3018Y_REG_YR		0x09
#define NCT3018Y_REG_CTRL	0x0A /* timer control */
#define NCT3018Y_REG_ST		0x0B /* status */
#define NCT3018Y_REG_CLKO	0x0C /* clock out */
#define NCT3018Y_REG_INTR_CTRL  0x12 /* intrusion control */
#define NCT3018Y_REG_INTR_TS_SC 0x13 /* intrusion timestamp */
#define NCT3018Y_REG_PART	0x21 /* part info */

#define NCT3018Y_BIT_AF		BIT(7)
#define NCT3018Y_BIT_ST		BIT(7)
#define NCT3018Y_BIT_DM		BIT(6)
#define NCT3018Y_BIT_HF		BIT(5)
#define NCT3018Y_BIT_DSM	BIT(4)
#define NCT3018Y_BIT_AIE	BIT(3)
#define NCT3018Y_BIT_OFIE	BIT(2)
#define NCT3018Y_BIT_CIE	BIT(1)
#define NCT3018Y_BIT_TWO	BIT(0)

#define NCT3018Y_REG_ST_OF    BIT(6)
#define NCT3018Y_REG_ST_RTCF  BIT(5)

#define NCT3018Y_REG_BAT_MASK		0x07
#define NCT3018Y_REG_CLKO_F_MASK	0x03 /* frequenc mask */
#define NCT3018Y_REG_CLKO_CKE		0x80 /* clock out enabled */
#define NCT3018Y_REG_PART_NCT3018Y	0x02

static const struct regmap_config nct3018y_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0x2F,
};

struct nct3018y {
	struct regmap *regmap;
	struct rtc_device *rtc;
	struct i2c_client *client;
	struct device *hwmon_dev;
	int part_num;
#ifdef CONFIG_COMMON_CLK
	struct clk_hw clkout_hw;
#endif
};

static int nct3018y_set_alarm_mode(struct nct3018y *nct3018y, bool on)
{
	int err;

	err = regmap_update_bits(nct3018y->regmap, NCT3018Y_REG_CTRL,
							 NCT3018Y_BIT_AIE, 
							 on ? NCT3018Y_BIT_AIE : 0);
	if (err)
		return err;

	err = regmap_update_bits(nct3018y->regmap, NCT3018Y_REG_CTRL,
							 NCT3018Y_BIT_CIE,
							 NCT3018Y_BIT_CIE);
	if (err)
		return err;

	return regmap_update_bits(nct3018y->regmap, NCT3018Y_REG_ST,
							  NCT3018Y_BIT_AF, 0);
}

static int nct3018y_get_alarm_mode(struct nct3018y *nct3018y,
				   unsigned char *alarm_enable,
				   unsigned char *alarm_flag)
{
	unsigned int val;
	int err;

	if (alarm_enable) {
		dev_dbg(&nct3018y->client->dev, "%s:NCT3018Y_REG_CTRL\n", __func__);
		err = regmap_read(nct3018y->regmap, NCT3018Y_REG_CTRL, &val);
		if (err)
			return err;
		*alarm_enable = val & NCT3018Y_BIT_AIE;
		dev_dbg(&nct3018y->client->dev, "%s:alarm_enable:%x\n", __func__, *alarm_enable);
	}

	if (alarm_flag) {
		dev_dbg(&nct3018y->client->dev, "%s:NCT3018Y_REG_ST\n", __func__);
		err = regmap_read(nct3018y->regmap, NCT3018Y_REG_ST, &val);
		if (err)
			return err;
		*alarm_flag = val & NCT3018Y_BIT_AF;
		dev_dbg(&nct3018y->client->dev, "%s:alarm_flag:%x\n", __func__, *alarm_flag);
	}

	return 0;
}

static irqreturn_t nct3018y_irq(int irq, void *dev_id)
{
	struct nct3018y *nct3018y = dev_id;
	int err;
	unsigned char alarm_flag;
	unsigned char alarm_enable;

	err = nct3018y_get_alarm_mode(nct3018y, &alarm_enable, &alarm_flag);
	if (err)
		return IRQ_NONE;

	if (alarm_flag) {
		rtc_update_irq(nct3018y->rtc, 1, RTC_IRQF | RTC_AF);
		nct3018y_set_alarm_mode(nct3018y, 0);
		return IRQ_HANDLED;
	}

	return IRQ_NONE;
}

/*
 * In the routines that deal directly with the nct3018y hardware, we use
 * rtc_time -- month 0-11, hour 0-23, yr = calendar year-epoch.
 */
static int nct3018y_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	struct nct3018y *nct3018y = dev_get_drvdata(dev);
	unsigned char buf[10];
	unsigned int val;
	int err;

	err = regmap_read(nct3018y->regmap, NCT3018Y_REG_ST, &val);
	if (err)
		return err;

	err = regmap_bulk_read(nct3018y->regmap, NCT3018Y_REG_SC, buf,
			       sizeof(buf));
	if (err)
		return err;

	tm->tm_sec = bcd2bin(buf[0] & 0x7F);
	tm->tm_min = bcd2bin(buf[2] & 0x7F);
	tm->tm_hour = bcd2bin(buf[4] & 0x3F);
	tm->tm_wday = buf[6] & 0x07;
	tm->tm_mday = bcd2bin(buf[7] & 0x3F);
	tm->tm_mon = bcd2bin(buf[8] & 0x1F) - 1;
	tm->tm_year = bcd2bin(buf[9]) + 100;

	return 0;
}

static int nct3018y_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	struct nct3018y *nct3018y = dev_get_drvdata(dev);
	unsigned char buf[4] = {0};
	unsigned int val;
	int err;
	int restore_flags = 0;

	err = regmap_read(nct3018y->regmap, NCT3018Y_REG_CTRL, &val);
	if (err)
		return err;

	/* Check and set TWO bit */
	if (nct3018y->part_num == NCT3018Y_REG_PART_NCT3018Y &&
	    !(val & NCT3018Y_BIT_TWO)) {
		restore_flags = 1;
		err = regmap_update_bits(nct3018y->regmap, NCT3018Y_REG_CTRL,
			                     NCT3018Y_BIT_TWO, NCT3018Y_BIT_TWO);
		if (err)
			return err;
	}

	err = regmap_write(nct3018y->regmap, NCT3018Y_REG_SC,
			   bin2bcd(tm->tm_sec));
	if (err)
		return err;

	err = regmap_write(nct3018y->regmap, NCT3018Y_REG_MN,
			   bin2bcd(tm->tm_min));
	if (err)
		return err;

	err = regmap_write(nct3018y->regmap, NCT3018Y_REG_HR,
			   bin2bcd(tm->tm_hour));
	if (err)
		return err;

	buf[0] = tm->tm_wday & 0x07;
	buf[1] = bin2bcd(tm->tm_mday);
	buf[2] = bin2bcd(tm->tm_mon + 1);
	buf[3] = bin2bcd(tm->tm_year - 100);
	err = regmap_bulk_write(nct3018y->regmap, NCT3018Y_REG_DW, buf,
				sizeof(buf));
	if (err)
		return err;

	/* Restore TWO bit */
	if (restore_flags) {
	    err = regmap_update_bits(nct3018y->regmap, NCT3018Y_REG_CTRL,
			                     NCT3018Y_BIT_TWO, 0);
		if (err)
			return err;
	}

	return 0;
}

static int nct3018y_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *tm)
{
	struct nct3018y *nct3018y = dev_get_drvdata(dev);
	unsigned char buf[5];
	int err;

	err = regmap_bulk_read(nct3018y->regmap, NCT3018Y_REG_SCA, buf,
			       sizeof(buf));
	if (err)
		return err;

	tm->time.tm_sec = bcd2bin(buf[0] & 0x7F);
	tm->time.tm_min = bcd2bin(buf[2] & 0x7F);
	tm->time.tm_hour = bcd2bin(buf[4] & 0x3F);

	err = nct3018y_get_alarm_mode(nct3018y, &tm->enabled, &tm->pending);
	if (err)
		return err;

	return 0;
}

static int nct3018y_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *tm)
{
	struct nct3018y *nct3018y = dev_get_drvdata(dev);
	int err;

	err = regmap_write(nct3018y->regmap, NCT3018Y_REG_SCA,
			   bin2bcd(tm->time.tm_sec));
	if (err)
		return err;

	err = regmap_write(nct3018y->regmap, NCT3018Y_REG_MNA,
			   bin2bcd(tm->time.tm_min));
	if (err)
		return err;

	err = regmap_write(nct3018y->regmap, NCT3018Y_REG_HRA,
			   bin2bcd(tm->time.tm_hour));
	if (err)
		return err;

	return nct3018y_set_alarm_mode(nct3018y, tm->enabled);
}

static int nct3018y_irq_enable(struct device *dev, unsigned int enabled)
{
	struct nct3018y *nct3018y = dev_get_drvdata(dev);

	return nct3018y_set_alarm_mode(nct3018y, enabled);
}

static ssize_t intrusion0_timestamp_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct nct3018y *nct3018y = dev_get_drvdata(dev);
	unsigned char data[6];
	struct rtc_time tm;
	time64_t timestamp;
	int ret;

	ret = regmap_bulk_read(nct3018y->regmap, NCT3018Y_REG_INTR_TS_SC,
			       data, sizeof(data));
	if (ret)
		return ret;

	tm.tm_sec = bcd2bin(data[0] & 0x7F);
	tm.tm_min = bcd2bin(data[1] & 0x7F);
	tm.tm_hour = bcd2bin(data[2] & 0xFF);
	tm.tm_mday = bcd2bin(data[3] & 0x3F);
	tm.tm_mon = bcd2bin(data[4] & 0x1F) - 1;
	tm.tm_year = bcd2bin(data[5]) + 100;

	/* Try to convert to timestamp, return 0 if invalid */
	if (rtc_valid_tm(&tm) < 0)
		timestamp = 0;
	else
		timestamp = rtc_tm_to_time64(&tm);

	return sysfs_emit(buf, "%lld\n", timestamp);
}

static DEVICE_ATTR_RO(intrusion0_timestamp);

static ssize_t intrusion0_alarm_show(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	struct nct3018y *nct3018y = dev_get_drvdata(dev);
	unsigned int val;
	int ret;

	ret = regmap_read(nct3018y->regmap, NCT3018Y_REG_INTR_CTRL, &val);
	if (ret)
		return ret;

	/* Return bit 1 value */
	return sysfs_emit(buf, "%d\n", (val & BIT(1)) ? 1 : 0);
}

static ssize_t intrusion0_alarm_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	struct nct3018y *nct3018y = dev_get_drvdata(dev);
	unsigned long input;
	int ret;

	if (kstrtoul(buf, 10, &input) < 0 || input != 0)
	    return -EINVAL;

	/* write 0 to clear */
	ret = regmap_update_bits(nct3018y->regmap, NCT3018Y_REG_INTR_CTRL, BIT(1), 0);
	if (ret)
		return ret;

	return count;
}

static DEVICE_ATTR_RW(intrusion0_alarm);

static struct attribute *nct3018y_hwmon_attrs[] = {
	&dev_attr_intrusion0_timestamp.attr,
	&dev_attr_intrusion0_alarm.attr,
	NULL
};
ATTRIBUTE_GROUPS(nct3018y_hwmon);

static int nct3018y_bvl_to_mv(unsigned int bvl)
{
	static const int mv_table[] = {
		0,
		1900,
		2100,
		2300,
		2500,
		2700,
		2900,
		3100,
	};
	return mv_table[bvl & 7];
}

static int nct3018y_ioctl(struct device *dev, unsigned int cmd,
			  unsigned long arg)
{
	struct nct3018y *nct3018y = dev_get_drvdata(dev);
	unsigned int status;
	unsigned int bvl;
	unsigned int flags = 0;
	int err;

	switch (cmd) {
	case RTC_VL_READ:
		err = regmap_read(nct3018y->regmap, NCT3018Y_REG_ST, &status);
		if (err)
			return err;

		/*
		 * OF/RTCF indicate the oscillator stopped or RTC lost all power,
		 * so RTC time data cannot be trusted.
		 */
		if (status & (NCT3018Y_REG_ST_OF | NCT3018Y_REG_ST_RTCF))
			flags |= RTC_VL_DATA_INVALID;

		/*
		 * BVL is a battery voltage level encoding. A level of 0 indicates
		 * VBAT is below the minimum threshold (<= 1.7V), which is not
		 * sufficient for reliable timestamp retention on this device.
		 */
		bvl = FIELD_GET(NCT3018Y_REG_BAT_MASK, status);
		if (bvl == 0)
			flags |= RTC_VL_BACKUP_LOW;

		return put_user(flags, (unsigned int __user *)arg);

	default:
		return -ENOIOCTLCMD;
	}
}

static int nct3018y_read_battery_voltage(struct nct3018y *nct3018y)
{
	unsigned int val, bvl;
	int err;

	err = regmap_read(nct3018y->regmap, NCT3018Y_REG_ST, &val);
	if (err)
		return err;

	bvl = FIELD_GET(NCT3018Y_REG_BAT_MASK, val);
	return nct3018y_bvl_to_mv(bvl);
}

static umode_t nct3018y_hwmon_is_visible(const void *data,
					 enum hwmon_sensor_types type,
					 u32 attr, int channel)
{
	if (type == hwmon_in && attr == hwmon_in_input)
		return 0444;

	return 0;
}

static int nct3018y_hwmon_read(struct device *dev,
			       enum hwmon_sensor_types type,
			       u32 attr, int channel, long *val)
{
	struct nct3018y *nct3018y = dev_get_drvdata(dev);
	int ret;

	if (type == hwmon_in && attr == hwmon_in_input) {
		ret = nct3018y_read_battery_voltage(nct3018y);
		if (ret < 0)
			return ret;
		*val = ret;
		return 0;
	}

	return -EOPNOTSUPP;
}

static const struct hwmon_channel_info *nct3018y_hwmon_info[] = {
	HWMON_CHANNEL_INFO(in, HWMON_I_INPUT),
	NULL
};

static const struct hwmon_ops nct3018y_hwmon_ops = {
	.is_visible = nct3018y_hwmon_is_visible,
	.read = nct3018y_hwmon_read,
};

static const struct hwmon_chip_info nct3018y_hwmon_chip_info = {
	.ops = &nct3018y_hwmon_ops,
	.info = nct3018y_hwmon_info,
};

#ifdef CONFIG_COMMON_CLK
/*
 * Handling of the clkout
 */

#define clkout_hw_to_nct3018y(_hw) container_of(_hw, struct nct3018y, clkout_hw)

static const int clkout_rates[] = {
	32768,
	1024,
	32,
	1,
};

static unsigned long nct3018y_clkout_recalc_rate(struct clk_hw *hw,
						 unsigned long parent_rate)
{
	struct nct3018y *nct3018y = clkout_hw_to_nct3018y(hw);
	unsigned int val;
	int err;

	err = regmap_read(nct3018y->regmap, NCT3018Y_REG_CLKO, &val);
	if (err)
		return 0;

	val &= NCT3018Y_REG_CLKO_F_MASK;
	return clkout_rates[val];
}

static long nct3018y_clkout_round_rate(struct clk_hw *hw, unsigned long rate,
				       unsigned long *prate)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(clkout_rates); i++)
		if (clkout_rates[i] <= rate)
			return clkout_rates[i];

	return clkout_rates[0];
}

static int nct3018y_clkout_set_rate(struct clk_hw *hw, unsigned long rate,
				    unsigned long parent_rate)
{
	struct nct3018y *nct3018y = clkout_hw_to_nct3018y(hw);
	int i;

	for (i = 0; i < ARRAY_SIZE(clkout_rates); i++)
		if (clkout_rates[i] == rate) {
			return regmap_update_bits(nct3018y->regmap, NCT3018Y_REG_CLKO,
				                      NCT3018Y_REG_CLKO_F_MASK, i);
		}

	return -EINVAL;
}

static int nct3018y_clkout_control(struct clk_hw *hw, bool enable)
{
	struct nct3018y *nct3018y = clkout_hw_to_nct3018y(hw);

	return regmap_update_bits(nct3018y->regmap, NCT3018Y_REG_CLKO,
								NCT3018Y_REG_CLKO_CKE,
								enable ? NCT3018Y_REG_CLKO_CKE : 0);
}

static int nct3018y_clkout_prepare(struct clk_hw *hw)
{
	return nct3018y_clkout_control(hw, 1);
}

static void nct3018y_clkout_unprepare(struct clk_hw *hw)
{
	nct3018y_clkout_control(hw, 0);
}

static int nct3018y_clkout_is_prepared(struct clk_hw *hw)
{
	struct nct3018y *nct3018y = clkout_hw_to_nct3018y(hw);
	unsigned int val;
	int err;

	err = regmap_read(nct3018y->regmap, NCT3018Y_REG_CLKO, &val);
	if (err)
		return err;

	return (val & NCT3018Y_REG_CLKO_CKE) ? 1 : 0;
}

static const struct clk_ops nct3018y_clkout_ops = {
	.prepare = nct3018y_clkout_prepare,
	.unprepare = nct3018y_clkout_unprepare,
	.is_prepared = nct3018y_clkout_is_prepared,
	.recalc_rate = nct3018y_clkout_recalc_rate,
	.round_rate = nct3018y_clkout_round_rate,
	.set_rate = nct3018y_clkout_set_rate,
};

static struct clk *nct3018y_clkout_register_clk(struct nct3018y *nct3018y,
						struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct device_node *node = dev->of_node;
	struct clk *clk;
	struct clk_init_data init;

	init.name = "nct3018y-clkout";
	init.ops = &nct3018y_clkout_ops;
	init.flags = 0;
	init.parent_names = NULL;
	init.num_parents = 0;
	nct3018y->clkout_hw.init = &init;

	/* optional override of the clockname */
	of_property_read_string(node, "clock-output-names", &init.name);

	/* register the clock */
	clk = devm_clk_register(dev, &nct3018y->clkout_hw);

	if (!IS_ERR(clk))
		of_clk_add_provider(node, of_clk_src_simple_get, clk);

	return clk;
}
#endif

static const struct rtc_class_ops nct3018y_rtc_ops = {
	.read_time	= nct3018y_rtc_read_time,
	.set_time	= nct3018y_rtc_set_time,
	.read_alarm	= nct3018y_rtc_read_alarm,
	.set_alarm	= nct3018y_rtc_set_alarm,
	.alarm_irq_enable = nct3018y_irq_enable,
	.ioctl		= nct3018y_ioctl,
};

static int nct3018y_probe(struct i2c_client *client)
{
	struct nct3018y *nct3018y;
	unsigned int val;
	int err;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return -ENODEV;

	nct3018y =
		devm_kzalloc(&client->dev, sizeof(struct nct3018y), GFP_KERNEL);
	if (!nct3018y)
		return -ENOMEM;

	nct3018y->client = client;
	nct3018y->regmap =
		devm_regmap_init_i2c(client, &nct3018y_regmap_config);
	if (IS_ERR(nct3018y->regmap))
		return PTR_ERR(nct3018y->regmap);

	i2c_set_clientdata(client, nct3018y);
	device_set_wakeup_capable(&client->dev, 1);

	err = regmap_read(nct3018y->regmap, NCT3018Y_REG_CTRL, &val);
	if (err) {
		return err;
	}

	if (val & NCT3018Y_BIT_TWO) {
		dev_dbg(&client->dev, "%s: NCT3018Y_BIT_TWO is set\n",
			__func__);
	}

	err = regmap_read(nct3018y->regmap, NCT3018Y_REG_PART, &val);
	if (err)
		return err;

	nct3018y->part_num = val & 0x03;
	if (nct3018y->part_num == NCT3018Y_REG_PART_NCT3018Y) {
		err = regmap_update_bits(nct3018y->regmap, NCT3018Y_REG_CTRL,
					 NCT3018Y_BIT_HF, NCT3018Y_BIT_HF);
		if (err)
			return err;
	}

	err = regmap_write(nct3018y->regmap, NCT3018Y_REG_ST, 0);
	if (err)
		return err;

	nct3018y->rtc = devm_rtc_allocate_device(&client->dev);
	if (IS_ERR(nct3018y->rtc))
		return PTR_ERR(nct3018y->rtc);

	nct3018y->rtc->ops = &nct3018y_rtc_ops;
	nct3018y->rtc->range_min = RTC_TIMESTAMP_BEGIN_2000;
	nct3018y->rtc->range_max = RTC_TIMESTAMP_END_2099;

	if (client->irq > 0) {
		err = devm_request_threaded_irq(&client->dev, client->irq,
						NULL, nct3018y_irq,
						IRQF_ONESHOT | IRQF_TRIGGER_FALLING,
						"nct3018y", nct3018y);
		if (err) {
			dev_dbg(&client->dev, "unable to request IRQ %d\n", client->irq);
			return err;
		}
	} else {
		clear_bit(RTC_FEATURE_UPDATE_INTERRUPT, nct3018y->rtc->features);
		clear_bit(RTC_FEATURE_ALARM, nct3018y->rtc->features);
	}

#ifdef CONFIG_COMMON_CLK
	/* register clk in common clk framework */
	nct3018y_clkout_register_clk(nct3018y, client);
#endif

	/* register hwmon device for battery voltage and intrusion detection */
	nct3018y->hwmon_dev = devm_hwmon_device_register_with_info(
		&client->dev, "nct3018y", nct3018y, &nct3018y_hwmon_chip_info,
		nct3018y_hwmon_groups);
	if (IS_ERR(nct3018y->hwmon_dev)) {
		dev_warn(&client->dev, "unable to register hwmon device\n");
		nct3018y->hwmon_dev = NULL;
	}

	return devm_rtc_register_device(nct3018y->rtc);
}

static const struct i2c_device_id nct3018y_id[] = {
	{ "nct3018y" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, nct3018y_id);

static const struct of_device_id nct3018y_of_match[] = {
	{ .compatible = "nuvoton,nct3018y" },
	{}
};
MODULE_DEVICE_TABLE(of, nct3018y_of_match);

static struct i2c_driver nct3018y_driver = {
	.driver		= {
		.name	= "rtc-nct3018y",
		.of_match_table = nct3018y_of_match,
	},
	.probe		= nct3018y_probe,
	.id_table	= nct3018y_id,
};

module_i2c_driver(nct3018y_driver);

MODULE_AUTHOR("Medad CChien <ctcchien@nuvoton.com>");
MODULE_AUTHOR("Mia Lin <mimi05633@gmail.com>");
MODULE_DESCRIPTION("Nuvoton NCT3018Y RTC driver");
MODULE_LICENSE("GPL");
