// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * smc_cpld.c - handle fans and led's with the custom cpld
 *
 * (C) 2023 by Radivoje (Ogi) Jovanovic <rjovanovic@nvidia.com>
 */

#include <linux/err.h>
#include <linux/hwmon.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/leds.h>

#define NR_CHANNEL 24
#define NR_CHANNEL_IN 8
#define NR_CHANNEL_PWM 3
#define PWM_FAN0 0x1

typedef enum smc_cpld_mode {
	FAN_MODE,
	ADC_MODE,
	GPIO_MODE,
} smc_cpld_mode;

struct smc_cpld_data {
	struct i2c_client *client;
};

static int set_cpld_mode(smc_cpld_mode mode, const struct i2c_client *client)
{
	int ret = 0;
	char msg[2];

	msg[0] = 0xf9;
	msg[1] = mode;

	ret = i2c_master_send(client, msg, 2);
	return ret == 2 ? 0 : ret;
}

static int read_register_byte(struct i2c_client *client, u8 reg, u8 *data, int data_size)
{
    struct i2c_msg msgs[2];
    int ret = 0;

    msgs[0].addr = client->addr;
    msgs[0].flags = 0;
    msgs[0].buf = &reg;
    msgs[0].len = 1;

    msgs[1].addr = client->addr;
    msgs[1].flags = I2C_M_RD;
    msgs[1].buf = data;
    msgs[1].len = data_size;
	ret = i2c_transfer(client->adapter, msgs, 2);
    if (ret < 0) {
        return ret;
    }

    return 0;
}

static int smc_cpld_write(struct device *dev, enum hwmon_sensor_types type,
			  u32 attr, int channel, long val)
{
	struct smc_cpld_data *data = dev_get_drvdata(dev);
	int ret = 0;

	if (channel >= NR_CHANNEL_PWM || channel < 0)
		return -ENODEV;

	if (set_cpld_mode(FAN_MODE, data->client))
		return -1;

	switch (type) {
	case hwmon_pwm:
		u8 msg[2];
		msg[0] = PWM_FAN0 + channel;
		if (val > 100)
			val = 100;
		if (val < 0)
			val = 0;
		msg[1] = val;
		ret = i2c_master_send(data->client, msg, 2);
		if (ret == 2)
			ret = 0;
		break;
	default:
		return -EOPNOTSUPP;
	}
	return ret;
}

static int smc_cpld_read_tach(struct smc_cpld_data *cpld_data, int *tach_data, int channel)
{
	u8 fan_num = (channel) >> 1;
	u8 off = (channel) % 2 + 1;
	u8 cmd = 0xD + 3 * fan_num;
	u8 data[3];
	int ret = 0;
	ret = read_register_byte(cpld_data->client, cmd, data, 3);
	if (ret)
		return ret;
	*tach_data = data[off] * 100;
	return 0;
}

static int smc_cpld_read(struct device *dev, enum hwmon_sensor_types type,
			 u32 attr, int channel, long *val)
{
	struct smc_cpld_data *cpld_data = dev_get_drvdata(dev);
	u8 data;
	int ret;

	if (channel >= NR_CHANNEL || channel < 0)
		return -ENODEV;

	switch (type) {
	case hwmon_fan:
		int tach_data;
		if (attr != hwmon_fan_input)
			return -1;

		ret = set_cpld_mode(FAN_MODE, cpld_data->client);
		if (ret)
			return -1;

		ret = smc_cpld_read_tach(cpld_data, &tach_data, channel);
		if (ret < 0)
			return ret;
		*val = tach_data;
		break;
	case hwmon_pwm:
		if (attr != hwmon_pwm_input || set_cpld_mode(FAN_MODE, cpld_data->client) || channel > NR_CHANNEL_PWM || channel < 0 )
			return -1;

		ret = read_register_byte(cpld_data->client, channel + 1, &data, 1);
		*val = data;
		break;
	case hwmon_in:
		u8 cmd = 0x5 + 2 * channel;
		if (attr != hwmon_in_input)
			return -1;
		ret = set_cpld_mode(ADC_MODE, cpld_data->client);
		if (ret)
			return -1;
		ret = read_register_byte(cpld_data->client, cmd, &data, 1);
		*val = data;
		break;
	default:
		return -EOPNOTSUPP;
	}
	return ret;
}

static umode_t smc_cpld_is_visible(const void *data_in,
				   enum hwmon_sensor_types type,
				   u32 attr, int channel)
{
	switch (type) {
	case hwmon_fan:
		if (channel >= 0 && channel < NR_CHANNEL) {
			return 0444;
		}
		break; 
	case hwmon_pwm:
		if (channel >= 0 && channel < NR_CHANNEL_PWM) {
			return 0644;
		}
		break;
	case hwmon_in:
		if (channel <  0 || channel >= NR_CHANNEL_IN)
			return 0;
		switch (attr) {
		case hwmon_in_label:
				return 0444;
		case hwmon_in_input:
			return 0444;
		default:
			return 0;
		}
	default:
		return 0;
	}
	return 0;
}


static const char *smc_cpld_voltage_labels[] = {
    "P3V3_SB",
    "P3V3",
    "P12V_SB",
    "P5V_SB",
    "P5V",
    "P1V8",
    "P12V",
    "P1V8_SB",
};

static int smc_cpld_read_string(struct device *dev, enum hwmon_sensor_types type,
			       u32 attr, int channel, const char **str)
{

	if (type != hwmon_in || channel < 0 || channel >= NR_CHANNEL_IN)
		return -1;

	*str = smc_cpld_voltage_labels[channel];

	return 0;
}


//SENSOR_DEVICE_ATTR
static const struct hwmon_channel_info *smc_cpld_info[] = {
	HWMON_CHANNEL_INFO(fan,
			   HWMON_F_INPUT,
			   HWMON_F_INPUT,
			   HWMON_F_INPUT,
			   HWMON_F_INPUT,
			   HWMON_F_INPUT,
			   HWMON_F_INPUT,
			   HWMON_F_INPUT,
			   HWMON_F_INPUT,
			   HWMON_F_INPUT,
			   HWMON_F_INPUT,
			   HWMON_F_INPUT,
			   HWMON_F_INPUT,
			   HWMON_F_INPUT,
			   HWMON_F_INPUT,
			   HWMON_F_INPUT,
			   HWMON_F_INPUT,
			   HWMON_F_INPUT,
			   HWMON_F_INPUT,
			   HWMON_F_INPUT,
			   HWMON_F_INPUT,
			   HWMON_F_INPUT,
			   HWMON_F_INPUT,
			   HWMON_F_INPUT,
			   HWMON_F_INPUT),
	HWMON_CHANNEL_INFO(pwm,
			   HWMON_PWM_INPUT,
			   HWMON_PWM_INPUT,
			   HWMON_PWM_INPUT),
	HWMON_CHANNEL_INFO(in,
			   HWMON_I_INPUT | HWMON_I_LABEL,
			   HWMON_I_INPUT | HWMON_I_LABEL,
			   HWMON_I_INPUT | HWMON_I_LABEL,
			   HWMON_I_INPUT | HWMON_I_LABEL,
			   HWMON_I_INPUT | HWMON_I_LABEL,
			   HWMON_I_INPUT | HWMON_I_LABEL,
			   HWMON_I_INPUT | HWMON_I_LABEL,
			   HWMON_I_INPUT | HWMON_I_LABEL),

	NULL
};

static const struct hwmon_ops smc_cpld_hwmon_ops = {
	.is_visible = smc_cpld_is_visible,
	.read_string = smc_cpld_read_string,
	.read = smc_cpld_read,
	.write = smc_cpld_write,
};

static const struct hwmon_chip_info smc_cpld_chip_info = {
	.ops = &smc_cpld_hwmon_ops,
	.info = smc_cpld_info,
};

static ssize_t cpld_version_show(struct device *dev,
			      struct device_attribute *attr,
			      char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	u8 cmd = 0xf8;
	int ret = 0;

	ret = i2c_smbus_read_byte_data(client, cmd);
	return sprintf(buf, "%x\n", ret);
}
static DEVICE_ATTR_RO(cpld_version);

static int smc_cpld_probe(struct i2c_client *client)
{
	struct i2c_adapter *adapter = client->adapter;
	struct device *dev = &client->dev;
	struct smc_cpld_data *data;
	struct device *hwmon_dev;

	if (!i2c_check_functionality(adapter,I2C_FUNC_I2C)) 
		return -ENODEV;

	data = devm_kzalloc(dev, sizeof(struct smc_cpld_data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->client = client;

	device_create_file(dev, &dev_attr_cpld_version);

	hwmon_dev = devm_hwmon_device_register_with_info(dev, client->name,
							 data,
							 &smc_cpld_chip_info,
							 NULL);

	return PTR_ERR_OR_ZERO(hwmon_dev);
}

static void smc_cpld_remove(struct i2c_client *client)
{
	device_remove_file(&client->dev, &dev_attr_cpld_version);
}

static const struct i2c_device_id smc_cpld_id[] = {
	{ "smc_c2_cpld", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, smc_cpld_id);

static struct i2c_driver smc_cpld_driver = {
	.class		= I2C_CLASS_HWMON,
	.probe_new	= smc_cpld_probe,
	.remove = smc_cpld_remove,
	.driver = {
		.name	= "smc_c2_cpld",
	},
	.id_table	= smc_cpld_id,
};

module_i2c_driver(smc_cpld_driver);

MODULE_AUTHOR("Radivoje (Ogi) Jovanovic <rjovanovic@nvidia.com>");
MODULE_DESCRIPTION("Supermicro custom cpld fan/led driver");
MODULE_LICENSE("GPL");
