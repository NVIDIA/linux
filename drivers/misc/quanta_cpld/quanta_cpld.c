// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * quata_cpld.c - handle fans and led's with the custom cpld
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

#define NR_CHANNEL_PWM 9
#define NR_CHANNEL (2 * NR_CHANNEL_PWM)
#define CPLD_VER_REG 0x0
#define FAN_CONTROL_MODE 0x1
#define FAN_PRST 0x4
#define PWM_FAN0 0x18
#define LED0 0x50
#define FAN_TACH_INSIDE_OFFSET 0x20
#define FAN_TACH_OUTSIDE_OFFSET 0x22

enum quanta_led_list {
	STATUS_LED,
	POWER_LED,
	ID_LED,
	BMC_HB_LED,
	MAX_LED,
};

struct quanta_cpld_data {
	struct i2c_client *client;
	struct quanta_led {
		struct led_classdev     cdev;
		struct quanta_cpld_data *data;
	} leds[MAX_LED];
};

const char *led_names[] = {
	"STATUS",
	"POWER",
	"ID",
	"BMC_HB",
};

static int quanta_cpld_write(struct device *dev, enum hwmon_sensor_types type,
			  u32 attr, int channel, long val)
{
	struct quanta_cpld_data *data = dev_get_drvdata(dev);

	if (channel >= NR_CHANNEL_PWM || channel < 0)
		return -ENODEV;

	switch (attr) {
	case hwmon_pwm_input:
		u8 cmd = PWM_FAN0 + channel;
		//for some reason the spec does not increment
		// the registers normally. channel 0 starts at
		//0x18 and channel 8 ends at 0x17
		if (channel == 8)
			cmd = 0x17;
		if (val > 100)
			val = 100;
		if (val < 0)
			val = 0;
		return i2c_smbus_write_byte_data(data->client, cmd, val);
	default:
		return -EOPNOTSUPP;
	}
	return 0;
}

static int quanta_cpld_read_tach(struct quanta_cpld_data *data, int *tach_data, int channel, int offset)
{
	int ret;
	u8 cmd = offset + (channel * 2);
	ret = i2c_smbus_read_byte_data(data->client, cmd);
	if (ret < 0)
		return ret;
	*tach_data = ret;
	cmd += 1;
	ret = i2c_smbus_read_byte_data(data->client, cmd);
	if (ret < 0)
		return ret;
	*tach_data |= ret << 8;
	return 0;
}

static int quanta_cpld_read(struct device *dev, enum hwmon_sensor_types type,
			 u32 attr, int channel, long *val)
{

	struct quanta_cpld_data *data = dev_get_drvdata(dev);
	int ret;

	if (channel >= NR_CHANNEL || channel < 0)
		return -ENODEV;
	switch (attr) {
	case hwmon_fan_input:
		int tach_data;
		ret = quanta_cpld_read_tach(data, &tach_data, channel, FAN_TACH_INSIDE_OFFSET);
		if (ret < 0)
			return ret;
		*val = tach_data;
		break;
	case hwmon_pwm_input:
		u8 cmd = PWM_FAN0 + channel;
		//for some reason the spec does not increment
		// the registers normally. channel 0 starts at
		//0x18 and channel 8 ends at 0x17
		if (channel == 8)
			cmd = 0x17;

		ret = i2c_smbus_read_byte_data(data->client, cmd);
		*val = ret;
		break;
	default:
		return -EOPNOTSUPP;
	}
	return 0;
}


static umode_t quanta_cpld_is_visible(const void *data_in,
				   enum hwmon_sensor_types type,
				   u32 attr, int channel)
{
	const struct quanta_cpld_data *data = data_in;
	int ret;

	switch (type) {
	case hwmon_fan:
		ret = i2c_smbus_read_byte_data(data->client, FAN_PRST);
		if (ret > 0 && channel < NR_CHANNEL) {
			return 0444;
		}
		break; 
	case hwmon_pwm:
		ret = i2c_smbus_read_byte_data(data->client, FAN_PRST);
		if (ret > 0 && channel < NR_CHANNEL_PWM) {
			return 0644;
		}
		break;
	default:
		return 0;
	}
	return 0;
}

//SENSOR_DEVICE_ATTR
static const struct hwmon_channel_info *quanta_cpld_info[] = {
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
			   HWMON_F_INPUT),
	HWMON_CHANNEL_INFO(pwm,
			   HWMON_PWM_INPUT,
			   HWMON_PWM_INPUT,
			   HWMON_PWM_INPUT,
			   HWMON_PWM_INPUT,
			   HWMON_PWM_INPUT,
			   HWMON_PWM_INPUT,
			   HWMON_PWM_INPUT,
			   HWMON_PWM_INPUT,
			   HWMON_PWM_INPUT),
	NULL
};

static const struct hwmon_ops quanta_cpld_hwmon_ops = {
	.is_visible = quanta_cpld_is_visible,
	.read = quanta_cpld_read,
	.write = quanta_cpld_write,
};

static const struct hwmon_chip_info quanta_cpld_chip_info = {
	.ops = &quanta_cpld_hwmon_ops,
	.info = quanta_cpld_info,
};

static int quanta_cpld_init_fan(struct i2c_client *client)
{
	//set the cpld into the pwm mode
	return i2c_smbus_write_byte_data(client, FAN_CONTROL_MODE, 0x01);
}

static int quanta_write_led(struct led_classdev *cdev, u8 val)
{
	struct quanta_led *led = container_of(cdev, struct quanta_led, cdev);
	struct quanta_cpld_data *data = led->data;
	int offset = led - data->leds;
	u8 cmd = LED0 + offset;

	return i2c_smbus_write_byte_data(data->client, cmd, val);
}

static int quanta_brightness_set(struct led_classdev *cdev,
				 enum led_brightness brightness)
{
	u8 val = 1;

	if (brightness != LED_OFF)
		val = 0;
	return quanta_write_led(cdev, val);
}

static int quanta_blink_set(struct led_classdev *cdev,
			    unsigned long *delay_on, unsigned long *delay_off)
{
	u8 val = 1;

	if (*delay_on == 500 && *delay_off == 500)
		val = 2;
	else if (*delay_on == 125 && *delay_off == 125)
		val = 3;
	else
		return -EOPNOTSUPP;

	return quanta_write_led(cdev, val);
}

static int quanta_cpld_init_leds(struct quanta_cpld_data *data)
{
	int i = 0;
	int ret = 0;
	for (i = 0; i < MAX_LED; i++) {
		struct quanta_led *led = &data->leds[i];
		led->cdev.brightness_set_blocking = quanta_brightness_set;
		led->cdev.blink_set = quanta_blink_set;
		led->cdev.name = led_names[i];
		led->data = data;
		ret = devm_led_classdev_register(&data->client->dev, &led->cdev);
		if (ret < 0)
			return ret;
	}
	return 0;
}

static int quanta_cpld_init_client(struct quanta_cpld_data *data)
{
	int ret;

	ret = quanta_cpld_init_fan(data->client);
	if (ret)
		return ret;

	ret = quanta_cpld_init_leds(data);

	return ret;
}

static ssize_t cpld_version_show(struct device *dev,
			      struct device_attribute *attr,
			      char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	u8 cmd = 0x0;
	int ret = 0;

	ret = i2c_smbus_read_byte_data(client, cmd);
	return sprintf(buf, "%x\n", ret);
}
static DEVICE_ATTR_RO(cpld_version);

static int quanta_cpld_probe(struct i2c_client *client)
{
	struct i2c_adapter *adapter = client->adapter;
	struct device *dev = &client->dev;
	struct quanta_cpld_data *data;
	struct device *hwmon_dev;
	int err;

	if (!i2c_check_functionality(adapter,
			I2C_FUNC_SMBUS_BYTE_DATA | I2C_FUNC_SMBUS_WORD_DATA))
		return -ENODEV;

	data = devm_kzalloc(dev, sizeof(struct quanta_cpld_data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->client = client;

	err = quanta_cpld_init_client(data);
	if (err)
		return err;

	device_create_file(dev, &dev_attr_cpld_version);

	hwmon_dev = devm_hwmon_device_register_with_info(dev, client->name,
							 data,
							 &quanta_cpld_chip_info,
							 NULL);

	return PTR_ERR_OR_ZERO(hwmon_dev);
}

static void quanta_cpld_remove(struct i2c_client *client)
{
	device_remove_file(&client->dev, &dev_attr_cpld_version);
}

static const struct i2c_device_id quanta_cpld_id[] = {
	{ "quanta_cg1_cpld", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, quanta_cpld_id);

static struct i2c_driver quanta_cpld_driver = {
	.class		= I2C_CLASS_HWMON,
	.probe_new	= quanta_cpld_probe,
	.remove = quanta_cpld_remove,
	.driver = {
		.name	= "quanta_cg1_cpld",
	},
	.id_table	= quanta_cpld_id,
};

module_i2c_driver(quanta_cpld_driver);

MODULE_AUTHOR("Radivoje (Ogi) Jovanovic <rjovanovic@nvidia.com>");
MODULE_DESCRIPTION("Quanta custom cpld fan/led driver");
MODULE_LICENSE("GPL");
