// SPDX-License-Identifier: GPL-2.0+
/*
 * Hardware monitoring driver for Infineon Multi-phase Digital XDPE1A2G5B,
 * XDPE1A2G7B, and XDPE1A2G7C Controllers
 *
 * Copyright (c) 2026 Infineon Technologies. All rights reserved.
 */

#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include "pmbus.h"

#define XDPE1A2G7B_PAGE_NUM 2
#define XDPE1A2G7B_NVIDIA_195MV 0x1E /* NVIDIA mode 1.95mV, VID step is 5mV */

enum xdpe1a2g7b_chip {
	xdpe1a2g5b = 1,
	xdpe1a2g7b,
	xdpe1a2g7c,
};

static int xdpe1a2g7c_force_page0(struct i2c_client *client, int page)
{
	int ret;

	/* PAGE1 is an unused loop with an incompatible VOUT format. */
	if (page == 1)
		return -ENXIO;

	/*
	 * The PAGE selector can be reset independently of the PMBus core's
	 * cached page. Force PAGE0 before every access to the monitored loop.
	 */
	ret = i2c_smbus_write_byte_data(client, PMBUS_PAGE, 0);
	if (ret < 0)
		return ret;

	return -ENODATA;
}

static int xdpe1a2g7c_read_byte_data(struct i2c_client *client, int page,
				     int reg)
{
	return xdpe1a2g7c_force_page0(client, page);
}

static int xdpe1a2g7c_read_word_data(struct i2c_client *client, int page,
				     int phase, int reg)
{
	return xdpe1a2g7c_force_page0(client, page);
}

static int xdpe1a2g7b_set_vout_mode(struct pmbus_driver_info *info, int page,
				    int vout_mode)
{
	u8 vout_params;

	switch (vout_mode >> 5) {
	case 0:
		info->format[PSC_VOLTAGE_OUT] = linear;
		break;
	case 1:
		info->format[PSC_VOLTAGE_OUT] = vid;
		vout_params = vout_mode & GENMASK(4, 0);
		/* Check for VID Code Type */
		switch (vout_params) {
		case XDPE1A2G7B_NVIDIA_195MV:
			info->vrm_version[page] = nvidia195mv;
			break;
		default:
			return -EINVAL;
		}
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int xdpe1a2g7b_identify(struct i2c_client *client,
			       struct pmbus_driver_info *info)
{
	int ret;
	int vout_mode;

	/*
	 * XDPE1A2G5B and XDPE1A2G7B support both Linear and NVIDIA PWM VID data
	 * formats via VOUT_MODE. Note that the device pages/loops are not fully
	 * independent: configuration is shared, so programming each page/loop
	 * separately is not supported.
	 */
	vout_mode = pmbus_read_byte_data(client, 0, PMBUS_VOUT_MODE);
	if (vout_mode < 0)
		return vout_mode;

	ret = xdpe1a2g7b_set_vout_mode(info, 0, vout_mode);
	if (!ret && info->format[PSC_VOLTAGE_OUT] == vid)
		info->vrm_version[1] = info->vrm_version[0];

	return ret;
}

static int xdpe1a2g7c_identify(struct i2c_client *client,
			       struct pmbus_driver_info *info)
{
	int operation;
	int page0_mode;
	int page1_mode;
	int restore;
	int status;
	int ret;

	page0_mode = pmbus_read_byte_data(client, 0, PMBUS_VOUT_MODE);
	if (page0_mode < 0)
		return page0_mode;

	ret = xdpe1a2g7b_set_vout_mode(info, 0, page0_mode);
	if (ret)
		return ret;

	page1_mode = pmbus_read_byte_data(client, 1, PMBUS_VOUT_MODE);
	if (page1_mode < 0) {
		ret = page1_mode;
		goto restore_page0;
	}

	/* The core supports different Linear16 exponents on individual pages. */
	if ((page0_mode >> 5) == (page1_mode >> 5)) {
		ret = xdpe1a2g7b_set_vout_mode(info, 1, page1_mode);
		goto restore_page0;
	}

	/*
	 * An active loop with a different VOUT format cannot be represented by
	 * the PMBus core. Only hide PAGE1 when both OPERATION and STATUS_WORD
	 * confirm that the differently configured loop is off.
	 */
	operation = pmbus_read_byte_data(client, 1, PMBUS_OPERATION);
	if (operation < 0) {
		ret = operation;
		goto restore_page0;
	}

	status = pmbus_read_word_data(client, 1, 0xff, PMBUS_STATUS_WORD);
	if (status < 0) {
		ret = status;
		goto restore_page0;
	}

	if ((operation & PB_OPERATION_CONTROL_ON) || !(status & PB_STATUS_OFF)) {
		dev_err(&client->dev,
			"PAGE1 is active with incompatible VOUT_MODE 0x%02x\n",
			page1_mode);
		ret = -ENODEV;
		goto restore_page0;
	}

	info->func[1] = 0;
	info->read_byte_data = xdpe1a2g7c_read_byte_data;
	info->read_word_data = xdpe1a2g7c_read_word_data;

	ret = 0;

restore_page0:
	restore = pmbus_set_page(client, 0, 0xff);
	if (!ret)
		ret = restore;

	return ret;
}

static struct pmbus_driver_info xdpe1a2g7b_info = {
	.pages = XDPE1A2G7B_PAGE_NUM,
	.identify = xdpe1a2g7b_identify,
	.format[PSC_VOLTAGE_IN] = linear,
	.format[PSC_TEMPERATURE] = linear,
	.format[PSC_CURRENT_IN] = linear,
	.format[PSC_CURRENT_OUT] = linear,
	.format[PSC_POWER] = linear,
	.func[0] = PMBUS_HAVE_VIN | PMBUS_HAVE_VOUT | PMBUS_HAVE_STATUS_VOUT |
		   PMBUS_HAVE_IIN | PMBUS_HAVE_IOUT | PMBUS_HAVE_STATUS_IOUT |
		   PMBUS_HAVE_TEMP | PMBUS_HAVE_TEMP2 | PMBUS_HAVE_STATUS_TEMP |
		   PMBUS_HAVE_POUT | PMBUS_HAVE_PIN | PMBUS_HAVE_STATUS_INPUT,
	.func[1] = PMBUS_HAVE_VIN | PMBUS_HAVE_VOUT | PMBUS_HAVE_STATUS_VOUT |
		   PMBUS_HAVE_IIN | PMBUS_HAVE_IOUT | PMBUS_HAVE_STATUS_IOUT |
		   PMBUS_HAVE_PIN | PMBUS_HAVE_POUT | PMBUS_HAVE_STATUS_INPUT,
};

static int xdpe1a2g7b_probe(struct i2c_client *client)
{
	struct pmbus_driver_info *info;
	enum xdpe1a2g7b_chip chip;

	chip = (kernel_ulong_t)i2c_get_match_data(client);

	info = devm_kmemdup(&client->dev, &xdpe1a2g7b_info, sizeof(*info),
			    GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	/* Keep both physical pages so the core always manages the PAGE selector. */
	if (chip == xdpe1a2g7c)
		info->identify = xdpe1a2g7c_identify;

	return pmbus_do_probe(client, info);
}

static const struct i2c_device_id xdpe1a2g7b_id[] = {
	{ "xdpe1a2g5b", xdpe1a2g5b },
	{ "xdpe1a2g7b", xdpe1a2g7b },
	{ "xdpe1a2g7c", xdpe1a2g7c },
	{}
};

MODULE_DEVICE_TABLE(i2c, xdpe1a2g7b_id);

static const struct of_device_id __maybe_unused xdpe1a2g7b_of_match[] = {
	{ .compatible = "infineon,xdpe1a2g5b", .data = (void *)xdpe1a2g5b },
	{ .compatible = "infineon,xdpe1a2g7b", .data = (void *)xdpe1a2g7b },
	{ .compatible = "infineon,xdpe1a2g7c", .data = (void *)xdpe1a2g7c },
	{}
};

MODULE_DEVICE_TABLE(of, xdpe1a2g7b_of_match);

static struct i2c_driver xdpe1a2g7b_driver = {
	.driver = {
		.name = "xdpe1a2g7b",
		.of_match_table = of_match_ptr(xdpe1a2g7b_of_match),
	},
	.probe = xdpe1a2g7b_probe,
	.id_table = xdpe1a2g7b_id,
};

module_i2c_driver(xdpe1a2g7b_driver);

MODULE_AUTHOR("Ashish Yadav <ashish.yadav@infineon.com>");
MODULE_DESCRIPTION("PMBus driver for Infineon XDPE1A2G5B/7B/7C");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS("PMBUS");
