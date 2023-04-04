// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) IBM Corporation 2020
 */

#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/limits.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/spinlock.h>

<<<<<<< HEAD
#define DEVICE_NAME	"ibm-panel"
=======
#define DEVICE_NAME		"ibm-panel"
#define PANEL_KEYCODES_COUNT	3
>>>>>>> origin/linux_6.1.15_upstream

struct ibm_panel {
	u8 idx;
	u8 command[11];
<<<<<<< HEAD
=======
	u32 keycodes[PANEL_KEYCODES_COUNT];
>>>>>>> origin/linux_6.1.15_upstream
	spinlock_t lock;	/* protects writes to idx and command */
	struct input_dev *input;
};

<<<<<<< HEAD
static void ibm_panel_process_command(struct ibm_panel *panel)
{
	u8 i;
	u8 chksum;
	u16 sum = 0;
	int pressed;
	int released;

	if (panel->command[0] != 0xff && panel->command[1] != 0xf0) {
		dev_dbg(&panel->input->dev, "command invalid\n");
		return;
	}
=======
static u8 ibm_panel_calculate_checksum(struct ibm_panel *panel)
{
	u8 chksum;
	u16 sum = 0;
	unsigned int i;
>>>>>>> origin/linux_6.1.15_upstream

	for (i = 0; i < sizeof(panel->command) - 1; ++i) {
		sum += panel->command[i];
		if (sum & 0xff00) {
			sum &= 0xff;
			sum++;
		}
	}

	chksum = sum & 0xff;
	chksum = ~chksum;
	chksum++;

<<<<<<< HEAD
	if (chksum != panel->command[sizeof(panel->command) - 1]) {
		dev_dbg(&panel->input->dev, "command failed checksum\n");
		return;
	}

	released = panel->command[2] & 0x80;
	pressed = released ? 0 : 1;

	switch (panel->command[2] & 0xf) {
	case 0:
		input_report_key(panel->input, BTN_NORTH, pressed);
		break;
	case 1:
		input_report_key(panel->input, BTN_SOUTH, pressed);
		break;
	case 2:
		input_report_key(panel->input, BTN_SELECT, pressed);
		break;
	default:
		dev_dbg(&panel->input->dev, "unknown command %u\n",
			panel->command[2] & 0xf);
		return;
	}

	input_sync(panel->input);
=======
	return chksum;
}

static void ibm_panel_process_command(struct ibm_panel *panel)
{
	u8 button;
	u8 chksum;

	if (panel->command[0] != 0xff && panel->command[1] != 0xf0) {
		dev_dbg(&panel->input->dev, "command invalid: %02x %02x\n",
			panel->command[0], panel->command[1]);
		return;
	}

	chksum = ibm_panel_calculate_checksum(panel);
	if (chksum != panel->command[sizeof(panel->command) - 1]) {
		dev_dbg(&panel->input->dev,
			"command failed checksum: %u != %u\n", chksum,
			panel->command[sizeof(panel->command) - 1]);
		return;
	}

	button = panel->command[2] & 0xf;
	if (button < PANEL_KEYCODES_COUNT) {
		input_report_key(panel->input, panel->keycodes[button],
				 !(panel->command[2] & 0x80));
		input_sync(panel->input);
	} else {
		dev_dbg(&panel->input->dev, "unknown button %u\n",
			button);
	}
>>>>>>> origin/linux_6.1.15_upstream
}

static int ibm_panel_i2c_slave_cb(struct i2c_client *client,
				  enum i2c_slave_event event, u8 *val)
{
	unsigned long flags;
	struct ibm_panel *panel = i2c_get_clientdata(client);

	dev_dbg(&panel->input->dev, "event: %u data: %02x\n", event, *val);

	spin_lock_irqsave(&panel->lock, flags);

	switch (event) {
	case I2C_SLAVE_STOP:
		if (panel->idx == sizeof(panel->command))
			ibm_panel_process_command(panel);
		else
			dev_dbg(&panel->input->dev,
				"command incorrect size %u\n", panel->idx);
		fallthrough;
	case I2C_SLAVE_WRITE_REQUESTED:
		panel->idx = 0;
		break;
	case I2C_SLAVE_WRITE_RECEIVED:
		if (panel->idx < sizeof(panel->command))
			panel->command[panel->idx++] = *val;
		else
			/*
			 * The command is too long and therefore invalid, so set the index
			 * to it's largest possible value. When a STOP is finally received,
			 * the command will be rejected upon processing.
			 */
			panel->idx = U8_MAX;
		break;
	case I2C_SLAVE_READ_REQUESTED:
	case I2C_SLAVE_READ_PROCESSED:
		*val = 0xff;
		break;
	default:
		break;
	}

	spin_unlock_irqrestore(&panel->lock, flags);

	return 0;
}

static int ibm_panel_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
<<<<<<< HEAD
	int rc;
	struct ibm_panel *panel = devm_kzalloc(&client->dev, sizeof(*panel),
					       GFP_KERNEL);

	if (!panel)
		return -ENOMEM;

=======
	struct ibm_panel *panel;
	int i;
	int error;

	panel = devm_kzalloc(&client->dev, sizeof(*panel), GFP_KERNEL);
	if (!panel)
		return -ENOMEM;

	spin_lock_init(&panel->lock);

>>>>>>> origin/linux_6.1.15_upstream
	panel->input = devm_input_allocate_device(&client->dev);
	if (!panel->input)
		return -ENOMEM;

	panel->input->name = client->name;
	panel->input->id.bustype = BUS_I2C;
<<<<<<< HEAD
	input_set_capability(panel->input, EV_KEY, BTN_NORTH);
	input_set_capability(panel->input, EV_KEY, BTN_SOUTH);
	input_set_capability(panel->input, EV_KEY, BTN_SELECT);

	rc = input_register_device(panel->input);
	if (rc) {
		dev_err(&client->dev, "Failed to register input device: %d\n",
			rc);
		return rc;
	}

	spin_lock_init(&panel->lock);

	i2c_set_clientdata(client, panel);
	rc = i2c_slave_register(client, ibm_panel_i2c_slave_cb);
	if (rc) {
		input_unregister_device(panel->input);
		return rc;
=======

	error = device_property_read_u32_array(&client->dev,
					       "linux,keycodes",
					       panel->keycodes,
					       PANEL_KEYCODES_COUNT);
	if (error) {
		/*
		 * Use gamepad buttons as defaults for compatibility with
		 * existing applications.
		 */
		panel->keycodes[0] = BTN_NORTH;
		panel->keycodes[1] = BTN_SOUTH;
		panel->keycodes[2] = BTN_SELECT;
	}

	for (i = 0; i < PANEL_KEYCODES_COUNT; ++i)
		input_set_capability(panel->input, EV_KEY, panel->keycodes[i]);

	error = input_register_device(panel->input);
	if (error) {
		dev_err(&client->dev,
			"Failed to register input device: %d\n", error);
		return error;
	}

	i2c_set_clientdata(client, panel);
	error = i2c_slave_register(client, ibm_panel_i2c_slave_cb);
	if (error) {
		dev_err(&client->dev,
			"Failed to register as i2c slave: %d\n", error);
		return error;
>>>>>>> origin/linux_6.1.15_upstream
	}

	return 0;
}

<<<<<<< HEAD
static int ibm_panel_remove(struct i2c_client *client)
{
	int rc;
	struct ibm_panel *panel = i2c_get_clientdata(client);

	rc = i2c_slave_unregister(client);

	input_unregister_device(panel->input);

	return rc;
=======
static void ibm_panel_remove(struct i2c_client *client)
{
	i2c_slave_unregister(client);
>>>>>>> origin/linux_6.1.15_upstream
}

static const struct of_device_id ibm_panel_match[] = {
	{ .compatible = "ibm,op-panel" },
	{ }
};
<<<<<<< HEAD
=======
MODULE_DEVICE_TABLE(of, ibm_panel_match);
>>>>>>> origin/linux_6.1.15_upstream

static struct i2c_driver ibm_panel_driver = {
	.driver = {
		.name = DEVICE_NAME,
		.of_match_table = ibm_panel_match,
	},
	.probe = ibm_panel_probe,
	.remove = ibm_panel_remove,
};
module_i2c_driver(ibm_panel_driver);

MODULE_AUTHOR("Eddie James <eajames@linux.ibm.com>");
MODULE_DESCRIPTION("IBM Operation Panel Driver");
MODULE_LICENSE("GPL");
