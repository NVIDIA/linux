// SPDX-License-Identifier: GPL-2.0-only
/*
 * Fake ARA target for ast 2600 soc Old i2c register mode
 *
 * Copyright (c) 2016, NVIDIA CORPORATION.  All rights reserved.
 */

#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/aspeed-2600-ara.h>
#include <linux/i2c-aspeed.h>

#define ARA_ADDRESS 0xC

/*
 * disable_ast2600_ara:
 * input:
 * i2c client structure for the target doing ssif traffic.
 * the structure is necessary so the method would know on which
 * i2c adapter ARA is present.
 */
static void disable_ast2600_ara(struct i2c_client *client)
{
	u32 addr_reg_val;
	struct aspeed_i2c_bus *bus;

	if (!client)
		return;

	bus = i2c_get_adapdata(client->adapter);
	addr_reg_val = readl(bus->base + ASPEED_I2C_DEV_ADDR_REG);
	if ((addr_reg_val & 0x7F) == ARA_ADDRESS)
	{
		addr_reg_val &= 0xFFFFFF7F;
		writel(addr_reg_val, bus->base + ASPEED_I2C_DEV_ADDR_REG);
	}
	if (((addr_reg_val >> 8) & 0x7F) == ARA_ADDRESS)
	{
		addr_reg_val &= 0xFFFF7FFF;
		writel(addr_reg_val, bus->base + ASPEED_I2C_DEV_ADDR_REG);
	}
	if (((addr_reg_val >> 16) & 0x7F) == ARA_ADDRESS)
	{
		addr_reg_val &= 0xFF7FFFFF;
		writel(addr_reg_val, bus->base + ASPEED_I2C_DEV_ADDR_REG);
	}
}

/*
 * enable_ast2600_ara:
 * input:
 * i2c client structure for the target doing ssif traffic.
 * the structure is necessary so the method would know on which
 * i2c adapter ARA is present.
 */
void enable_ast2600_ara(struct i2c_client *client)
{
	u32 addr_reg_val;
	struct aspeed_i2c_bus *bus;

	if (!client)
		return;

	bus = i2c_get_adapdata(client->adapter);
	addr_reg_val = readl(bus->base + ASPEED_I2C_DEV_ADDR_REG);
	if ((addr_reg_val & 0x7F) == ARA_ADDRESS)
	{
		addr_reg_val |= 0x80;
		writel(addr_reg_val, bus->base + ASPEED_I2C_DEV_ADDR_REG);
	}
	else if (((addr_reg_val >> 8) & 0x7F) == ARA_ADDRESS)
	{
		addr_reg_val |= 0x8000;
		writel(addr_reg_val, bus->base + ASPEED_I2C_DEV_ADDR_REG);
	}
	else if (((addr_reg_val >> 16) & 0x7F) == ARA_ADDRESS)
	{
		addr_reg_val |= 0x800000;
		writel(addr_reg_val, bus->base + ASPEED_I2C_DEV_ADDR_REG);
	}
}
EXPORT_SYMBOL(enable_ast2600_ara);

static int ast2600_ara_cb(struct i2c_client *client, enum i2c_slave_event event, u8 *val)
{
	int ret = 0;
	struct ast2600_ara *ara = i2c_get_clientdata(client);
	switch (event) {
	case I2C_SLAVE_READ_REQUESTED:
		*val = (ara->client->addr << 1);
		disable_ast2600_ara(client);
		break;
	case I2C_SLAVE_WRITE_REQUESTED:
	case I2C_SLAVE_READ_PROCESSED:
	case I2C_SLAVE_WRITE_RECEIVED:
	case I2C_SLAVE_STOP:
	default:
		break;
	}

	return ret;
}

struct ast2600_ara* register_ast2600_ara(struct i2c_client *client)
{
	int ret;
	struct ast2600_ara *ara;
	struct i2c_adapter *adap;
	struct i2c_board_info info;
	char name[] ="ast2600-ara";

	if (!client) {
		return NULL;
	}
	adap = client->adapter;
	memset(&info, 0, sizeof(struct i2c_board_info));
	memcpy(info.type, name, 11);
	info.addr = ARA_ADDRESS;
	info.flags |= I2C_CLIENT_SLAVE;

	ara = devm_kzalloc(&client->dev, sizeof(*ara), GFP_KERNEL);
	if(!ara)
		return NULL;
	ara->client = client;
	ara->ara_i2c_client = i2c_new_client_device(adap, &info);
	if (IS_ERR(ara->ara_i2c_client)) {
		devm_kfree(&client->dev, ara);
		return NULL;
	}
	i2c_set_clientdata(ara->ara_i2c_client, ara);
	ret = i2c_slave_register(ara->ara_i2c_client, ast2600_ara_cb);
	if (ret) {
		devm_kfree(&client->dev, ara);
		return NULL;
	}

	disable_ast2600_ara(client);

	return ara;
}

EXPORT_SYMBOL(register_ast2600_ara);

void unregister_ast2600_ara(struct ast2600_ara *ara)
{
	if (ara)
		i2c_slave_unregister(ara->ara_i2c_client);
}
EXPORT_SYMBOL(unregister_ast2600_ara);

MODULE_AUTHOR("Radivoje Jovanovic<rjovanovic@nvidia.com>");
MODULE_DESCRIPTION("ast2600 ara support");
MODULE_LICENSE("GPL");
