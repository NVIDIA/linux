// SPDX-License-Identifier: GPL-2.0
/*
 * ADS7142 - Nanopower, Dual-Channel, Programmable Sensor Monitor
 *
 * Copyright (c) 2024 NVIDIA CORPORATION.  All rights reserved.
 *
 */

#include <linux/module.h>
#include <linux/i2c.h>

#include <linux/iio/iio.h>
#include <linux/iio/types.h>
#include <linux/iio/sysfs.h>

#define ADS7142_NAME "ads7142"

#define ADS7142_CHANNELS 2

/* ADS7142 OpCodes */
#define ADS7142_GENERAL_CALL		0x00
#define ADS7142_SINGLE_REG_WRITE 	0x08

/* A general call followed by Soft Reset call triggers device reset */
#define ADS7142_SOFT_RESET_DATA		0x06

/* Configures the analog input channels */
#define ADS7142_CHANNEL_INPUT_CFG_REG	0x24
#define ADS7142_CHANNEL_INPUT_CFG_2CHAN_1END		0x00
#define ADS7142_CHANNEL_INPUT_CFG_1CHAN_1END		0x01
#define ADS7142_CHANNEL_INPUT_CFG_1CHAN_DIFF		0x02

/* Sets the operation mode and enables auto sequencing */
#define ADS7142_OPMODE_SEL_REG		0x1C
#define ADS7142_OPMODE_SEL_MAN_CH0	0x00
#define ADS7142_OPMODE_SEL_MAN_SEQ	0x04
#define ADS7142_OPMODE_SEL_AUTO_MON	0x06
#define ADS7142_OPMODE_SEL_HIGH_PRE	0x07

/* Enables auto sequencing for selected channels */
#define ADS7142_AUTO_SEQ_CHEN_REG	0x20

/* Starts the channel scanning sequence */
#define ADS7142_START_SEQUENCE_REG	0x1E
#define ADS7142_START_SEQUENCE_DATA	0x01

/* Aborts the channel scannin sequence */
#define ADS7142_ABORT_SEQUENCE_REG	0x1F
#define ADS7142_ABORT_SEQUENCE_DATA	0x01

#define ADS7142_CHAN(index) {	\
	.type = IIO_VOLTAGE, \
	.address = index, \
	.indexed = 1, \
	.channel = index, \
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW), \
	.scan_index = index, \
	.scan_type = { \
		.sign = 'u', \
		.realbits = 12, \
		.storagebits = 16, \
		.endianness = IIO_CPU, \
	}, \
	.datasheet_name = "AIN"#index, \
}

static const struct iio_chan_spec ads7142_channels[] = {
	ADS7142_CHAN(0),
	ADS7142_CHAN(1),
	IIO_CHAN_SOFT_TIMESTAMP(ADS7142_CHANNELS),
};

/**
 * struct ads7142_chip_data - chip specifc information
 * @info:		iio core function callbacks structure
 * @channels:		channel specification
 * @num_channels:       number of channels
 */
struct ads7142_chip_data {
	const struct iio_info		*info;
	struct iio_chan_spec const	*channels;
	int				num_channels;
};

/**
 * struct ads7142_state - driver instance specific data
 * @client:		i2c_client
 * @lock:		lock to ensure state is consistent
 * @chip:		chip model specific constants, available modes, etc.
 */
struct ads7142_state {
	struct i2c_client	*client;
	struct mutex 		lock;
	const struct ads7142_chip_data *chip;
};

static int ads7142_reg_write(const struct i2c_client *client, u8 reg, u8 data)
{
	int ret;
	u8 buf[] = {ADS7142_SINGLE_REG_WRITE, reg, data};
	struct i2c_msg msg = {
		.addr = client->addr,
		.len = 3,
		.buf = buf,
		.flags = 0,
	};

	ret = i2c_transfer(client->adapter, &msg, 1);

	return ret < 0 ? ret : 0;
}

static int ads7142_soft_reset(const struct i2c_client *client)
{
	int ret;
	u8 buf[] = {ADS7142_GENERAL_CALL, ADS7142_SOFT_RESET_DATA};
	struct i2c_msg msg = {
		.addr = client->addr,
		.len = 2,
		.buf = buf,
		.flags = 0,
	};

	ret = i2c_transfer(client->adapter, &msg, 1);

	return ret < 0 ? ret : 0;
}

static int ads7142_result_read(const struct i2c_client *client, void *data)
{
	int ret;
	struct i2c_msg msg = {
		.addr = client->addr,
		.len = 2,
		.buf = data,
		.flags = I2C_M_RD,
	};

	ret = i2c_transfer(client->adapter, &msg, 1);

	return ret < 0 ? ret : 0;
}

/* 
 * The sequence follows the flowchart in Firgure 55. Device Operation in Manual
 * Mode in the datasheet.
 */
static int ads7142_read_channel(struct iio_dev *indio_dev, int channel,
								int *val)
{
	struct i2c_client *client = to_i2c_client(indio_dev->dev.parent);
	u16 output;
	int ret;

	ret = iio_device_claim_direct_mode(indio_dev);
	if (ret)
		return ret;

	/* Configure the inputs to Two Channel, Single Ended mode */
	ret = ads7142_reg_write(client,
		ADS7142_CHANNEL_INPUT_CFG_REG, ADS7142_CHANNEL_INPUT_CFG_2CHAN_1END);
	if (ret)
		goto release_direct;

	/* Put Operation Mode in Manual Mode with Auto Sequencing enabled */
	ret = ads7142_reg_write(client,
		ADS7142_OPMODE_SEL_REG, ADS7142_OPMODE_SEL_MAN_SEQ);
	if (ret)
		goto release_direct;

	/* Enable Auto-Sequencing only for requested channel */
	ret = ads7142_reg_write(client,
		ADS7142_AUTO_SEQ_CHEN_REG, 1 << channel);
	if (ret)
		goto release_direct;

	/* Kick start the sequencing */
	ret = ads7142_reg_write(client,
		ADS7142_START_SEQUENCE_REG, ADS7142_START_SEQUENCE_DATA);
	if (ret)
		goto release_direct;

	/* Read the result from the chip */
	ret = ads7142_result_read(client, &output);
	if (ret)
		goto abort;

	/* In manual mode, the 4 LSBs are zeroes.  Shift out to get 12-bit value */
	*val = be16_to_cpu(output);
	*val >>= 4;

abort:
	/* Abort the sequence once reading is complete or aborted */
	ret = ads7142_reg_write(client,
		ADS7142_ABORT_SEQUENCE_REG, ADS7142_ABORT_SEQUENCE_DATA);

release_direct:
	iio_device_release_direct_mode(indio_dev);

	return ret;
}

static int ads7142_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int *val, int *val2, long mask)
{
	int ret;
	struct ads7142_state *state = iio_priv(indio_dev);

	mutex_lock(&state->lock);

	switch(mask) {
	case IIO_CHAN_INFO_RAW:
		ret = ads7142_read_channel(indio_dev, chan->address, val);
		if (!ret)
			ret = IIO_VAL_INT;
		break;
	default:
		ret = -EINVAL;
		break;
	}

	mutex_unlock(&state->lock);

	return ret;
}

static int ads7142_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct iio_dev *indio_dev;
	struct ads7142_state *state;
	int ret;

	indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*state));
	if (!indio_dev)
		return -ENOMEM;

	state = iio_priv(indio_dev);
	i2c_set_clientdata(client, indio_dev);

	mutex_init(&state->lock);

	state->chip = device_get_match_data(&client->dev);
	if (!state->chip)
		state->chip = (const struct ads7142_chip_data *)id->driver_data;
	if (!state->chip)
		return dev_err_probe(&client->dev, -EINVAL, "Unknown chip.\n");

	indio_dev->name = id->name;
	indio_dev->channels = state->chip->channels;
	indio_dev->num_channels = state->chip->num_channels;
	indio_dev->info = state->chip->info;
	indio_dev->modes = INDIO_DIRECT_MODE;

	ret = ads7142_soft_reset(client);
	if (ret)
		return ret;

	ret = iio_device_register(indio_dev);
	if (ret < 0) {
		dev_err(&client->dev, "Failed to register IIO device\n");
		return ret;
	}

	return 0;
}

static void ads7142_remove(struct i2c_client *client)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(client);

	iio_device_unregister(indio_dev);
}

static const struct iio_info ads7142_info = {
	.read_raw	= ads7142_read_raw,
};

static const struct ads7142_chip_data ads7142_data = {
	.info		= &ads7142_info,
	.channels	= ads7142_channels,
	.num_channels	= ARRAY_SIZE(ads7142_channels),
};

static const struct i2c_device_id ads7142_id[] = {
	{ ADS7142_NAME, (kernel_ulong_t)&ads7142_data },
	{}
};
MODULE_DEVICE_TABLE(i2c, ads7142_id);

static const struct of_device_id ads7142_of_match[] = {
	{ .compatible = "ti,ads7142", .data = &ads7142_data },
	{}
};
MODULE_DEVICE_TABLE(of, ads7142_of_match);

static struct i2c_driver ads7142_driver = {
	.driver = {
		.name = ADS7142_NAME,
		.of_match_table = ads7142_of_match,
	},
	.probe		= ads7142_probe,
	.remove		= ads7142_remove,
	.id_table	= ads7142_id,
};

module_i2c_driver(ads7142_driver);

MODULE_AUTHOR("Howard Tang <howardt@nvidia.com>");
MODULE_DESCRIPTION("Texas Instruments ADS7142 ADC driver");
MODULE_LICENSE("GPL v2");
