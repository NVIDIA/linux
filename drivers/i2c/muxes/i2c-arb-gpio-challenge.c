// SPDX-License-Identifier: GPL-2.0-only
/*
 * GPIO-based I2C Arbitration Using a Challenge & Response Mechanism
 *
 * Copyright (C) 2012 Google, Inc
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/i2c-mux.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#if CONFIG_DRAGON_CHASSIS
#include <linux/device.h>
#include <linux/workqueue.h>
#include <linux/spinlock.h>
#define BUS_TIMEOUT 600
#endif //CONFIG_DRAGON_CHASSIS

/**
 * struct i2c_arbitrator_data - Driver data for I2C arbitrator
 *
 * @our_gpio: GPIO descriptor we'll use to claim.
 * @their_gpio: GPIO descriptor that the other side will use to claim.
 * @slew_delay_us: microseconds to wait for a GPIO to go high.
 * @wait_retry_us: we'll attempt another claim after this many microseconds.
 * @wait_free_us: we'll give up after this many microseconds.
 */

struct i2c_arbitrator_data {
	struct gpio_desc *our_gpio;
	struct gpio_desc *their_gpio;
	struct gpio_desc *cpld_refresh;
	unsigned int slew_delay_us;
	unsigned int wait_retry_us;
	unsigned int wait_free_us;
	struct timer_list timer;
	struct i2c_mux_core *muxc;
#if CONFIG_DRAGON_CHASSIS
	struct i2c_adapter *cpld_access;
	u8 cpld_address;
	bool bus_locked;
	struct workqueue_struct *wq;
	struct delayed_work dwork;
	spinlock_t bus_locked_spin;
	int cnt;
#endif //CONFIG_DRAGON_CHASSIS
};


/*
 * i2c_arbitrator_select - claim the I2C bus
 *
 * Use the GPIO-based signalling protocol; return -EBUSY if we fail.
 */
static int i2c_arbitrator_select(struct i2c_mux_core *muxc, u32 chan)
{
	const struct i2c_arbitrator_data *arb = i2c_mux_priv(muxc);
	unsigned long stop_retry, stop_time;

#if CONFIG_DRAGON_CHASSIS
	if (!IS_ERR(arb->cpld_refresh)) {
		if (gpiod_get_value(arb->cpld_refresh))
			return -EBUSY;
	}

	if (arb->bus_locked)
		return 0;

#endif //CONFIG_DRAGON_CHASSIS

	/* Start a round of trying to claim the bus */
	stop_time = jiffies + usecs_to_jiffies(arb->wait_free_us) + 1;
	do {
		/* Indicate that we want to claim the bus */
		gpiod_set_value(arb->our_gpio, 1);
		udelay(arb->slew_delay_us);

		/* Wait for the other master to release it */
		stop_retry = jiffies + usecs_to_jiffies(arb->wait_retry_us) + 1;
		while (time_before(jiffies, stop_retry)) {
			int gpio_val = gpiod_get_value(arb->their_gpio);

			if (gpio_val) {
				/* We got it, so return */
				return 0;
			}

			usleep_range(50, 200);
		}

		/* It didn't release, so give up, wait, and try again */
		gpiod_set_value(arb->our_gpio, 0);

		usleep_range(arb->wait_retry_us, arb->wait_retry_us * 2);
	} while (time_before(jiffies, stop_time));

	/* Give up, release our claim */
	gpiod_set_value(arb->our_gpio, 0);
	udelay(arb->slew_delay_us);
	dev_err(muxc->dev, "Could not claim bus, timeout\n");
	return -EBUSY;
}

/*
 * i2c_arbitrator_deselect - release the I2C bus
 *
 * Release the I2C bus using the GPIO-based signalling protocol.
 */
static int i2c_arbitrator_deselect(struct i2c_mux_core *muxc, u32 chan)
{
	const struct i2c_arbitrator_data *arb = i2c_mux_priv(muxc);

#if CONFIG_DRAGON_CHASSIS
	if (arb->bus_locked) //if bus is locked do not deselect it
		return 0;
#endif //CONFIG_DRAGON_CHASSIS

	/* Release the bus and wait for the other master to notice */
	gpiod_set_value(arb->our_gpio, 0);
	udelay(arb->slew_delay_us);

	return 0;
}

#if CONFIG_DRAGON_CHASSIS
static ssize_t gpio_our_show(struct device *dev,
				 struct device_attribute *attr,
				 char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct i2c_mux_core *muxc = i2c_get_clientdata(client);
	const struct i2c_arbitrator_data *arb = i2c_mux_priv(muxc);
	int gpio_val = -1;

	gpio_val = gpiod_get_value(arb->our_gpio);

	return sprintf(buf, "%d\n", gpio_val);
}
static DEVICE_ATTR_RO(gpio_our);

static ssize_t gpio_their_show(struct device *dev,
				 struct device_attribute *attr,
				 char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct i2c_mux_core *muxc = i2c_get_clientdata(client);
	const struct i2c_arbitrator_data *arb = i2c_mux_priv(muxc);
	int gpio_val = -1;

	gpio_val = gpiod_get_value(arb->their_gpio);

	return sprintf(buf, "%d\n", gpio_val);
}
static DEVICE_ATTR_RO(gpio_their);

static ssize_t cpld_refresh_show(struct device *dev,
				 struct device_attribute *attr,
				 char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct i2c_mux_core *muxc = i2c_get_clientdata(client);
	const struct i2c_arbitrator_data *arb = i2c_mux_priv(muxc);
	int gpio_val = -1;

	if (!IS_ERR(arb->cpld_refresh))
		gpio_val = gpiod_get_value(arb->cpld_refresh);

	return sprintf(buf, "%d\n", gpio_val);
}
static DEVICE_ATTR_RO(cpld_refresh);

static ssize_t arb_state_show(struct device *dev,
			      struct device_attribute *attr,
			      char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct i2c_mux_core *muxc = i2c_get_clientdata(client);
	const struct i2c_arbitrator_data *arb = i2c_mux_priv(muxc);

	int gpio_val = gpiod_get_value(arb->their_gpio);

	if (!IS_ERR(arb->cpld_refresh)) {
		if (gpiod_get_value(arb->cpld_refresh))
			return -EBUSY;
	}

	return sprintf(buf, "%d\n", gpio_val);
}

static ssize_t arb_state_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct i2c_mux_core *muxc = i2c_get_clientdata(client);
	struct i2c_arbitrator_data *arb = i2c_mux_priv(muxc);
	int val;
	int ret;

	ret = kstrtoint(buf, 0, &val);
	if (ret < 0)
		return ret;

	if (val != 0 && val != 1)
		return -EINVAL;

	if (!val) {
		spin_lock(&arb->bus_locked_spin);
		arb->bus_locked = false;
		spin_unlock(&arb->bus_locked_spin);
		cancel_delayed_work_sync(&arb->dwork);
		i2c_arbitrator_deselect(muxc, 0);
	} else {
		ret = i2c_arbitrator_select(muxc, 0);
		if (ret < 0)
			return ret;
		spin_lock(&arb->bus_locked_spin);
		arb->bus_locked = true;
		arb->cnt = 0;
		spin_unlock(&arb->bus_locked_spin);
		queue_delayed_work(arb->wq, &arb->dwork, 0);
	}
	return count;
}
static DEVICE_ATTR_RW(arb_state);

static void pet_cpld_register(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct i2c_arbitrator_data *arb = container_of(dwork, struct i2c_arbitrator_data, dwork);
	struct i2c_mux_core *muxc = arb->muxc;
	int gpio_val;
	u8 out_buf0[2];
	u8 out_buf1[2];
	struct i2c_msg msgs[] = {
		{
			.addr = arb->cpld_address,
			.flags = 0,
			.len = 2,
			.buf = out_buf0,
		},
		{
			.addr = arb->cpld_address,
			.flags = 0,
			.len = 2,
			.buf = out_buf1,
		}
	};

	out_buf0[0] = 16;
	out_buf0[1] = 0x20;
	out_buf1[0] = 16;
	out_buf1[1] = 0x10;

	gpio_val = gpiod_get_value(arb->their_gpio);

	if (gpio_val)
	{
		if (i2c_transfer(arb->cpld_access, msgs, 2) != 2) {
			dev_err(muxc->dev, "Could not access cpld to pet the watchdog\n");
			spin_lock(&arb->bus_locked_spin);
			arb->bus_locked = false;
			spin_unlock(&arb->bus_locked_spin);
			i2c_arbitrator_deselect(muxc, 0);

		}
		else {
			spin_lock(&arb->bus_locked_spin);
			arb->cnt++;
			spin_unlock(&arb->bus_locked_spin);
			if (arb->cnt >= BUS_TIMEOUT) {
				dev_err(muxc->dev, "Bus was held for more than %d seconds. Cancel arbitration\n", BUS_TIMEOUT);
				spin_lock(&arb->bus_locked_spin);
				arb->bus_locked = false;
				spin_unlock(&arb->bus_locked_spin);
				i2c_arbitrator_deselect(muxc, 0);
			}
			else
				queue_delayed_work(arb->wq, &arb->dwork, msecs_to_jiffies(1000));
		}
	}
	else
	{
			spin_lock(&arb->bus_locked_spin);
			arb->bus_locked = false;
			spin_unlock(&arb->bus_locked_spin);
			i2c_arbitrator_deselect(muxc, 0);
			dev_err(muxc->dev, "Arbitration was unexpectedly dropped.\n");
	}
}
#endif //CONFIG_DRAGON_CHASSIS

static int i2c_arbitrator_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct device_node *parent_np;
	struct i2c_mux_core *muxc;
	struct i2c_arbitrator_data *arb;
	struct gpio_desc *dummy;
	int ret;

#if CONFIG_DRAGON_CHASSIS
	struct device_node *cpld_np;
#endif //CONFIG_DRAGON_CHASSIS

	/* We only support probing from device tree; no platform_data */
	if (!np) {
		dev_err(dev, "Cannot find device tree node\n");
		return -ENODEV;
	}
	if (dev_get_platdata(dev)) {
		dev_err(dev, "Platform data is not supported\n");
		return -EINVAL;
	}

	muxc = i2c_mux_alloc(NULL, dev, 1, sizeof(*arb), I2C_MUX_ARBITRATOR,
			     i2c_arbitrator_select, i2c_arbitrator_deselect);
	if (!muxc)
		return -ENOMEM;
	arb = i2c_mux_priv(muxc);
	arb->muxc = muxc;

	platform_set_drvdata(pdev, muxc);

	/* Request GPIOs, our GPIO as unclaimed to begin with */
	arb->our_gpio = devm_gpiod_get(dev, "our-claim", GPIOD_OUT_LOW);
	if (IS_ERR(arb->our_gpio)) {
		dev_err(dev, "could not get \"our-claim\" GPIO (%ld)\n",
			PTR_ERR(arb->our_gpio));
		return PTR_ERR(arb->our_gpio);
	}

	arb->their_gpio = devm_gpiod_get(dev, "their-claim", GPIOD_IN);
	if (IS_ERR(arb->their_gpio)) {
		dev_err(dev, "could not get \"their-claim\" GPIO (%ld)\n",
			PTR_ERR(arb->their_gpio));
		return PTR_ERR(arb->their_gpio);
	}

	arb->cpld_refresh = devm_gpiod_get(dev, "cpld-refresh", GPIOD_IN);

	/* At the moment we only support a single two master (us + 1 other) */
	dummy = devm_gpiod_get_index(dev, "their-claim", 1, GPIOD_IN);
	if (!IS_ERR(dummy)) {
		dev_err(dev, "Only one other master is supported\n");
		return -EINVAL;
	} else if (PTR_ERR(dummy) == -EPROBE_DEFER) {
		return -EPROBE_DEFER;
	}

	/* Arbitration parameters */
	if (of_property_read_u32(np, "slew-delay-us", &arb->slew_delay_us))
		arb->slew_delay_us = 10;
	if (of_property_read_u32(np, "wait-retry-us", &arb->wait_retry_us))
		arb->wait_retry_us = 3000;
	if (of_property_read_u32(np, "wait-free-us", &arb->wait_free_us))
		arb->wait_free_us = 50000;

#if CONFIG_DRAGON_CHASSIS
	/* Find our cpld i2c bus */
	cpld_np = of_parse_phandle(np, "i2c-cpld", 0);
	if (!cpld_np) {
		dev_err(dev, "Cannot parse i2c-cpld\n");
		return -EINVAL;
	}
	arb->cpld_access = of_get_i2c_adapter_by_node(cpld_np);
	of_node_put(cpld_np);
	if (!arb->cpld_access) {
		dev_err(dev, "Cannot find cpld bus\n");
		return -EPROBE_DEFER;
	}
	if (of_property_read_u8(np, "cpld-address", &arb->cpld_address)) {
		dev_err(dev, "cpld-address must be present in dts\n");
		return -EINVAL;
	}

	arb->wq = create_singlethread_workqueue("i2c_arb_events");
	if (!arb->wq) {
		dev_err(dev, "Cannot create work queue\n");
		return -ENOMEM;
	}
	spin_lock_init(&arb->bus_locked_spin);
#endif //CONFIG_DRAGON_CHASSIS

	/* Find our parent */
	parent_np = of_parse_phandle(np, "i2c-parent", 0);
	if (!parent_np) {
		dev_err(dev, "Cannot parse i2c-parent\n");
		return -EINVAL;
	}
	muxc->parent = of_get_i2c_adapter_by_node(parent_np);
	of_node_put(parent_np);
	if (!muxc->parent) {
		dev_err(dev, "Cannot find parent bus\n");
		return -EPROBE_DEFER;
	}

	/* Actually add the mux adapter */
	ret = i2c_mux_add_adapter(muxc, 0, 0, 0);
	if (ret)
		i2c_put_adapter(muxc->parent);

#if CONFIG_DRAGON_CHASSIS
	INIT_DELAYED_WORK(&arb->dwork, pet_cpld_register);
	device_create_file(dev, &dev_attr_arb_state);
	device_create_file(dev, &dev_attr_cpld_refresh);
	device_create_file(dev, &dev_attr_gpio_our);
	device_create_file(dev, &dev_attr_gpio_their);
	arb->bus_locked = false;
#endif //CONFIG_DRAGON_CHASSIS

	return ret;
}

static int i2c_arbitrator_remove(struct platform_device *pdev)
{
	struct i2c_mux_core *muxc = platform_get_drvdata(pdev);

#if CONFIG_DRAGON_CHASSIS
	device_remove_file(&pdev->dev, &dev_attr_arb_state);
	device_remove_file(&pdev->dev, &dev_attr_cpld_refresh);
#endif //CONFIG_DRAGON_CHASSIS

	i2c_mux_del_adapters(muxc);
	i2c_put_adapter(muxc->parent);
	return 0;
}

static const struct of_device_id i2c_arbitrator_of_match[] = {
	{ .compatible = "i2c-arb-gpio-challenge", },
	{},
};
MODULE_DEVICE_TABLE(of, i2c_arbitrator_of_match);

static struct platform_driver i2c_arbitrator_driver = {
	.probe	= i2c_arbitrator_probe,
	.remove	= i2c_arbitrator_remove,
	.driver	= {
		.name	= "i2c-arb-gpio-challenge",
		.of_match_table = i2c_arbitrator_of_match,
	},
};

module_platform_driver(i2c_arbitrator_driver);

MODULE_DESCRIPTION("GPIO-based I2C Arbitration");
MODULE_AUTHOR("Doug Anderson <dianders@chromium.org>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:i2c-arb-gpio-challenge");
