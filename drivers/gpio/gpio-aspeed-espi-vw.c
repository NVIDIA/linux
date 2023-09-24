// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2023, Intel Corporation.

#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#include <linux/mfd/syscon.h>
#include <linux/soc/aspeed/aspeed-espi.h>

#define ASPEED_ESPI_CTRL 0x000
#define ASPEED_ESPI_CTRL_VW_SW_RDY BIT(3)

#define ASPEED_ESPI_INT_STS 0x008
#define ASPEED_ESPI_INT_STS_HW_RESET BIT(31)
#define ASPEED_ESPI_INT_STS_VW_SYSEVT1 BIT(22)
#define ASPEED_ESPI_INT_STS_VW_GPIOEVT BIT(9)
#define ASPEED_ESPI_INT_STS_VW_SYSEVT BIT(8)
#define ASPEED_ESPI_INT_STS_VW_MASK				\
	(ASPEED_ESPI_INT_STS_VW_SYSEVT1 |			\
	 ASPEED_ESPI_INT_STS_VW_GPIOEVT |			\
	 ASPEED_ESPI_INT_STS_VW_SYSEVT)

#define ASPEED_ESPI_INT_EN 0x00c
#define ASPEED_ESPI_INT_EN_VW_SYSEVT1 BIT(22)
#define ASPEED_ESPI_INT_EN_VW_GPIOEVT BIT(9)
#define ASPEED_ESPI_INT_EN_VW_SYSEVT BIT(8)
#define ASPEED_ESPI_INT_EN_VW_MASK				\
	(ASPEED_ESPI_INT_EN_VW_SYSEVT1 |			\
	 ASPEED_ESPI_INT_EN_VW_GPIOEVT |			\
	 ASPEED_ESPI_INT_EN_VW_SYSEVT)

#define ASPEED_ESPI_VW_GPIO_VAL 0x09c
#define ASPEED_ESPI_VW_GPIO_DIR 0x0c0

#define ASPEED_ESPI_VW_GPIO_RESET_SELECTION 0x0c8
#define ASPEED_ESPI_VW_RESET_BY_ESPI 0
#define ASPEED_ESPI_VW_RESET_BY_PLTRST 1

static u32 cached_reg_val;
struct aspeed_espi_gpio {
	struct device *dev;
	struct gpio_chip chip;
	struct regmap *map;
	u32 dir_mask;
	/* Lock to protect GPIO val register access */
	spinlock_t lock;
};

static void aspeed_espi_vw_gpio_enable(struct regmap *map, u32 dir_mask)
{
	regmap_update_bits(map, ASPEED_ESPI_INT_EN, ASPEED_ESPI_INT_EN_VW_MASK,
			   ASPEED_ESPI_INT_EN_VW_MASK);

	regmap_update_bits(map, ASPEED_ESPI_CTRL, ASPEED_ESPI_CTRL_VW_SW_RDY,
			   ASPEED_ESPI_CTRL_VW_SW_RDY);
	regmap_write(map, ASPEED_ESPI_VW_GPIO_DIR, ~dir_mask);
	regmap_write(map, ASPEED_ESPI_VW_GPIO_RESET_SELECTION,
		     ASPEED_ESPI_VW_RESET_BY_ESPI);
}

static void aspeed_espi_vw_gpio_disable(struct regmap *map)
{
	regmap_update_bits(map, ASPEED_ESPI_CTRL, ASPEED_ESPI_CTRL_VW_SW_RDY, 0);
	regmap_update_bits(map, ASPEED_ESPI_INT_EN, ASPEED_ESPI_INT_EN_VW_MASK, 0);
}

static void set_nth_bit(u32 *n, uint8_t offset, u32 val)
{
	if (val == 0)
		*n = *n & ~(1ul << offset);
	else
		*n = *n | (1ul << offset);
}

static int vgpio_get_value(struct gpio_chip *gc, unsigned int offset)
{
	struct aspeed_espi_gpio *gpio = gpiochip_get_data(gc);
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&gpio->lock, flags);
	ret = regmap_test_bits(gpio->map, ASPEED_ESPI_VW_GPIO_VAL, BIT(offset));
	spin_unlock_irqrestore(&gpio->lock, flags);
	return ret;
}

static void vgpio_set_value(struct gpio_chip *gc, unsigned int offset, int val)
{
	struct aspeed_espi_gpio *gpio = gpiochip_get_data(gc);
	unsigned long flags;
	int ret;

	val = val ? BIT(offset) : 0;
	spin_lock_irqsave(&gpio->lock, flags);
	ret = regmap_update_bits(gpio->map, ASPEED_ESPI_VW_GPIO_VAL, BIT(offset), val);
	if (!ret)
		set_nth_bit(&cached_reg_val, offset, val);
	spin_unlock_irqrestore(&gpio->lock, flags);
}

static int vgpio_get_direction(struct gpio_chip *gc, unsigned int offset)
{
	struct aspeed_espi_gpio *gpio = gpiochip_get_data(gc);

	return (gpio->dir_mask & BIT(offset)) ? GPIO_LINE_DIRECTION_OUT : GPIO_LINE_DIRECTION_IN;
}

static int vgpio_direction_input(struct gpio_chip *gc, unsigned int offset)
{
	struct aspeed_espi_gpio *gpio = gpiochip_get_data(gc);

	return (gpio->dir_mask & BIT(offset)) ? -EOPNOTSUPP : 0;
}

static int vgpio_direction_output(struct gpio_chip *gc, unsigned int offset, int val)
{
	struct aspeed_espi_gpio *gpio = gpiochip_get_data(gc);

	if (!(gpio->dir_mask & BIT(offset))) {
		dev_dbg(gpio->dev, "VW GPIO: Offset %d is not output\n", offset);
		return -EOPNOTSUPP;
	}

	vgpio_set_value(gc, offset, val);
	return 0;
}

static int aspeed_espi_vw_gpio_init(struct device *dev, struct aspeed_espi_gpio *gpio)
{
	const char **gpio_names, **out_strs;
	int names_count, ret, i, j = 0;
	u32 names_mask, dir_mask;
	unsigned int gpio_count;

	ret = of_property_read_u32(dev->of_node, "gpio-names-mask", &names_mask);
	if (ret) {
		dev_err(dev, "Unable to read gpio names mask: %d\n", ret);
		return ret;
	}

	ret = of_property_read_u32(dev->of_node, "gpio-dir-mask", &dir_mask);
	if (ret) {
		dev_err(dev, "Unable to read gpio direction mask: %d\n", ret);
		return ret;
	}
	gpio->dir_mask = dir_mask;

	ret = of_property_read_u32(dev->of_node, "gpio-count", &gpio_count);
	if (ret) {
		dev_err(dev, "Unable to read gpio names count: %d %d\n", ret, gpio_count);
		return ret;
	}

	names_count = of_property_read_string_array(dev->of_node, "gpio-names", NULL, 0);
	if (names_count <= 0) {
		dev_err(dev, "Unable to read gpio names count\n");
		return -EINVAL;
	}

	out_strs = devm_kzalloc(dev, sizeof(char *) * names_count, GFP_KERNEL);
	if (!out_strs)
		return -ENOMEM;

	ret = of_property_read_string_array(dev->of_node, "gpio-names", out_strs, names_count);
	if (ret < 0) {
		dev_err(dev, "Unable to read gpio names array\n");
		return ret;
	}

	gpio_names = devm_kzalloc(dev, sizeof(char *) * gpio_count, GFP_KERNEL);
	if (!gpio_names)
		return -ENOMEM;

	for (i = 0; i < gpio_count; i++) {
		if (names_mask & BIT(i)) {
			if (j >= names_count) {
				dev_err(dev, "Not enough names in device tree to use in VW GPIO\n");
				return -EINVAL;
			}
			gpio_names[i] = out_strs[j++];
		} else {
			gpio_names[i] = "VW_GPIO_UNUSED";
		}
	}

	aspeed_espi_vw_gpio_enable(gpio->map, dir_mask);

	spin_lock_init(&gpio->lock);

	gpio->chip.label = "ESPI-VW-GPIO";
	gpio->chip.base = -1;
	gpio->chip.parent = dev;
	gpio->chip.owner = THIS_MODULE;
	gpio->chip.ngpio = gpio_count;
	gpio->chip.can_sleep = 0;
	gpio->chip.get = vgpio_get_value;
	gpio->chip.set = vgpio_set_value;
	gpio->chip.direction_output = vgpio_direction_output;
	gpio->chip.direction_input = vgpio_direction_input;
	gpio->chip.get_direction = vgpio_get_direction;
	gpio->chip.names = gpio_names;

	ret = devm_gpiochip_add_data(dev, &gpio->chip, gpio);
	if (ret)
		dev_err(dev, "Error adding ESPI VGPIO\n");

	return ret;
}

static void aspeed_espi_vw_irq(int irq, void *arg)
{
	struct aspeed_espi_gpio *gpio = arg;
	unsigned long flags;
	u32 sts;

	if (regmap_read(gpio->map, ASPEED_ESPI_INT_STS, &sts)) {
		dev_dbg(gpio->dev, "Error reading int status\n");
		return;
	}

	if (sts & ASPEED_ESPI_INT_STS_HW_RESET) {
		dev_dbg(gpio->dev, "Resetting VGPIO value [%08X]\n", cached_reg_val);
		aspeed_espi_vw_gpio_enable(gpio->map, gpio->dir_mask);

		spin_lock_irqsave(&gpio->lock, flags);
		regmap_update_bits(gpio->map, ASPEED_ESPI_VW_GPIO_VAL, gpio->dir_mask,
				   cached_reg_val);
		spin_unlock_irqrestore(&gpio->lock, flags);
	}
	/* Clearing of status register will be done from parent driver*/
}
static int aspeed_espi_gpio_probe(struct platform_device *pdev)
{
	struct aspeed_espi_gpio *gpio;
	int ret;

	gpio = devm_kzalloc(&pdev->dev, sizeof(*gpio), GFP_KERNEL);
	if (!gpio)
		return -ENOMEM;

	gpio->map = device_node_to_regmap(pdev->dev.parent->of_node);
	if (!gpio->map) {
		dev_err(&pdev->dev, "Couldn't get regmap\n");
		return -ENODEV;
	}

	dev_set_drvdata(&pdev->dev, gpio);
	ret = aspeed_espi_vw_gpio_init(&pdev->dev, gpio);
	if (ret) {
		dev_err(&pdev->dev, "Unable to register espi gpio\n");
		return ret;
	}
	gpio->dev = &pdev->dev;

	aspeed_espi_register_gpio(pdev->dev.parent, aspeed_espi_vw_irq, gpio);

	return ret;
}

static int aspeed_espi_gpio_remove(struct platform_device *pdev)
{
	struct aspeed_espi_gpio *gpio = dev_get_drvdata(&pdev->dev);

	aspeed_espi_vw_gpio_disable(gpio->map);
	return 0;
}

static const struct of_device_id of_espi_match_table[] = {
	{ .compatible = "aspeed,ast2600-espi-vw-gpio" },
	{}
};

MODULE_DEVICE_TABLE(of, of_espi_match_table);

static struct platform_driver aspeed_espi_gpio_driver = {
	.driver = {
		.name = KBUILD_MODNAME,
		.of_match_table = of_match_ptr(of_espi_match_table),
	},
	.probe = aspeed_espi_gpio_probe,
	.remove = aspeed_espi_gpio_remove,
};
module_platform_driver(aspeed_espi_gpio_driver);

MODULE_AUTHOR("Nidhin M S <nidhin.ms@intel.com>");
MODULE_DESCRIPTION("Aspeed eSPI virtual wire GPIO driver");
MODULE_LICENSE("GPL");
