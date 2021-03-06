/*
 * linux/drivers/video/backlight/pwm_bl.c
 *
 * simple PWM based backlight control, board code has to setup
 * 1) pin configuration so PWM waveforms can output
 * 2) platform_data being correctly configured
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/fb.h>
#include <linux/backlight.h>
#include <linux/err.h>
#include <linux/pwm.h>
#include <linux/pwm_backlight.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>

struct pwm_bl_alt_brightness_table {
	const char		*identifier;
	unsigned int		*levels;
};

struct pwm_bl_data {
	struct pwm_device	*pwm;
	struct device		*dev;
	unsigned int		period;
	unsigned int		lth_brightness;
	unsigned int		*levels;
	struct pwm_bl_alt_brightness_table *alt_brightness_tables;
	unsigned int		num_alt_brightness_tables;
	int			enable_gpio;
	bool			gpio_invert;
	int			(*notify)(struct device *,
					  int brightness);
	void			(*notify_after)(struct device *,
					int brightness);
	int			(*check_fb)(struct device *, struct fb_info *);
	void			(*exit)(struct device *);
};

static void pwm_backlight_gpio_set(struct pwm_bl_data *pb, bool enable)
{
	int val;

	if (gpio_is_valid(pb->enable_gpio)) {
		if (enable)
			val = !pb->gpio_invert;
		else
			val = pb->gpio_invert;
		gpio_direction_output(pb->enable_gpio, val);
	}
}

static int pwm_backlight_update_status(struct backlight_device *bl)
{
	struct pwm_bl_data *pb = bl_get_data(bl);
	int brightness = bl->props.brightness;
	int max = bl->props.max_brightness;

	if (bl->props.power != FB_BLANK_UNBLANK ||
	    bl->props.fb_blank != FB_BLANK_UNBLANK ||
	    bl->props.state & BL_CORE_FBBLANK)
		brightness = 0;

	if (pb->notify)
		brightness = pb->notify(pb->dev, brightness);

	if (brightness == 0) {
		pwm_config(pb->pwm, 0, pb->period);
		pwm_disable(pb->pwm);
		pwm_backlight_gpio_set(pb, false);
	} else {
		int duty_cycle;

		if (pb->levels) {
			duty_cycle = pb->levels[brightness];
			max = pb->levels[max];
		} else {
			duty_cycle = brightness;
		}

		duty_cycle = pb->lth_brightness +
		     (duty_cycle * (pb->period - pb->lth_brightness) / max);
		pwm_config(pb->pwm, duty_cycle, pb->period);
		pwm_enable(pb->pwm);
		pwm_backlight_gpio_set(pb, true);
	}

	if (pb->notify_after)
		pb->notify_after(pb->dev, brightness);

	return 0;
}

static int pwm_backlight_get_brightness(struct backlight_device *bl)
{
	return bl->props.brightness;
}

static int pwm_backlight_check_fb(struct backlight_device *bl,
				  struct fb_info *info)
{
	struct pwm_bl_data *pb = bl_get_data(bl);

	return !pb->check_fb || pb->check_fb(pb->dev, info);
}

static int pwm_backlight_choose(struct backlight_device *bl,
				char *identifier)
{
	struct pwm_bl_data *pb = bl_get_data(bl);
	struct pwm_bl_alt_brightness_table *table = NULL;
	int table_index;

	if (!pb->alt_brightness_tables)
		return -EINVAL;

	for (table_index = 0;
	     table_index < pb->num_alt_brightness_tables;
	     table_index++)
		if (strcmp(pb->alt_brightness_tables[table_index].identifier,
			   identifier) == 0) {
			table = &pb->alt_brightness_tables[table_index];
			break;
		}

	if (!table)
		return -EINVAL;

	memcpy(pb->levels, table->levels,
	       sizeof(*table->levels) * (bl->props.max_brightness + 1));

	pwm_backlight_update_status(bl);

	dev_info(pb->dev, "Chose %s\n", identifier);

	return 0;
}

static const struct backlight_ops pwm_backlight_ops = {
	.update_status	= pwm_backlight_update_status,
	.get_brightness	= pwm_backlight_get_brightness,
	.check_fb	= pwm_backlight_check_fb,
	.choose		= pwm_backlight_choose,
};

#ifdef CONFIG_OF
int pwm_backlight_parse_dt(struct device *dev,
				  struct platform_pwm_backlight_data *data)
{
	struct device_node *node = dev->of_node;
	struct property *prop;
	int length;
	u32 value;
	int ret;
	enum of_gpio_flags flags;

	if (!node)
		return -ENODEV;

	memset(data, 0, sizeof(*data));

	/* determine the number of brightness levels */
	prop = of_find_property(node, "brightness-levels", &length);
	if (!prop)
		return -EINVAL;

	data->max_brightness = length / sizeof(u32);

	/* read brightness levels from DT property */
	if (data->max_brightness > 0) {
		size_t size = sizeof(*data->levels) * data->max_brightness;

		data->levels = devm_kzalloc(dev, size, GFP_KERNEL);
		if (!data->levels)
			return -ENOMEM;

		ret = of_property_read_u32_array(node, "brightness-levels",
						 data->levels,
						 data->max_brightness);
		if (ret < 0)
			return ret;

		ret = of_property_read_u32(node, "default-brightness-level",
					   &value);
		if (ret < 0)
			return ret;

		data->dft_brightness = value;
		data->max_brightness--;
	}

	data->enable_gpio = of_get_named_gpio_flags(node, "enable-gpio", 0,
							&flags);
	if (gpio_is_valid(data->enable_gpio) && (flags & OF_GPIO_ACTIVE_LOW))
		data->gpio_invert = true;

	return 0;
}

static void pwm_backlight_alt_brightness_table_parse_dt(struct device *dev,
							struct pwm_bl_data *pb)
{
	struct device_node *node = dev->of_node;
	struct property *prop;
	struct device_node *iter;
	int default_length;
	int table_index;

	prop = of_find_property(node, "brightness-levels", &default_length);
	if (!prop)
		return;

	/* Determine the number of alternate brightness level tables */
	for_each_child_of_node(node, iter)
		if (of_device_is_compatible(iter,
					"pwm-backlight-alt-brightness-levels"))
			pb->num_alt_brightness_tables++;

	if (pb->num_alt_brightness_tables == 0)
		return;

	pb->alt_brightness_tables = devm_kzalloc(dev,
		sizeof(struct pwm_bl_alt_brightness_table) *
		pb->num_alt_brightness_tables,
		GFP_KERNEL);
	if (!pb->alt_brightness_tables)
		return;

	/* Read in each alternate brightness level table */
	table_index = 0;
	for_each_child_of_node(node, iter) {
		struct pwm_bl_alt_brightness_table *table =
			&pb->alt_brightness_tables[table_index];
		struct property *prop;
		int length;
		int max_brightness;
		size_t size;
		int ret;

		/*
		 * Read in the identifier that will be used to select the
		 * table.
		 */
		ret = of_property_read_string(iter, "identifier",
					      &table->identifier);
		if (ret < 0)
			goto fail;

		/*
		 * Determine the number of brightness levels.  All of the
		 * alternate brightness tables must be of the same length
		 * as the default one.
		 */
		prop = of_find_property(iter, "brightness-levels", &length);
		max_brightness = length / sizeof(u32);
		size = sizeof(*table->levels) * max_brightness;

		if (size != default_length)
			goto fail;

		table->levels = devm_kzalloc(dev, size, GFP_KERNEL);
		if (!table->levels)
			goto fail;

		/* Finally, read in the alternate table. */
		ret = of_property_read_u32_array(iter, "brightness-levels",
						 table->levels,
						 max_brightness);
		if (ret < 0)
			goto fail;

		table_index++;
	}

	dev_notice(dev, "Read in %d alternate brightness tables\n",
		   pb->num_alt_brightness_tables);

	return;

fail:
	dev_notice(dev, "Failed reading alternate brightness tables\n");

	/*
	 * If we fail for whatever reason, roll back the alternate brightness
	 * level tables' allocations.
	 */
	for (table_index = 0;
	     table_index < pb->num_alt_brightness_tables;
	     table_index++) {
		struct pwm_bl_alt_brightness_table *table =
			&pb->alt_brightness_tables[table_index];

		/*
		 * table->identifier is not actually allocated by us, so don't
		 * worry about it here.
		 */

		if (table->levels)
			devm_kfree(dev, table->levels);
	}

	devm_kfree(dev, pb->alt_brightness_tables);
	pb->alt_brightness_tables = NULL;
}

static struct of_device_id pwm_backlight_of_match[] = {
	{ .compatible = "pwm-backlight" },
	{ }
};

MODULE_DEVICE_TABLE(of, pwm_backlight_of_match);
#else
int pwm_backlight_parse_dt(struct device *dev,
				  struct platform_pwm_backlight_data *data)
{
	return -ENODEV;
}

static void pwm_backlight_alt_brightness_table_parse_dt(struct device *dev,
							struct pwm_bl_data *pb)
{
}

#endif

static int pwm_backlight_probe(struct platform_device *pdev)
{
	struct platform_pwm_backlight_data *data = pdev->dev.platform_data;
	struct platform_pwm_backlight_data defdata;
	struct backlight_properties props;
	struct backlight_device *bl;
	struct pwm_bl_data *pb;
	unsigned int max;
	int ret;

	if (!data) {
		ret = pwm_backlight_parse_dt(&pdev->dev, &defdata);
		if (ret < 0) {
			dev_err(&pdev->dev, "failed to find platform data\n");
			return ret;
		}

		data = &defdata;
	}

	if (data->init) {
		ret = data->init(&pdev->dev);
		if (ret < 0)
			return ret;
	}

	pb = devm_kzalloc(&pdev->dev, sizeof(*pb), GFP_KERNEL);
	if (!pb) {
		dev_err(&pdev->dev, "no memory for state\n");
		ret = -ENOMEM;
		goto err_alloc;
	}

	if (data->levels) {
		max = data->levels[data->max_brightness];
		pb->levels = data->levels;
	} else
		max = data->max_brightness;

	if (gpio_is_valid(data->enable_gpio)) {
		ret = devm_gpio_request(&pdev->dev, data->enable_gpio,
					"pwm-bl-en");
		if (ret) {
			dev_err(&pdev->dev, "unable to request enable GPIO\n");
			goto err_alloc;
		}
	}
	pb->enable_gpio = data->enable_gpio;
	pb->gpio_invert = data->gpio_invert;

	pb->notify = data->notify;
	pb->notify_after = data->notify_after;
	pb->check_fb = data->check_fb;
	pb->exit = data->exit;
	pb->dev = &pdev->dev;

	pb->pwm = devm_pwm_get(&pdev->dev, NULL);
	if (IS_ERR(pb->pwm)) {
		dev_err(&pdev->dev, "unable to request PWM, trying legacy API\n");

		pb->pwm = pwm_request(data->pwm_id, "pwm-backlight");
		if (IS_ERR(pb->pwm)) {
			dev_err(&pdev->dev, "unable to request legacy PWM\n");
			ret = PTR_ERR(pb->pwm);
			goto err_alloc;
		}
	}

	dev_dbg(&pdev->dev, "got pwm for backlight\n");

	/*
	 * The DT case will set the pwm_period_ns field to 0 and store the
	 * period, parsed from the DT, in the PWM device. For the non-DT case,
	 * set the period from platform data.
	 */
	if (data->pwm_period_ns > 0)
		pwm_set_period(pb->pwm, data->pwm_period_ns);

	pb->period = pwm_get_period(pb->pwm);
	pb->lth_brightness = data->lth_brightness * (pb->period / max);

	memset(&props, 0, sizeof(struct backlight_properties));
	props.type = BACKLIGHT_RAW;
	props.max_brightness = data->max_brightness;
	bl = backlight_device_register(dev_name(&pdev->dev), &pdev->dev, pb,
				       &pwm_backlight_ops, &props);
	if (IS_ERR(bl)) {
		dev_err(&pdev->dev, "failed to register backlight\n");
		ret = PTR_ERR(bl);
		goto err_alloc;
	}

	if (data->dft_brightness > data->max_brightness) {
		dev_warn(&pdev->dev,
			 "invalid default brightness level: %u, using %u\n",
			 data->dft_brightness, data->max_brightness);
		data->dft_brightness = data->max_brightness;
	}

	pwm_backlight_alt_brightness_table_parse_dt(&pdev->dev, pb);

	bl->props.brightness = data->dft_brightness;
	backlight_update_status(bl);

	platform_set_drvdata(pdev, bl);
	return 0;

err_alloc:
	if (data->exit)
		data->exit(&pdev->dev);
	return ret;
}

static int pwm_backlight_remove(struct platform_device *pdev)
{
	struct backlight_device *bl = platform_get_drvdata(pdev);
	struct pwm_bl_data *pb = bl_get_data(bl);

	backlight_device_unregister(bl);
	pwm_config(pb->pwm, 0, pb->period);
	pwm_disable(pb->pwm);
	pwm_backlight_gpio_set(pb, false);
	if (pb->exit)
		pb->exit(&pdev->dev);
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int pwm_backlight_suspend(struct device *dev)
{
	struct backlight_device *bl = dev_get_drvdata(dev);
	struct pwm_bl_data *pb = bl_get_data(bl);

	if (pb->notify)
		pb->notify(pb->dev, 0);
	pwm_config(pb->pwm, 0, pb->period);
	pwm_disable(pb->pwm);
	pwm_backlight_gpio_set(pb, false);
	if (pb->notify_after)
		pb->notify_after(pb->dev, 0);
	return 0;
}

static int pwm_backlight_resume(struct device *dev)
{
	struct backlight_device *bl = dev_get_drvdata(dev);

	backlight_update_status(bl);
	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(pwm_backlight_pm_ops, pwm_backlight_suspend,
			 pwm_backlight_resume);

static struct platform_driver pwm_backlight_driver = {
	.driver		= {
		.name		= "pwm-backlight",
		.owner		= THIS_MODULE,
		.pm		= &pwm_backlight_pm_ops,
		.of_match_table	= of_match_ptr(pwm_backlight_of_match),
	},
	.probe		= pwm_backlight_probe,
	.remove		= pwm_backlight_remove,
};

module_platform_driver(pwm_backlight_driver);

MODULE_DESCRIPTION("PWM based Backlight Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:pwm-backlight");

