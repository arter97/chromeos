#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>

#include <media/soc_camera.h>

#include <dt-bindings/gpio/tegra-gpio.h>

/* RCAM is rear-facing IMX219 with focuser.  RCAM_MCLK is CAM_MCLK */
/* FCAM is front-facing IMX208.  FCAM_MCLK is PBB0 */
#define RCAM_RST_L	TEGRA_GPIO(BB, 3)
#define FCAM_RST_L	TEGRA_GPIO(BB, 4)

/* IMX219 */
static int imx219_subdev_power(struct device *dev, int on)
{
	int retval = 0;

	if (on) {
		retval = gpio_request(RCAM_RST_L, "rcam-rst-l");
		if (retval < 0) {
			dev_err(dev, "failed requesting gpio rcam-rst-l\n");
			return retval;
		}

		gpio_direction_output(RCAM_RST_L, 0);
		msleep(10);
		gpio_set_value(RCAM_RST_L, 1);
		msleep(100);
	} else {
		gpio_set_value(RCAM_RST_L, 0);

		gpio_free(RCAM_RST_L);
	}

	return retval;
}

static struct regulator_bulk_data imx219_subdev_regulators[] = {
	{ .supply = "avdd-dsi-csi" },
	{ .supply = "vdd-1v8" }, // SMPS8 -> PP1800 -> PP1800_CAM -> PP1800_RCAM
	{ .supply = "en-vddcam-1v8" }, // TPS65913 GPIO 4 -> EN_VDDCAM_1V8
	{ .supply = "vdd-camera" },  // LDO3 -> PP1200_RCAM
	{ .supply = "avdd-rear-camera" }, // LDO4 -> PP2800L_RCAM -> PP2800L_RCAMA
};

static struct soc_camera_subdev_desc imx219_subdev_desc = {
	.power = imx219_subdev_power,
	.sd_pdata = {
		.regulators = imx219_subdev_regulators,
		.num_regulators = ARRAY_SIZE(imx219_subdev_regulators),
	},
};

/* IMX208 */
static int imx208_subdev_power(struct device *dev, int on)
{
	int retval = 0;

	if (on) {
		retval = gpio_request(FCAM_RST_L, "fcam-rst-l");
		if (retval < 0) {
			dev_err(dev, "failed requesting gpio fcam-rst-l\n");
			return retval;
		}

		gpio_direction_output(FCAM_RST_L, 0);
		msleep(10);
		gpio_set_value(FCAM_RST_L, 1);
		msleep(100);
	} else {
		gpio_set_value(FCAM_RST_L, 0);

		gpio_free(FCAM_RST_L);
	}

	return retval;
}

static struct regulator_bulk_data imx208_subdev_regulators[] = {
	{ .supply = "avdd-dsi-csi" },
	{ .supply = "vdd-1v8" }, // SMPS8 -> PP1800 -> PP1800_CAM -> PP1800_FCAM
	{ .supply = "en-vddcam-1v8" }, // TPS65913 GPIO 4 -> EN_VDDCAM_1V8
	{ .supply = "vdd-camera" }, // LDO3 -> PP1200_RCAM
	{ .supply = "avdd-rear-camera" }, // LDO4 -> PP2800L_RCAM -> PP2800L_FCAMA
};

static struct soc_camera_subdev_desc imx208_subdev_desc = {
	.power = imx208_subdev_power,
	.sd_pdata = {
		.regulators = imx208_subdev_regulators,
		.num_regulators = ARRAY_SIZE(imx208_subdev_regulators),
	},
};

static struct regulator_bulk_data ad5823_subdev_regulators[] = {
	{ .supply = "avdd-front-camera" }, // LDO7 -> PP2800L_FCAM -> PP2800_VCM
	{ .supply = "af-pwdn" }, // Tegra GPIO PBB7
};

static struct soc_camera_subdev_desc ad5823_subdev_desc = {
	.sd_pdata = {
		.regulators = ad5823_subdev_regulators,
		.num_regulators = ARRAY_SIZE(ad5823_subdev_regulators),
	},
};

/* Bus notifier for hooking platform data to the sensors */
static int tegra_media_notifier_call(struct notifier_block *nb,
				     unsigned long event, void *__dev)
{
	struct device *dev = __dev;

	if (event != BUS_NOTIFY_ADD_DEVICE)
		return NOTIFY_DONE;

	if (of_device_is_compatible(dev->of_node, "sony,imx219"))
		dev->platform_data = &imx219_subdev_desc;
	else if (of_device_is_compatible(dev->of_node, "sony,imx208"))
		dev->platform_data = &imx208_subdev_desc;
	else if (of_device_is_compatible(dev->of_node, "ad,ad5823"))
		dev->platform_data = &ad5823_subdev_desc;

	return NOTIFY_OK;
}

static struct notifier_block tegra_media_nb = {
	.notifier_call = tegra_media_notifier_call,
};

static int __init tegra_media_arch_init(void)
{
	bus_register_notifier(&i2c_bus_type, &tegra_media_nb);

	return 0;
}
arch_initcall(tegra_media_arch_init);
