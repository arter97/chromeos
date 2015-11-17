/*
 * Copyright (C) 2015 Luis R. Rodriguez
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/firmware.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

static struct platform_device *pdev;

struct test_stuff {
	char name[1024];
};

static void test_mod_cb(const struct firmware *fw, void *context)
{
	if (!fw) {
		pr_info("No firmware found\n");
	}
	pr_info("firmware found\n");
	release_firmware(fw);
}

static int __init test_init(void)
{
	struct test_stuff *stuff;
	const char *name = "test_module_stuff.bin";
	int ret;

	pdev = platform_device_register_simple("fake-dev", 0, NULL, 0);
	if (IS_ERR(pdev))
		return PTR_ERR(pdev);

	stuff = kzalloc(sizeof(struct test_stuff), GFP_KERNEL);
	if (!stuff) {
		ret = -ENOMEM;
		goto out;
	}
	memcpy(stuff->name, name, strlen(name));

	/*
	 * firmware worker consumer here may access stuff->name later just
	 * too late if the driver is sloppy. For instance:
	 * */
	ret = request_firmware_nowait(THIS_MODULE, 1, stuff->name, &pdev->dev,
				      GFP_KERNEL, pdev, test_mod_cb);
	if (ret)
		goto out;
	kfree(stuff);

	return 0;
out:
	dev_set_uevent_suppress(&pdev->dev, true);
	platform_device_unregister(pdev);
	kfree(stuff);
	return ret;
}

static void __exit test_exit(void)
{
	dev_set_uevent_suppress(&pdev->dev, true);
	platform_device_unregister(pdev);
}

module_init(test_init)
module_exit(test_exit)
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Luis R. Rodriguez");
