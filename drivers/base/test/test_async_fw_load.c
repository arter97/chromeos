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
 *
 * Test if asynchronous firmware request mechanism has a
 * use-after-free/data-race on the firmware name field. Looks for an aribtrary
 * "firmware" (just fill it with a bit of junk) at
 * /lib/firmware/test_module_stuff.bin". A return code of '0' from the module
 * init function means the firmware load succeeded. A non-zero return code
 * likely means there request_firmware_nowait() is still racy.
 *
 * Note that a successful request doesn't mean there is no bug; we may have
 * just "won" the race.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/firmware.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/completion.h>

static struct platform_device *pdev;
static DECLARE_COMPLETION(fw_done);
static int fw_found;

struct test_stuff {
	char name[1024];
};

static void test_mod_cb(const struct firmware *fw, void *context)
{
	if (!fw) {
		pr_info("No firmware found\n");
	} else {
		pr_info("firmware found\n");
		fw_found = 1;
	}
	release_firmware(fw);
	complete(&fw_done);
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
	stuff = NULL; /* don't try to kfree() it again on error */

	/*
	 * Even though we're testing asnc firmware loads, we still want to
	 * report whether the request succeeded or not
	 */
	if (!wait_for_completion_timeout(&fw_done, msecs_to_jiffies(10000))) {
		pr_info("timed out waiting for firmware request\n");
		ret = -ETIMEDOUT;
		goto out;
	}

	if (!fw_found) {
		ret = -ENODEV;
		goto out;
	}

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
