/*
 * NVIDIA Tegra xHCI host controller driver
 *
 * Copyright (C) 2014 NVIDIA Corporation
 * Copyright (C) 2014 Google, Inc.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/firmware.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/mailbox_client.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/reset.h>
#include <linux/slab.h>
#include <linux/workqueue.h>

#include <soc/tegra/pmc.h>
#include <soc/tegra/tegra_emc.h>
#include <soc/tegra/xusb.h>

#include "xhci.h"

#define TEGRA_XHCI_SS_CLK_HIGH_SPEED 120000000
#define TEGRA_XHCI_SS_CLK_LOW_SPEED 12000000

/* FPCI CFG registers */
#define XUSB_CFG_1				0x004
#define  XUSB_IO_SPACE_EN			BIT(0)
#define  XUSB_MEM_SPACE_EN			BIT(1)
#define  XUSB_BUS_MASTER_EN			BIT(2)
#define XUSB_CFG_4				0x010
#define  XUSB_BASE_ADDR_SHIFT			15
#define  XUSB_BASE_ADDR_MASK			0x1ffff
#define XUSB_CFG_ARU_C11_CSBRANGE		0x41c
#define XUSB_CFG_CSB_BASE_ADDR			0x800

/* IPFS registers */
#define IPFS_XUSB_HOST_CONFIGURATION_0		0x180
#define  IPFS_EN_FPCI				BIT(0)
#define IPFS_XUSB_HOST_INTR_MASK_0		0x188
#define  IPFS_IP_INT_MASK			BIT(16)
#define IPFS_XUSB_HOST_CLKGATE_HYSTERESIS_0	0x1bc

#define CSB_PAGE_SELECT_MASK			0x7fffff
#define CSB_PAGE_SELECT_SHIFT			9
#define CSB_PAGE_OFFSET_MASK			0x1ff
#define CSB_PAGE_SELECT(addr)	((addr) >> (CSB_PAGE_SELECT_SHIFT) &	\
				 CSB_PAGE_SELECT_MASK)
#define CSB_PAGE_OFFSET(addr)	((addr) & CSB_PAGE_OFFSET_MASK)

/* Falcon CSB registers */
#define XUSB_FALC_CPUCTL			0x100
#define  CPUCTL_STARTCPU			BIT(1)
#define  CPUCTL_STATE_HALTED			BIT(4)
#define  CPUCTL_STATE_STOPPED			BIT(5)
#define XUSB_FALC_BOOTVEC			0x104
#define XUSB_FALC_DMACTL			0x10c
#define XUSB_FALC_IMFILLRNG1			0x154
#define  IMFILLRNG1_TAG_MASK			0xffff
#define  IMFILLRNG1_TAG_LO_SHIFT		0
#define  IMFILLRNG1_TAG_HI_SHIFT		16
#define XUSB_FALC_IMFILLCTL			0x158

/* MP CSB registers */
#define XUSB_CSB_MP_ILOAD_ATTR			0x101a00
#define XUSB_CSB_MP_ILOAD_BASE_LO		0x101a04
#define XUSB_CSB_MP_ILOAD_BASE_HI		0x101a08
#define XUSB_CSB_MP_L2IMEMOP_SIZE		0x101a10
#define  L2IMEMOP_SIZE_SRC_OFFSET_SHIFT		8
#define  L2IMEMOP_SIZE_SRC_OFFSET_MASK		0x3ff
#define  L2IMEMOP_SIZE_SRC_COUNT_SHIFT		24
#define  L2IMEMOP_SIZE_SRC_COUNT_MASK		0xff
#define XUSB_CSB_MP_L2IMEMOP_TRIG		0x101a14
#define  L2IMEMOP_ACTION_SHIFT			24
#define  L2IMEMOP_INVALIDATE_ALL		(0x40 << L2IMEMOP_ACTION_SHIFT)
#define  L2IMEMOP_LOAD_LOCKED_RESULT		(0x11 << L2IMEMOP_ACTION_SHIFT)
#define XUSB_CSB_MP_APMAP			0x10181c
#define  APMAP_BOOTPATH				BIT(31)

#define IMEM_BLOCK_SIZE				256

struct tegra_xhci_fw_cfgtbl {
	u32 boot_loadaddr_in_imem;
	u32 boot_codedfi_offset;
	u32 boot_codetag;
	u32 boot_codesize;
	u32 phys_memaddr;
	u16 reqphys_memsize;
	u16 alloc_phys_memsize;
	u32 rodata_img_offset;
	u32 rodata_section_start;
	u32 rodata_section_end;
	u32 main_fnaddr;
	u32 fwimg_cksum;
	u32 fwimg_created_time;
	u32 imem_resident_start;
	u32 imem_resident_end;
	u32 idirect_start;
	u32 idirect_end;
	u32 l2_imem_start;
	u32 l2_imem_end;
	u32 version_id;
	u8 init_ddirect;
	u8 reserved[3];
	u32 phys_addr_log_buffer;
	u32 total_log_entries;
	u32 dequeue_ptr;
	u32 dummy_var[2];
	u32 fwimg_len;
	u8 magic[8];
	u32 ss_low_power_entry_timeout;
	u8 num_hsic_port;
	u8 padding[139]; /* Pad to 256 bytes */
};

struct tegra_xhci_phy_type {
	const char *name;
	unsigned int num;
};

struct tegra_xhci_soc_data {
	const char *firmware_file;
	const char * const *supply_names;
	unsigned int num_supplies;
	const struct tegra_xhci_phy_type *phy_types;
	unsigned int num_types;
	bool scale_ss_clock;
};

struct tegra_xhci_hcd {
	struct device *dev;
	struct usb_hcd *hcd;

	int irq;

	void __iomem *ipfs_base;
	struct regmap *fpci_regs;

	const struct tegra_xhci_soc_data *soc;

	struct regulator_bulk_data *supplies;

	struct clk *host_clk;
	struct clk *falc_clk;
	struct clk *ss_clk;
	struct clk *ss_src_clk;
	struct clk *hs_src_clk;
	struct clk *fs_src_clk;
	struct clk *pll_u_480m;
	struct clk *clk_m;
	struct clk *pll_e;
	struct clk *emc_clk;

	struct reset_control *host_rst;
	struct reset_control *ss_rst;

	struct phy **phys;
	unsigned int num_phys;

	struct work_struct mbox_req_work;
	struct tegra_xusb_mbox_msg mbox_req;
	struct mbox_client mbox_client;
	struct mbox_chan *mbox_chan;

	/* Firmware loading related */
	void *fw_data;
	size_t fw_size;
	dma_addr_t fw_dma_addr;
	bool fw_loaded;
};

static struct hc_driver __read_mostly tegra_xhci_hc_driver;

static inline struct tegra_xhci_hcd *
mbox_work_to_tegra(struct work_struct *work)
{
	return container_of(work, struct tegra_xhci_hcd, mbox_req_work);
}

static inline u32 fpci_readl(struct tegra_xhci_hcd *tegra, u32 addr)
{
	u32 val;

	regmap_read(tegra->fpci_regs, addr, &val);

	return val;
}

static inline void fpci_writel(struct tegra_xhci_hcd *tegra, u32 val, u32 addr)
{
	regmap_write(tegra->fpci_regs, addr, val);
}

static inline u32 ipfs_readl(struct tegra_xhci_hcd *tegra, u32 addr)
{
	return readl(tegra->ipfs_base + addr);
}

static inline void ipfs_writel(struct tegra_xhci_hcd *tegra, u32 val, u32 addr)
{
	writel(val, tegra->ipfs_base + addr);
}

static u32 csb_readl(struct tegra_xhci_hcd *tegra, u32 addr)
{
	u32 page, offset;

	page = CSB_PAGE_SELECT(addr);
	offset = CSB_PAGE_OFFSET(addr);
	fpci_writel(tegra, page, XUSB_CFG_ARU_C11_CSBRANGE);
	return fpci_readl(tegra, XUSB_CFG_CSB_BASE_ADDR + offset);
}

static void csb_writel(struct tegra_xhci_hcd *tegra, u32 val, u32 addr)
{
	u32 page, offset;

	page = CSB_PAGE_SELECT(addr);
	offset = CSB_PAGE_OFFSET(addr);
	fpci_writel(tegra, page, XUSB_CFG_ARU_C11_CSBRANGE);
	fpci_writel(tegra, val, XUSB_CFG_CSB_BASE_ADDR + offset);
}

static void tegra_xhci_ipfs_config(struct tegra_xhci_hcd *tegra)
{
	u32 reg;

	reg = ipfs_readl(tegra, IPFS_XUSB_HOST_CONFIGURATION_0);
	reg |= IPFS_EN_FPCI;
	ipfs_writel(tegra, reg, IPFS_XUSB_HOST_CONFIGURATION_0);
	udelay(10);

	/* Program BAR0 space */
	reg = fpci_readl(tegra, XUSB_CFG_4);
	reg &= ~(XUSB_BASE_ADDR_MASK << XUSB_BASE_ADDR_SHIFT);
	reg |= tegra->hcd->rsrc_start & (XUSB_BASE_ADDR_MASK <<
					 XUSB_BASE_ADDR_SHIFT);
	fpci_writel(tegra, reg, XUSB_CFG_4);
	usleep_range(100, 200);

	/* Enable bus master */
	reg = fpci_readl(tegra, XUSB_CFG_1);
	reg |= XUSB_IO_SPACE_EN | XUSB_MEM_SPACE_EN | XUSB_BUS_MASTER_EN;
	fpci_writel(tegra, reg, XUSB_CFG_1);

	/* Enable interrupt assertion */
	reg = ipfs_readl(tegra, IPFS_XUSB_HOST_INTR_MASK_0);
	reg |= IPFS_IP_INT_MASK;
	ipfs_writel(tegra, reg, IPFS_XUSB_HOST_INTR_MASK_0);

	/* Set hysteresis */
	ipfs_writel(tegra, 0x80, IPFS_XUSB_HOST_CLKGATE_HYSTERESIS_0);
}

static int tegra_xhci_load_firmware(struct tegra_xhci_hcd *tegra)
{
	struct device *dev = tegra->dev;
	struct tegra_xhci_fw_cfgtbl *cfg_tbl;
	struct tm fw_tm;
	u32 val, code_tag_blocks, code_size_blocks;
	u64 fw_base;
	time_t fw_time;
	unsigned long timeout;

	if (csb_readl(tegra, XUSB_CSB_MP_ILOAD_BASE_LO) != 0) {
		dev_info(dev, "Firmware already loaded, Falcon state 0x%x\n",
			 csb_readl(tegra, XUSB_FALC_CPUCTL));
		return 0;
	}

	cfg_tbl = (struct tegra_xhci_fw_cfgtbl *)tegra->fw_data;

	/* Program the size of DFI into ILOAD_ATTR. */
	csb_writel(tegra, tegra->fw_size, XUSB_CSB_MP_ILOAD_ATTR);

	/*
	 * Boot code of the firmware reads the ILOAD_BASE registers
	 * to get to the start of the DFI in system memory.
	 */
	fw_base = tegra->fw_dma_addr + sizeof(*cfg_tbl);
	csb_writel(tegra, fw_base, XUSB_CSB_MP_ILOAD_BASE_LO);
	csb_writel(tegra, fw_base >> 32, XUSB_CSB_MP_ILOAD_BASE_HI);

	/* Set BOOTPATH to 1 in APMAP. */
	csb_writel(tegra, APMAP_BOOTPATH, XUSB_CSB_MP_APMAP);

	/* Invalidate L2IMEM. */
	csb_writel(tegra, L2IMEMOP_INVALIDATE_ALL, XUSB_CSB_MP_L2IMEMOP_TRIG);

	/*
	 * Initiate fetch of bootcode from system memory into L2IMEM.
	 * Program bootcode location and size in system memory.
	 */
	code_tag_blocks = DIV_ROUND_UP(le32_to_cpu(cfg_tbl->boot_codetag),
				       IMEM_BLOCK_SIZE);
	code_size_blocks = DIV_ROUND_UP(le32_to_cpu(cfg_tbl->boot_codesize),
					IMEM_BLOCK_SIZE);
	val = ((code_tag_blocks & L2IMEMOP_SIZE_SRC_OFFSET_MASK) <<
	       L2IMEMOP_SIZE_SRC_OFFSET_SHIFT) |
	      ((code_size_blocks & L2IMEMOP_SIZE_SRC_COUNT_MASK) <<
	       L2IMEMOP_SIZE_SRC_COUNT_SHIFT);
	csb_writel(tegra, val, XUSB_CSB_MP_L2IMEMOP_SIZE);

	/* Trigger L2IMEM load operation. */
	csb_writel(tegra, L2IMEMOP_LOAD_LOCKED_RESULT,
		   XUSB_CSB_MP_L2IMEMOP_TRIG);

	/* Setup Falcon auto-fill. */
	csb_writel(tegra, code_size_blocks, XUSB_FALC_IMFILLCTL);

	val = ((code_tag_blocks & IMFILLRNG1_TAG_MASK) <<
	       IMFILLRNG1_TAG_LO_SHIFT) |
	      (((code_size_blocks + code_tag_blocks) & IMFILLRNG1_TAG_MASK) <<
	       IMFILLRNG1_TAG_HI_SHIFT);
	csb_writel(tegra, val, XUSB_FALC_IMFILLRNG1);

	csb_writel(tegra, 0, XUSB_FALC_DMACTL);
	msleep(50);

	csb_writel(tegra, le32_to_cpu(cfg_tbl->boot_codetag),
		   XUSB_FALC_BOOTVEC);

	/* Boot Falcon CPU and wait for it to enter the STOPPED (idle) state. */
	timeout = jiffies + msecs_to_jiffies(5);
	csb_writel(tegra, CPUCTL_STARTCPU, XUSB_FALC_CPUCTL);
	while (time_before(jiffies, timeout)) {
		if (csb_readl(tegra, XUSB_FALC_CPUCTL) == CPUCTL_STATE_STOPPED)
			break;
		usleep_range(100, 200);
	}
	if (csb_readl(tegra, XUSB_FALC_CPUCTL) != CPUCTL_STATE_STOPPED) {
		dev_err(dev, "Falcon failed to start, state: %#x\n",
			csb_readl(tegra, XUSB_FALC_CPUCTL));
		return -EIO;
	}

	fw_time = le32_to_cpu(cfg_tbl->fwimg_created_time);
	time_to_tm(fw_time, 0, &fw_tm);
	dev_info(dev, "Firmware timestamp: %ld-%02d-%02d %02d:%02d:%02d UTC\n",
		 fw_tm.tm_year + 1900, fw_tm.tm_mon + 1, fw_tm.tm_mday,
		 fw_tm.tm_hour, fw_tm.tm_min, fw_tm.tm_sec);

	return 0;
}

static int tegra_xhci_set_ss_clk(struct tegra_xhci_hcd *tegra,
				 unsigned long rate)
{
	struct clk *clk = tegra->ss_src_clk;
	unsigned long new_parent_rate, old_parent_rate;
	int ret, div;

	if (clk_get_rate(clk) == rate)
		return 0;

	switch (rate) {
	case TEGRA_XHCI_SS_CLK_HIGH_SPEED:
		/*
		 * Reparent to PLLU_480M. Set divider first to avoid
		 * overclocking.
		 */
		old_parent_rate = clk_get_rate(clk_get_parent(clk));
		new_parent_rate = clk_get_rate(tegra->pll_u_480m);
		div = new_parent_rate / rate;
		ret = clk_set_rate(clk, old_parent_rate / div);
		if (ret)
			return ret;
		ret = clk_set_parent(clk, tegra->pll_u_480m);
		if (ret)
			return ret;
		/*
		 * The rate should already be correct, but set it again just
		 * to be sure.
		 */
		ret = clk_set_rate(clk, rate);
		if (ret)
			return ret;
		break;
	case TEGRA_XHCI_SS_CLK_LOW_SPEED:
		/* Reparent to CLK_M */
		ret = clk_set_parent(clk, tegra->clk_m);
		if (ret)
			return ret;
		ret = clk_set_rate(clk, rate);
		if (ret)
			return ret;
		break;
	default:
		dev_err(tegra->dev, "Invalid SS rate: %lu\n", rate);
		return -EINVAL;
	}

	if (clk_get_rate(clk) != rate) {
		dev_err(tegra->dev, "SS clock doesn't match requested rate\n");
		return -EINVAL;
	}

	return 0;
}

static int tegra_xhci_clk_enable(struct tegra_xhci_hcd *tegra)
{
	int ret;

	ret = clk_prepare_enable(tegra->pll_e);
	if (ret < 0)
		return ret;
	ret = clk_prepare_enable(tegra->falc_clk);
	if (ret < 0)
		goto disable_plle;
	ret = clk_prepare_enable(tegra->fs_src_clk);
	if (ret < 0)
		goto disable_falc;
	ret = clk_prepare_enable(tegra->hs_src_clk);
	if (ret < 0)
		goto disable_fs_src;
	ret = clk_prepare_enable(tegra->emc_clk);
	if (ret < 0)
		goto disable_hs_src;
	clk_set_rate(tegra->emc_clk, 0);
	if (tegra->soc->scale_ss_clock) {
		ret = tegra_xhci_set_ss_clk(tegra,
					    TEGRA_XHCI_SS_CLK_HIGH_SPEED);
		if (ret < 0)
			goto disable_emc;
	}

	return 0;

disable_emc:
	clk_disable_unprepare(tegra->emc_clk);
disable_hs_src:
	clk_disable_unprepare(tegra->hs_src_clk);
disable_fs_src:
	clk_disable_unprepare(tegra->fs_src_clk);
disable_falc:
	clk_disable_unprepare(tegra->falc_clk);
disable_plle:
	clk_disable_unprepare(tegra->pll_e);
	return ret;
}

static void tegra_xhci_clk_disable(struct tegra_xhci_hcd *tegra)
{
	clk_disable_unprepare(tegra->pll_e);
	clk_disable_unprepare(tegra->falc_clk);
	clk_disable_unprepare(tegra->fs_src_clk);
	clk_disable_unprepare(tegra->hs_src_clk);
	clk_disable_unprepare(tegra->emc_clk);
}

static int tegra_xhci_phy_enable(struct tegra_xhci_hcd *tegra)
{
	unsigned int i;
	int ret;

	for (i = 0; i < tegra->num_phys; i++) {
		ret = phy_init(tegra->phys[i]);
		if (ret)
			goto disable_phy;
		ret = phy_power_on(tegra->phys[i]);
		if (ret) {
			phy_exit(tegra->phys[i]);
			goto disable_phy;
		}
	}

	return 0;
disable_phy:
	for (; i > 0; i--) {
		phy_power_off(tegra->phys[i - 1]);
		phy_exit(tegra->phys[i - 1]);
	}
	return ret;
}

static void tegra_xhci_phy_disable(struct tegra_xhci_hcd *tegra)
{
	unsigned int i;

	for (i = 0; i < tegra->num_phys; i++) {
		phy_power_off(tegra->phys[i]);
		phy_exit(tegra->phys[i]);
	}
}

static bool is_host_mbox_message(u32 cmd)
{
	switch (cmd) {
	case MBOX_CMD_INC_SSPI_CLOCK:
	case MBOX_CMD_DEC_SSPI_CLOCK:
	case MBOX_CMD_INC_FALC_CLOCK:
	case MBOX_CMD_DEC_FALC_CLOCK:
	case MBOX_CMD_SET_BW:
		return true;
	default:
		return false;
	}
}

static void tegra_xhci_mbox_work(struct work_struct *work)
{
	struct tegra_xhci_hcd *tegra = mbox_work_to_tegra(work);
	struct tegra_xusb_mbox_msg *msg = &tegra->mbox_req;
	struct tegra_xusb_mbox_msg resp;
	unsigned long freq;
	int ret;

	resp.cmd = 0;
	switch (msg->cmd) {
	case MBOX_CMD_INC_SSPI_CLOCK:
	case MBOX_CMD_DEC_SSPI_CLOCK:
		if (tegra->soc->scale_ss_clock) {
			ret = tegra_xhci_set_ss_clk(tegra, msg->data * 1000);
			resp.data = clk_get_rate(tegra->ss_src_clk) / 1000;
		} else {
			ret = 0;
			resp.data = msg->data;
		}
		if (ret)
			resp.cmd = MBOX_CMD_NAK;
		else
			resp.cmd = MBOX_CMD_ACK;
		break;
	case MBOX_CMD_INC_FALC_CLOCK:
	case MBOX_CMD_DEC_FALC_CLOCK:
		resp.data = clk_get_rate(tegra->falc_clk) / 1000;
		if (resp.data != msg->data)
			resp.cmd = MBOX_CMD_NAK;
		else
			resp.cmd = MBOX_CMD_ACK;
		break;
	case MBOX_CMD_SET_BW:
		freq = tegra_emc_bw_to_freq_req(msg->data << 10) * 1000;
		clk_set_rate(tegra->emc_clk, freq);
		break;
	default:
		break;
	}

	if (resp.cmd)
		mbox_send_message(tegra->mbox_chan, &resp);
}

static void tegra_xhci_mbox_rx(struct mbox_client *cl, void *data)
{
	struct tegra_xhci_hcd *tegra = dev_get_drvdata(cl->dev);
	struct tegra_xusb_mbox_msg *msg = data;

	if (is_host_mbox_message(msg->cmd)) {
		tegra->mbox_req = *msg;
		schedule_work(&tegra->mbox_req_work);
	}
}

static void tegra_xhci_quirks(struct device *dev, struct xhci_hcd *xhci)
{
	xhci->quirks |= XHCI_PLAT;
}

static int tegra_xhci_setup(struct usb_hcd *hcd)
{
	return xhci_gen_setup(hcd, tegra_xhci_quirks);
}

static const char * const tegra124_supply_names[] = {
	"avddio-pex",
	"dvddio-pex",
	"avdd-usb",
	"avdd-pll-utmip",
	"avdd-pll-erefe",
	"avdd-usb-ss-pll",
	"hvdd-usb-ss",
	"hvdd-usb-ss-pll-e",
};

static const struct tegra_xhci_phy_type tegra124_phy_types[] = {
	{ .name = "usb3", .num = 2, },
	{ .name = "utmi", .num = 3, },
	{ .name = "hsic", .num = 2, },
};

static const struct tegra_xhci_soc_data tegra124_soc_data = {
	.firmware_file = "nvidia/tegra124/xusb.bin",
	.supply_names = tegra124_supply_names,
	.num_supplies = ARRAY_SIZE(tegra124_supply_names),
	.phy_types = tegra124_phy_types,
	.num_types = ARRAY_SIZE(tegra124_phy_types),
	.scale_ss_clock = true,
};
MODULE_FIRMWARE("nvidia/tegra124/xusb.bin");

static const char * const tegra210_supply_names[] = {
	"dvddio-pex",
	"hvddio-pex",
	"avdd-usb",
	"avdd-pll-utmip",
	"avdd-pll-uerefe",
	"dvdd-usb-ss-pll",
	"hvdd-usb-ss-pll-e",
};

static const struct tegra_xhci_phy_type tegra210_phy_types[] = {
	{ .name = "usb3", .num = 4, },
	{ .name = "utmi", .num = 4, },
	{ .name = "hsic", .num = 1, },
};

static const struct tegra_xhci_soc_data tegra210_soc_data = {
	.firmware_file = "nvidia/tegra210/xusb.bin",
	.supply_names = tegra210_supply_names,
	.num_supplies = ARRAY_SIZE(tegra210_supply_names),
	.phy_types = tegra210_phy_types,
	.num_types = ARRAY_SIZE(tegra210_phy_types),
	.scale_ss_clock = false,
};
MODULE_FIRMWARE("nvidia/tegra210/xusb.bin");

static const struct of_device_id tegra_xhci_of_match[] = {
	{ .compatible = "nvidia,tegra124-xhci", .data = &tegra124_soc_data },
	{ .compatible = "nvidia,tegra210-xhci", .data = &tegra210_soc_data },
	{ },
};
MODULE_DEVICE_TABLE(of, tegra_xhci_of_match);

static void tegra_xhci_probe_finish(const struct firmware *fw, void *context)
{
	struct tegra_xhci_hcd *tegra = context;
	struct device *dev = tegra->dev;
	struct xhci_hcd *xhci = NULL;
	struct tegra_xhci_fw_cfgtbl *cfg_tbl;
	struct tegra_xusb_mbox_msg msg;
	int ret;

	if (!fw)
		goto put_usb2_hcd;

	/* Load Falcon controller with its firmware. */
	cfg_tbl = (struct tegra_xhci_fw_cfgtbl *)fw->data;
	tegra->fw_size = le32_to_cpu(cfg_tbl->fwimg_len);
	tegra->fw_data = dma_alloc_coherent(dev, tegra->fw_size,
					    &tegra->fw_dma_addr,
					    GFP_KERNEL);
	if (!tegra->fw_data)
		goto put_usb2_hcd;
	memcpy(tegra->fw_data, fw->data, tegra->fw_size);

	ret = tegra_xhci_load_firmware(tegra);
	if (ret < 0)
		goto put_usb2_hcd;

	ret = usb_add_hcd(tegra->hcd, tegra->irq, IRQF_SHARED);
	if (ret < 0)
		goto put_usb2_hcd;
	device_wakeup_enable(tegra->hcd->self.controller);

	/*
	 * USB 2.0 roothub is stored in drvdata now. Swap it with the Tegra HCD.
	 */
	tegra->hcd = dev_get_drvdata(dev);
	dev_set_drvdata(dev, tegra);
	xhci = hcd_to_xhci(tegra->hcd);
	xhci->shared_hcd = usb_create_shared_hcd(&tegra_xhci_hc_driver,
						 dev, dev_name(dev),
						 tegra->hcd);
	if (!xhci->shared_hcd)
		goto dealloc_usb2_hcd;

	ret = usb_add_hcd(xhci->shared_hcd, tegra->irq, IRQF_SHARED);
	if (ret < 0)
		goto put_usb3_hcd;

	/* Enable firmware messages from controller. */
	msg.cmd = MBOX_CMD_MSG_ENABLED;
	msg.data = 0;
	ret = mbox_send_message(tegra->mbox_chan, &msg);
	if (ret < 0)
		goto dealloc_usb3_hcd;

	tegra->fw_loaded = true;
	release_firmware(fw);
	return;

	/* Free up as much as we can and wait to be unbound. */
dealloc_usb3_hcd:
	usb_remove_hcd(xhci->shared_hcd);
put_usb3_hcd:
	usb_put_hcd(xhci->shared_hcd);
dealloc_usb2_hcd:
	usb_remove_hcd(tegra->hcd);
	kfree(xhci);
put_usb2_hcd:
	usb_put_hcd(tegra->hcd);
	tegra->hcd = NULL;
	release_firmware(fw);
}

static int tegra_xhci_probe(struct platform_device *pdev)
{
	const struct of_device_id *match;
	struct tegra_xhci_hcd *tegra;
	struct resource *res;
	struct usb_hcd *hcd;
	struct phy *phy;
	unsigned int i, j, k;
	int ret;

	BUILD_BUG_ON(sizeof(struct tegra_xhci_fw_cfgtbl) != 256);

	tegra = devm_kzalloc(&pdev->dev, sizeof(*tegra), GFP_KERNEL);
	if (!tegra)
		return -ENOMEM;
	tegra->dev = &pdev->dev;
	platform_set_drvdata(pdev, tegra);

	match = of_match_device(tegra_xhci_of_match, &pdev->dev);
	tegra->soc = match->data;

	hcd = usb_create_hcd(&tegra_xhci_hc_driver, &pdev->dev,
			     dev_name(&pdev->dev));
	if (!hcd)
		return -ENOMEM;
	tegra->hcd = hcd;

	tegra->fpci_regs = dev_get_drvdata(pdev->dev.parent);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	hcd->regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(hcd->regs)) {
		ret = PTR_ERR(hcd->regs);
		goto put_hcd;
	}
	hcd->rsrc_start = res->start;
	hcd->rsrc_len = resource_size(res);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	tegra->ipfs_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(tegra->ipfs_base)) {
		ret = PTR_ERR(tegra->ipfs_base);
		goto put_hcd;
	}

	tegra->irq = platform_get_irq(pdev, 0);
	if (tegra->irq < 0) {
		ret = tegra->irq;
		goto put_hcd;
	}

	tegra->host_rst = devm_reset_control_get(&pdev->dev, "xusb_host");
	if (IS_ERR(tegra->host_rst)) {
		ret = PTR_ERR(tegra->host_rst);
		goto put_hcd;
	}
	tegra->ss_rst = devm_reset_control_get(&pdev->dev, "xusb_ss");
	if (IS_ERR(tegra->ss_rst)) {
		ret = PTR_ERR(tegra->ss_rst);
		goto put_hcd;
	}

	tegra->host_clk = devm_clk_get(&pdev->dev, "xusb_host");
	if (IS_ERR(tegra->host_clk)) {
		ret = PTR_ERR(tegra->host_clk);
		goto put_hcd;
	}
	tegra->falc_clk = devm_clk_get(&pdev->dev, "xusb_falcon_src");
	if (IS_ERR(tegra->falc_clk)) {
		ret = PTR_ERR(tegra->falc_clk);
		goto put_hcd;
	}
	tegra->ss_clk = devm_clk_get(&pdev->dev, "xusb_ss");
	if (IS_ERR(tegra->ss_clk)) {
		ret = PTR_ERR(tegra->ss_clk);
		goto put_hcd;
	}
	tegra->ss_src_clk = devm_clk_get(&pdev->dev, "xusb_ss_src");
	if (IS_ERR(tegra->ss_src_clk)) {
		ret = PTR_ERR(tegra->ss_src_clk);
		goto put_hcd;
	}
	tegra->hs_src_clk = devm_clk_get(&pdev->dev, "xusb_hs_src");
	if (IS_ERR(tegra->hs_src_clk)) {
		ret = PTR_ERR(tegra->hs_src_clk);
		goto put_hcd;
	}
	tegra->fs_src_clk = devm_clk_get(&pdev->dev, "xusb_fs_src");
	if (IS_ERR(tegra->fs_src_clk)) {
		ret = PTR_ERR(tegra->fs_src_clk);
		goto put_hcd;
	}
	tegra->pll_u_480m = devm_clk_get(&pdev->dev, "pll_u_480m");
	if (IS_ERR(tegra->pll_u_480m)) {
		ret = PTR_ERR(tegra->pll_u_480m);
		goto put_hcd;
	}
	tegra->clk_m = devm_clk_get(&pdev->dev, "clk_m");
	if (IS_ERR(tegra->clk_m)) {
		ret = PTR_ERR(tegra->clk_m);
		goto put_hcd;
	}
	tegra->pll_e = devm_clk_get(&pdev->dev, "pll_e");
	if (IS_ERR(tegra->pll_e)) {
		ret = PTR_ERR(tegra->pll_e);
		goto put_hcd;
	}

	ret = tegra_powergate_sequence_power_up(TEGRA_POWERGATE_XUSBA,
						tegra->ss_clk,
						tegra->ss_rst);
	if (ret < 0)
		goto put_hcd;
	ret = tegra_powergate_sequence_power_up(TEGRA_POWERGATE_XUSBC,
						tegra->host_clk,
						tegra->host_rst);
	if (ret < 0)
		goto powergate_xusba;

	tegra->emc_clk = devm_clk_get(&pdev->dev, "emc");
	if (IS_ERR(tegra->emc_clk)) {
		dev_warn(&pdev->dev, "EMC scaling disabled\n");
		tegra->emc_clk = NULL;
	}
	ret = tegra_xhci_clk_enable(tegra);
	if (ret)
		goto powergate_xusbc;

	tegra->supplies = devm_kcalloc(&pdev->dev, tegra->soc->num_supplies,
				       sizeof(*tegra->supplies), GFP_KERNEL);
	if (!tegra->supplies) {
		ret = -ENOMEM;
		goto disable_clk;
	}
	for (i = 0; i < tegra->soc->num_supplies; i++)
		tegra->supplies[i].supply = tegra->soc->supply_names[i];
	ret = devm_regulator_bulk_get(&pdev->dev, tegra->soc->num_supplies,
				      tegra->supplies);
	if (ret)
		goto disable_clk;
	ret = regulator_bulk_enable(tegra->soc->num_supplies, tegra->supplies);
	if (ret)
		goto disable_clk;

	INIT_WORK(&tegra->mbox_req_work, tegra_xhci_mbox_work);
	tegra->mbox_client.dev = &pdev->dev;
	tegra->mbox_client.tx_block = true;
	tegra->mbox_client.tx_tout = 0;
	tegra->mbox_client.rx_callback = tegra_xhci_mbox_rx;
	tegra->mbox_chan = mbox_request_channel(&tegra->mbox_client, 0);
	if (IS_ERR(tegra->mbox_chan)) {
		ret = PTR_ERR(tegra->mbox_chan);
		goto disable_regulator;
	}

	for (i = 0; i < tegra->soc->num_types; i++)
		tegra->num_phys += tegra->soc->phy_types[i].num;
	tegra->phys = devm_kcalloc(&pdev->dev, tegra->num_phys,
				   sizeof(*tegra->phys), GFP_KERNEL);
	if (!tegra->phys) {
		ret = -ENOMEM;
		goto put_mbox;
	}
	for (i = 0, k = 0; i < tegra->soc->num_types; i++) {
		char prop[8];

		for (j = 0; j < tegra->soc->phy_types[i].num; j++) {
			snprintf(prop, sizeof(prop), "%s-%d",
				 tegra->soc->phy_types[i].name, j);
			phy = devm_phy_optional_get(&pdev->dev, prop);
			if (IS_ERR(phy)) {
				ret = PTR_ERR(phy);
				goto put_mbox;
			}
			tegra->phys[k++] = phy;
		}
	}

	tegra_xhci_ipfs_config(tegra);

	ret = tegra_xhci_phy_enable(tegra);
	if (ret < 0)
		goto put_mbox;

	ret = request_firmware_nowait(THIS_MODULE, true,
				      tegra->soc->firmware_file,
				      tegra->dev, GFP_KERNEL, tegra,
				      tegra_xhci_probe_finish);
	if (ret < 0)
		goto disable_phy;

	return 0;

disable_phy:
	tegra_xhci_phy_disable(tegra);
put_mbox:
	mbox_free_channel(tegra->mbox_chan);
disable_regulator:
	regulator_bulk_disable(tegra->soc->num_supplies, tegra->supplies);
disable_clk:
	tegra_xhci_clk_disable(tegra);
powergate_xusbc:
	tegra_powergate_sequence_power_down(TEGRA_POWERGATE_XUSBC,
					    tegra->host_clk,
					    tegra->host_rst);
powergate_xusba:
	tegra_powergate_sequence_power_down(TEGRA_POWERGATE_XUSBA,
					    tegra->ss_clk,
					    tegra->ss_rst);
put_hcd:
	usb_put_hcd(hcd);
	return ret;
}

static int tegra_xhci_remove(struct platform_device *pdev)
{
	struct tegra_xhci_hcd *tegra = platform_get_drvdata(pdev);
	struct usb_hcd *hcd = tegra->hcd;
	struct xhci_hcd *xhci;

	if (tegra->fw_loaded) {
		xhci = hcd_to_xhci(hcd);
		usb_remove_hcd(xhci->shared_hcd);
		usb_put_hcd(xhci->shared_hcd);
		usb_remove_hcd(hcd);
		usb_put_hcd(hcd);
		kfree(xhci);
	} else if (hcd) {
		/* Unbound after probe(), but before firmware loading. */
		usb_put_hcd(hcd);
	}

	if (tegra->fw_data)
		dma_free_coherent(tegra->dev, tegra->fw_size, tegra->fw_data,
				  tegra->fw_dma_addr);

	cancel_work_sync(&tegra->mbox_req_work);
	mbox_free_channel(tegra->mbox_chan);
	tegra_xhci_phy_disable(tegra);
	regulator_bulk_disable(tegra->soc->num_supplies, tegra->supplies);
	tegra_powergate_sequence_power_down(TEGRA_POWERGATE_XUSBC,
					    tegra->host_clk,
					    tegra->host_rst);
	tegra_powergate_sequence_power_down(TEGRA_POWERGATE_XUSBA,
					    tegra->ss_clk,
					    tegra->ss_rst);
	tegra_xhci_clk_disable(tegra);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int tegra_xhci_suspend(struct device *dev)
{
	struct tegra_xhci_hcd *tegra = dev_get_drvdata(dev);
	struct xhci_hcd *xhci = hcd_to_xhci(tegra->hcd);
	bool wakeup = device_may_wakeup(dev);

	/* TODO: Powergate controller across suspend/resume. */
	return xhci_suspend(xhci, wakeup);
}

static int tegra_xhci_resume(struct device *dev)
{
	struct tegra_xhci_hcd *tegra = dev_get_drvdata(dev);
	struct xhci_hcd *xhci = hcd_to_xhci(tegra->hcd);

	return xhci_resume(xhci, 0);
}
#endif

static const struct dev_pm_ops tegra_xhci_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(tegra_xhci_suspend, tegra_xhci_resume)
};

static struct platform_driver tegra_xhci_driver = {
	.probe = tegra_xhci_probe,
	.remove = tegra_xhci_remove,
	.driver = {
		.name = "tegra-xhci",
		.pm = &tegra_xhci_pm_ops,
		.of_match_table = tegra_xhci_of_match,
	},
};

static const struct xhci_driver_overrides tegra_xhci_overrides __initconst = {
	.extra_priv_size = sizeof(struct xhci_hcd),
	.reset = tegra_xhci_setup,
};

static int __init tegra_xhci_init(void)
{
	xhci_init_driver(&tegra_xhci_hc_driver, &tegra_xhci_overrides);
	return platform_driver_register(&tegra_xhci_driver);
}
module_init(tegra_xhci_init);

static void __exit tegra_xhci_exit(void)
{
	platform_driver_unregister(&tegra_xhci_driver);
}
module_exit(tegra_xhci_exit);

MODULE_AUTHOR("Andrew Bresticker <abrestic@chromium.org>");
MODULE_DESCRIPTION("NVIDIA Tegra xHCI host-controller driver");
MODULE_LICENSE("GPL v2");
