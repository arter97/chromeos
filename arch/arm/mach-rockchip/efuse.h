#ifndef _ARCH_ROCKCHIP_EFUSE_H_
#define _ARCH_ROCKCHIP_EFUSE_H_

#define EFUSE_CHIP_UID_LEN		16

struct platform_device;

int rockchip_efuse_get_cpuleakage(struct platform_device *efuse,
				unsigned int *value);

int rockchip_efuse_get_chip_version(struct platform_device *efuse,
				    unsigned int *version);

int rockchip_efuse_get_uid(struct platform_device *pdev, char *buf);

#endif /* _ARCH_ROCKCHIP_EFUSE_H_ */
