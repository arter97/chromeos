#
# Makefile for memory devices
#

ifeq ($(CONFIG_DDR),y)
obj-$(CONFIG_OF)		+= of_memory.o
endif
obj-$(CONFIG_TI_EMIF)		+= emif.o
obj-$(CONFIG_TEGRA20_MC)	+= tegra20-mc.o
obj-$(CONFIG_TEGRA30_MC)	+= tegra30-mc.o
obj-$(CONFIG_TEGRA114_MC)	+= tegra114-mc.o
obj-$(CONFIG_TEGRA114_EMC)	+= tegra114-emc.o
obj-$(CONFIG_TEGRA124_MC)	+= tegra124-mc.o
obj-$(CONFIG_TEGRA124_EMC)	+= tegra124-emc.o
