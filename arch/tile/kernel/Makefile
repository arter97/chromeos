#
# Makefile for the Linux/TILE kernel.
#

extra-y := vmlinux.lds head_$(BITS).o
obj-y := backtrace.o entry.o irq.o messaging.o \
	pci-dma.o proc.o process.o ptrace.o reboot.o \
	setup.o signal.o single_step.o stack.o sys.o \
	sysfs.o time.o traps.o vdso.o \
	intvec_$(BITS).o regs_$(BITS).o tile-desc_$(BITS).o

obj-$(CONFIG_HARDWALL)		+= hardwall.o
obj-$(CONFIG_COMPAT)		+= compat.o compat_signal.o
obj-$(CONFIG_SMP)		+= smpboot.o smp.o tlb.o
obj-$(CONFIG_MODULES)		+= module.o
obj-$(CONFIG_EARLY_PRINTK)	+= early_printk.o
obj-$(CONFIG_KEXEC)		+= machine_kexec.o relocate_kernel_$(BITS).o
ifdef CONFIG_TILEGX
obj-$(CONFIG_PCI)		+= pci_gx.o
else
obj-$(CONFIG_PCI)		+= pci.o
endif
obj-$(CONFIG_TILE_USB)		+= usb.o

obj-y				+= vdso/
