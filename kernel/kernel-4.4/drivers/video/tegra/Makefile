GCOV_PROFILE := y
#subdir-ccflags-y := -Werror
EXTRA_CFLAGS += -I../nvhost/drivers/video/tegra/host
EXTRA_CFLAGS += -Idrivers/video/tegra/host
ifeq ($(CONFIG_ARM64),y)
subdir-ccflags-y += -Iarch/arm/mach-tegra/include
ccflags += -Iarch/arm/mach-tegra/include
endif
ifeq ($(CONFIG_ML_SFD),y)
ccflags-y += -I$(srctree)/../ml_graphics
ccflags-y += -I$(srctree)/../display/drivers/video/tegra/dc
endif


obj-$(CONFIG_TEGRA_GRHOST) += $(call tegra-path,nvhost/,drivers/video/tegra/host/)
obj-$(CONFIG_TEGRA_NVMAP)  += $(call tegra-path,nvmap/,drivers/video/tegra/nvmap/)
obj-$(CONFIG_TEGRA_DC)     += $(call tegra-path,display,drivers/video/tegra/dc/)
obj-$(CONFIG_FB_TEGRA)     += $(call tegra-path,display,drivers/video/tegra/fb.o)
obj-$(CONFIG_ML_SFD)     += $(call tegra-path,display,drivers/video/tegra/dc/ext/control.o)
obj-$(CONFIG_ML_SFD)     += $(call tegra-path,display,drivers/video/tegra/dc/ext/events.o)
obj-$(CONFIG_ML_SFD)     += $(call tegra-path,ml_graphics,)
obj-$(CONFIG_TEGRA_CAMERA_PLATFORM) += camera/
obj-$(CONFIG_TEGRA_GR_VIRTUALIZATION) += virt/
