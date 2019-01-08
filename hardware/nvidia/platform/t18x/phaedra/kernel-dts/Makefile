old-dtb := $(dtb-y)
dtb-y :=
makefile-path := platform/t18x/phaedra/kernel-dts

dtb-$(CONFIG_ARCH_TEGRA_18x_SOC) += tegra186-phaedra-p0.dtb
dtb-$(CONFIG_ARCH_TEGRA_18x_SOC) += tegra186-phaedra-p1.dtb
dtb-$(CONFIG_ARCH_TEGRA_18x_SOC) += tegra186-phaedra-p2.dtb
dtb-$(CONFIG_ARCH_TEGRA_18x_SOC) += tegra186-phaedra-p2b.dtb
dtb-$(CONFIG_ARCH_TEGRA_18x_SOC) += tegra186-phaedra-p3a.dtb
dtb-$(CONFIG_ARCH_TEGRA_18x_SOC) += tegra186-phaedra-p3b.dtb
dtb-$(CONFIG_ARCH_TEGRA_18x_SOC) += tegra186-phaedra-p4b.dtb
dtb-$(CONFIG_ARCH_TEGRA_18x_SOC) += tegra186-phaedra-p5a.dtb
dtb-$(CONFIG_ARCH_TEGRA_18x_SOC) += tegra186-phaedra-p6a.dtb
dtb-$(CONFIG_ARCH_TEGRA_18x_SOC) += tegra186-phaedra-r0a.dtb

ifneq ($(dtb-y),)
dtb-y := $(addprefix $(makefile-path)/,$(dtb-y))
dts-include += $(makefile-path)
endif

dtb-y += $(old-dtb)
