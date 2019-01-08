#
# This file picks up the following defconfigs:
# The config defined by NV_BUILD_KERNEL_CONFIG_IN (e.g. tegra18_android_defconfig)
# A security config file defined for this platform and build variant (e.g. ext_config/security-tegra18_android-userdebug.config)
# An optional debug config defined for this platform (e.g. ext_config/debug-tegra18_android.config)
# An optional debug config defined for the product (PRODUCT_KERNEL_DEBUG_DEFCONFIG)
# Any additional kernel config defined by KERNEL_EXTRA_CONFIG in BoardConfig.mk
#
# The output of this file is ml_combined_defconfig which feeds into the linux kernel's
# make defconfig which produces the .config file.
#

ifeq ($(NV_BUILD_KERNEL_CONFIG_IN),)
$(error NV_BUILD_KERNEL_CONFIG_IN is not defined)
endif
ifeq ($(NV_BUILD_KERNEL_CONFIG_OUT),)
$(error NV_BUILD_KERNEL_CONFIG_OUT is not defined)
endif

KERNEL_DEFCONFIG:=$(notdir $(NV_BUILD_KERNEL_CONFIG_IN))

# Note: on nvidia platforms, we cannot use $(dir $(NV_BUILD_KERNEL_CONFIG_OUT))
# for the directory of our generated defconfig $(TARGET_DEFCONFIG). The
# name of the directory also should not match the name of our defconfig.
DEFCONFIGSRC                    := $(dir $(NV_BUILD_KERNEL_CONFIG_IN))
DEFCONFIGDEST                   := $(dir $(NV_BUILD_KERNEL_CONFIG_OUT))generated
EXTDEFCONFIGSRC	                := ${DEFCONFIGSRC}/ext_config
PRODUCT_SPECIFIC_DEFCONFIGS     := $(DEFCONFIGSRC)/$(KERNEL_DEFCONFIG)
TARGET_DEFCONFIG                := $(DEFCONFIGDEST)/ml_combined_defconfig
KERNEL_DEBUG_DEFCONFIG          := $(EXTDEFCONFIGSRC)/debug-$(subst _defconfig,,$(KERNEL_DEFCONFIG)).config
PRODUCT_KERNEL_DEBUG_DEFCONFIG  := $(EXTDEFCONFIGSRC)/$(PRODUCT_DEBUG_DEFCONFIG)

# add debug config file for non-user build
ifneq ($(TARGET_BUILD_VARIANT), user)
ifneq ($(TARGET_NO_KERNEL_DEBUG), true)
ifneq ($(wildcard $(KERNEL_DEBUG_DEFCONFIG)),)
PRODUCT_SPECIFIC_DEFCONFIGS += $(KERNEL_DEBUG_DEFCONFIG)
# Add a product-specific debug defconfig, too
ifneq ($(PRODUCT_DEBUG_DEFCONFIG),)
PRODUCT_SPECIFIC_DEFCONFIGS += $(PRODUCT_KERNEL_DEBUG_DEFCONFIG)
endif
endif
endif
endif

ifneq ($(TARGET_ENABLE_ML_SECURITY),false)
ifeq ($(TARGET_BUILD_VARIANT),user)
PRODUCT_SPECIFIC_DEFCONFIGS += $(EXTDEFCONFIGSRC)/ml-security-prod.config
else
PRODUCT_SPECIFIC_DEFCONFIGS += $(EXTDEFCONFIGSRC)/ml-security-debug.config
endif
endif

ifneq ($(filter %_factory,$(TARGET_PRODUCT)),)
PRODUCT_SPECIFIC_DEFCONFIGS += $(EXTDEFCONFIGSRC)/ml-factory.config
endif

# append all additional configs
ifneq ($(KERNEL_EXTRA_CONFIG),)
PRODUCT_SPECIFIC_DEFCONFIGS += $(KERNEL_EXTRA_CONFIG:%=$(EXTDEFCONFIGSRC)/%.config)
endif


define do-make-defconfig
	$(hide) mkdir -p $(dir $(1))
	( perl -le 'print "# This file was automatically generated from:\n#\t" . join("\n#\t", @ARGV) . "\n"' $(2) && sed -e '$$s/$$/\n/' -s $(2) ) > $(1) || ( rm -f $(1) && false )
endef

#
# make combined defconfig file
#---------------------------------------
$(TARGET_DEFCONFIG): FORCE $(PRODUCT_SPECIFIC_DEFCONFIGS)
	$(call do-make-defconfig,$@,$(PRODUCT_SPECIFIC_DEFCONFIGS))

.PHONY: FORCE
FORCE:
