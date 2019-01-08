#
# Makefile for ML_SECURITY
#

#note : don't add files to the obj-y below, since this is included in each build regardless of the configs.
#       if you need something common to the ml-security that would only be included if "either" of the
#       ml-security features are set, add them to the common/Makefile
obj-y			:=

#ifdef example
#ifdef CONFIG_NO_BOOTMEM
#	obj-y		+= nobootmem.o
#else
#	obj-y		+= bootmem.o
#endif

obj-$(CONFIG_DEBUG_FS)          += mlsec_debugfs.o
obj-$(CONFIG_MLSEC_COMMON)      += common/
obj-$(CONFIG_MLSEC_ELF_SIGNING) += signing/
obj-$(CONFIG_MLSEC_SANDBOX)     += sandbox/
obj-y                           += os_hardenings/
