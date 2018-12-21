#
# Makefile for ML_SECURITY's hardenings
#

ifeq ($(CONFIG_DEBUG_FS),y)

obj-$(CONFIG_GRKERNSEC_RANDSTRUCT) += tests/test_randstruct_debugfs.o
obj-$(CONFIG_THREAD_INFO_IN_TASK) += tests/test_thread_info_debugfs.o
obj-$(CONFIG_HARDENED_USERCOPY) += tests/test_usercopy_debugfs.o

endif

obj-$(CONFIG_MLSEC_ALLOW_PERMISSIVE_DEBUGGING) += debug/debugharden_debugfs.o