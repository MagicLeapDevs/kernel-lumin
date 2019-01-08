#
# Makefile for ML_SECURITY's sandbox
#

obj-y := scripts/
obj-y += sandbox.o syscall_inspect.o sandbox_lsm_hooks.o

obj-$(CONFIG_COMPAT) += syscall_inspect_compat.o
obj-$(CONFIG_MLSEC_SANDBOX_AUDIT) += sandbox_audit.o

ifneq ($(filter y,$(CONFIG_MLSEC_SANDBOX_AUDIT) $(CONFIG_MLSEC_SANDBOX_DEVELOP)),)
obj-y += syscall_name.o
obj-$(CONFIG_COMPAT) += syscall_name_compat.o
endif

obj-$(CONFIG_MLSEC_SANDBOX_DEVELOP) += dev/
