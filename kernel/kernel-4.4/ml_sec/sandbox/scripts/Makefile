profiles_dir = $(srctree)/../../nova/system/security/profiles/sandbox

#
# Build the host program that generates the sandbox profiles data.
#
hostprogs-y += gen_sandbox_profiles

HOSTCFLAGS_gen_sandbox_profiles.o := -I$(src)/include/host -DCONFIG_MLSEC_SANDBOX -DCONFIG_MLSEC_SANDBOX_DEVELOP

ifeq ($(CONFIG_COMPAT),y)
HOST_EXTRACFLAGS += -DCONFIG_COMPAT
endif

#
# Command line for pre-processing a .c file.
#
quiet_cmd_cc_inc_c = CPP $(quiet_modtag) $@
      cmd_cc_inc_c = $(CPP) $(c_flags) -D__ASSEMBLY__   -o $@ $<

#
# Create the syscall filters intermediate file.
#
$(obj)/syscall_table.inc: $(src)/template_syscall_table.c
	$(call if_changed_dep,cc_inc_c)

$(obj)/gen_sandbox_profiles: $(obj)/syscall_table.inc

targets += syscall_table.inc
clean-files += syscall_table.inc

ifeq ($(CONFIG_COMPAT),y)
$(obj)/syscall_table_compat.inc: $(src)/template_syscall_table_compat.c
	$(call if_changed_dep,cc_inc_c)

$(obj)/gen_sandbox_profiles: $(obj)/syscall_table_compat.inc

targets += syscall_table_compat.inc
clean-files += syscall_table_compat.inc
endif


#
# Command line for generating the sandbox_profiles.inc and
# the syscall filters .c files, which contain the actual filter masks.
#
quiet_cmd_sandbox_profiles = GEN  $@
      cmd_sandbox_profiles = $(obj)/gen_sandbox_profiles $(profiles_dir) $(obj)

$(obj)/sandbox_profiles.inc: $(profiles_dir) $(wildcard $(profiles_dir)/*.profile)
$(obj)/sandbox_profiles.inc: $(obj)/gen_sandbox_profiles
	$(call if_changed,sandbox_profiles)

$(obj)/syscall_filters.c: $(obj)/sandbox_profiles.inc
	@touch $@

targets += sandbox_profiles.inc syscall_filters.c
clean-files += sandbox_profiles.inc syscall_filters.c

ifeq ($(CONFIG_COMPAT),y)
$(obj)/syscall_filters_compat.c: $(obj)/sandbox_profiles.inc
	@touch $@

targets += syscall_filters_compat.c
clean-files += syscall_filters_compat.c
endif


#
# Build the actual sandbox profiles object.
#
obj-y := sandbox_profiles.o
$(obj)/sandbox_profiles.o: $(obj)/sandbox_profiles.inc
